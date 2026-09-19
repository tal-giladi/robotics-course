"""13.15 — the detector node: image in, `vision_msgs/Detection2DArray` out, honestly measured.

    ros2 run karmel_vision detector_node --ros-args -p inference_ms:=150.0 -p image_qos:=sensor

Parameters worth playing with:

    image_qos     "sensor" (best effort, depth 5), "latest" (best effort, depth 1) or
                  "reliable10" (what a tutorial's default gives you)
    inference_ms  pretend the network takes this long, so a laptop can reproduce a Pi's behaviour
    scale         run the detector on a downscaled image; boxes are reported in ORIGINAL pixels
    detector      "colour" (no download, deterministic) or "ssdlite" (torchvision, 13.10)

It publishes `/detections` and, only while someone subscribes, an annotated `/detections/image`.
"""
from __future__ import annotations

import time
from collections import deque

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import Image
from vision_msgs.msg import BoundingBox2D, Detection2D, Detection2DArray, ObjectHypothesisWithPose

# HSV windows for the course's green bottle and blue mug (13.03's method, measured on the frames).
COLOUR_CLASSES = {
    "bottle": ((35, 60, 40), (85, 255, 255)),
    "cup": ((90, 60, 40), (125, 255, 255)),
}
MIN_AREA_PX = 200


def detect_by_colour(bgr: np.ndarray) -> list[tuple[str, float, tuple[float, float, float, float]]]:
    """The classical baseline of 13.03, kept because a baseline you can run beats a model you can't."""
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    out = []
    for name, (lo, hi) in COLOUR_CLASSES.items():
        mask = cv2.inRange(hsv, np.array(lo, np.uint8), np.array(hi, np.uint8))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for c in contours:
            area = cv2.contourArea(c)
            if area < MIN_AREA_PX:
                continue
            x, y, w, h = cv2.boundingRect(c)
            fill = area / float(max(w * h, 1))                    # how box-shaped the blob is
            out.append((name, float(min(0.99, 0.5 + 0.5 * fill)), (float(x), float(y), float(x + w), float(y + h))))
    return out


class DetectorNode(Node):
    def __init__(self) -> None:
        super().__init__("detector_node")
        self.declare_parameter("image_topic", "/camera/image_raw")
        self.declare_parameter("image_qos", "sensor")   # "sensor" | "latest" | "reliable10"
        self.declare_parameter("inference_ms", 0.0)
        self.declare_parameter("scale", 1.0)
        self.declare_parameter("detector", "colour")
        self.declare_parameter("score_threshold", 0.5)
        self.declare_parameter("report_every_s", 5.0)

        qos_name = self.get_parameter("image_qos").value
        if qos_name == "sensor":                                   # rclpy's sensor profile: BEST_EFFORT, depth 5
            qos = qos_profile_sensor_data
        elif qos_name == "latest":                                 # BEST_EFFORT, depth 1: always the newest frame
            qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        elif qos_name == "reliable10":                             # what a tutorial's default gives you
            qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        else:
            raise ValueError(f"image_qos must be 'sensor', 'latest' or 'reliable10', not {qos_name!r}")

        self.bridge = CvBridge()
        self.detect = self._build_detector(self.get_parameter("detector").value)
        self.pub = self.create_publisher(Detection2DArray, "/detections", 10)
        self.pub_debug = self.create_publisher(Image, "/detections/image", 1)
        self.sub = self.create_subscription(Image, self.get_parameter("image_topic").value, self.on_image, qos)
        self.ages: deque[float] = deque(maxlen=400)
        self.infer_ms: deque[float] = deque(maxlen=400)
        self.frames = 0
        self.t_report = time.perf_counter()
        self.get_logger().info(f"detector={self.get_parameter('detector').value} qos={qos_name} "
                               f"inference_ms={self.get_parameter('inference_ms').value}")

    def _build_detector(self, name: str):
        if name == "colour":
            return detect_by_colour
        if name == "ssdlite":                                      # 13.10's model, if torch is installed
            import torch
            from torchvision.models import detection as tvd
            weights = tvd.SSDLite320_MobileNet_V3_Large_Weights.DEFAULT
            model = tvd.ssdlite320_mobilenet_v3_large(weights=weights).eval()
            names = weights.meta["categories"]

            def run(bgr: np.ndarray):
                rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB).transpose(2, 0, 1)
                with torch.inference_mode():
                    out = model([torch.from_numpy(np.ascontiguousarray(rgb)).float() / 255.0])[0]
                return [(names[int(l)], float(s), tuple(float(v) for v in b))
                        for b, s, l in zip(out["boxes"], out["scores"], out["labels"])
                        if names[int(l)] in ("bottle", "cup")]
            return run
        raise ValueError(f"unknown detector {name!r}")

    def on_image(self, msg: Image) -> None:
        t_start = time.perf_counter()
        # desired_encoding='bgr8' makes cv_bridge convert whatever the driver publishes (rgb8,
        # mono8, bayer_*) into the BGR order OpenCV expects. 'passthrough' would hand you the
        # driver's own order and a red/blue swap you will chase for an afternoon (13.01).
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        scale = float(self.get_parameter("scale").value)
        small = cv2.resize(frame, None, fx=scale, fy=scale) if scale != 1.0 else frame

        busy_s = float(self.get_parameter("inference_ms").value) / 1000.0
        if busy_s > 0:                                             # stand-in for a real network
            end = time.perf_counter() + busy_s
            while time.perf_counter() < end:
                pass
        raw = self.detect(small)
        thr = float(self.get_parameter("score_threshold").value)
        dets = [(n, s, tuple(v / scale for v in b)) for n, s, b in raw if s >= thr]  # back to ORIGINAL pixels

        out = Detection2DArray()
        out.header = msg.header                                    # capture time and optical frame: never "now"
        for name, score, (x1, y1, x2, y2) in dets:
            d = Detection2D()
            d.header = msg.header
            d.id = name
            hyp = ObjectHypothesisWithPose()
            hyp.hypothesis.class_id = name
            hyp.hypothesis.score = score
            d.results.append(hyp)
            bbox = BoundingBox2D()
            bbox.center.position.x = (x1 + x2) / 2
            bbox.center.position.y = (y1 + y2) / 2
            bbox.size_x = x2 - x1
            bbox.size_y = y2 - y1
            d.bbox = bbox
            out.detections.append(d)
        self.pub.publish(out)

        self.infer_ms.append((time.perf_counter() - t_start) * 1000.0)
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.ages.append((self.get_clock().now().nanoseconds * 1e-9 - stamp) * 1000.0)
        self.frames += 1
        if self.pub_debug.get_subscription_count() > 0:            # encode only when watched (07.08)
            self.publish_debug(frame, dets, msg)
        self.report()

    def publish_debug(self, frame: np.ndarray, dets, msg: Image) -> None:
        img = frame.copy()
        for name, score, (x1, y1, x2, y2) in dets:
            cv2.rectangle(img, (int(x1), int(y1)), (int(x2), int(y2)), (0, 0, 255), 2)
            cv2.putText(img, f"{name} {score:.2f}", (int(x1), max(int(y1) - 4, 10)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
        out = self.bridge.cv2_to_imgmsg(img, encoding="bgr8")
        out.header = msg.header
        self.pub_debug.publish(out)

    def report(self) -> None:
        every = float(self.get_parameter("report_every_s").value)
        now = time.perf_counter()
        if every <= 0 or now - self.t_report < every:
            return
        hz = self.frames / (now - self.t_report)
        ages, infer = np.array(self.ages), np.array(self.infer_ms)
        self.get_logger().info(
            f"{hz:4.1f} Hz processed | age of the detection when published: "
            f"p50 {np.percentile(ages, 50):6.1f} ms  p90 {np.percentile(ages, 90):6.1f} ms | "
            f"work per frame p50 {np.percentile(infer, 50):5.1f} ms")
        self.frames, self.t_report = 0, now


def main() -> None:
    rclpy.init()
    node = DetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
