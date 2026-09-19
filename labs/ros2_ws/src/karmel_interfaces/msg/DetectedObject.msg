# One object found by a detector, localized in 3D when possible.
#
# header.frame_id is the frame the pose is expressed in (camera_optical_frame straight from a
# detector, base_link or map after a TF transform). header.stamp is the time of the source image —
# never "now" — so the pose can be transformed with the TF of the moment the image was taken.

std_msgs/Header header

string id                      # tracker id, stable across frames ("" if not tracked)
string label                   # class or open-vocabulary label, e.g. "red cup"
float32 confidence             # 0..1

bool has_pose                  # false when only the 2D box is known (e.g. no depth)
geometry_msgs/Pose pose        # object centre in header.frame_id; orientation identity if unknown
geometry_msgs/Vector3 size     # approximate 3D extent in metres (0 if unknown)
float64[36] pose_covariance    # row-major 6x6 (x, y, z, rot x, rot y, rot z); all 0 if unknown

BoundingBox2D bbox             # in the source image
string source_image_frame      # frame_id of the image the bbox refers to
