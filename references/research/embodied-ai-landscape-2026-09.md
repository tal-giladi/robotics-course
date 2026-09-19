# Embodied AI landscape — state of the field (verified 2026-09-16)

Scope: advanced part of the practical robotics course. Audience: experienced software/AI engineer.
Every URL below was fetched or HTTP-checked on 2026-09-16 (arXiv IDs checked against the arXiv API; DOIs against Crossref; package versions against PyPI). Anything not confirmed is marked UNVERIFIED.

Maturity labels:
- MATURE: stable, widely used in production or as a standard baseline; docs and releases are dependable.
- USABLE-BY-HOBBYIST: a student with one consumer GPU (or Colab / rented A10G) and a ~$100–$500 arm can get real results.
- RESEARCH: interesting and important, but expect big compute, closed weights, sparse docs, or fast breaking changes.

Compute shorthand: "consumer GPU" = RTX 3090/4090 (24 GB). "Orin Nano" = Jetson Orin Nano (Super) 8 GB.

---

## Part A — Current embodied AI landscape

### A.0 One-paragraph picture of September 2026

The stack has settled into three layers. (1) A high-level "reasoning brain" (a VLM/LLM such as Gemini Robotics-ER 2, or a general LLM with tool calling) plans, grounds objects in images, and calls skills. (2) A low-level visuomotor policy (a VLA such as pi0.5 / GR00T N1.7 / SmolVLA, or a small task-specific imitation policy such as ACT or Diffusion Policy) turns camera frames + language into action chunks at 10–50 Hz. (3) Classical control, ROS 2, Nav2 and MoveIt still do the safety-critical real-time parts. Around this: world models (Cosmos 3, V-JEPA 2, Genie 3) are used for synthetic data, policy evaluation and planning; GPU simulators (Isaac Lab 3 beta, MuJoCo Playground/MuJoCo Warp, Genesis, ManiSkill3) are used for RL and sim-to-real. Hugging Face LeRobot has become the de-facto hobbyist/academic hub: one dataset format, one CLI, and almost every open VLA wrapped as a policy type. The frontier closed models (Physical Intelligence pi0.7, Gemini Robotics 2 VLA) are ahead of what is open, but the open tier (pi0.5 via openpi/LeRobot, GR00T N1.7, SmolVLA, X-VLA) is genuinely usable.

What a student can realistically do on one 24 GB GPU: train ACT / Diffusion Policy from scratch on 50 demos in 1–4 h; fine-tune SmolVLA; LoRA-fine-tune pi0/pi0.5 (tight); run inference of most open VLAs. On a Jetson Orin Nano: run ACT comfortably; SmolVLA is borderline (~1 s per action chunk reported on Orin-class hardware, so use async inference / real-time chunking or offload to a desktop GPU).

---

### A.1 Hugging Face LeRobot

- What: end-to-end open robot-learning library: hardware drivers + teleoperation, LeRobotDataset (Parquet + MP4, streamable from the Hub), training/eval CLI, policy zoo, sim benchmarks, async inference, `lerobot-rollout` deployment with DAgger-style corrections, LeLab browser GUI.
- Current version: v0.6.1 (PyPI, 2026-08-03). v0.6.0 (blog 2026-07-07) was a large breaking cleanup: Python 3.12+, PyTorch 2.7–2.11, modular extras, rebuilt RL stack, world-model policies, reward-model API.
- Supported policies (README, 2026-09): ACT, Diffusion, VQ-BeT, Multitask DiT, HIL-SERL (SAC-based RL), TD-MPC, Pi0, Pi0-FAST, Pi0.5, GR00T N1.7 (`groot` policy type), SmolVLA, X-VLA, EO-1, MolmoAct2, WALL-OSS, EVO1, world-model policies VLA-JEPA, LingBot-VA, FastWAM; reward models SARM, TOPReward, Robometer.
  - pi0/pi0.5: yes, native PyTorch ports in LeRobot (checkpoints `lerobot/pi05_base`), in addition to Physical Intelligence's own openpi repo.
  - GR00T: yes, integrated (N1.7).
- Supported hardware: SO-100/SO-101 (flagship low-cost arm), LeKiwi (mobile base + SO arm), Koch v1.1, HopeJR, OMX, EarthRover, Reachy 2, OpenARM, Unitree G1, reBot B601; gamepad/keyboard/phone teleop; third-party plugins for xArm, UR5e, Franka and others.
- Compute (official Compute & Hardware Guide, indicative): ACT/VQ-BeT/TD-MPC ~2–6 GB VRAM (laptop RTX 3060 OK); Diffusion ~8–14 GB; SmolVLA ~10–16 GB; pi0/pi0-FAST/pi0.5/X-VLA ~24–40 GB (24 GB "tight at batch 1"); GR00T ~24–40 GB. ACT on 50 episodes: ~30–60 min on an RTX 4090, ~1–2 h on L4/A10G, ~6–14 h on Apple M-series Max. SmolVLA: ~3–6 h on L4. pi0/pi0.5: ~4–8 h on A100 40 GB. Free Colab notebooks exist; CPU-only training is explicitly discouraged. No built-in ONNX/TensorRT export yet (open GitHub issue #3146).
- Jetson: ACT runs fine on Orin-class devices. SmolVLA on Jetson AGX Orin measured ~1.0–1.3 s to first action (third-party study, not official), so on an Orin Nano treat VLA inference as "offload to a GPU server via LeRobot async inference". An Aug-2026 arXiv paper (2608.03938) reports quantized ACT bimanual control within 8 GB on an entry-level Jetson.
- License: Apache-2.0.
- Maturity: USABLE-BY-HOBBYIST (the reference choice for this course). API still breaks between minor versions — pin the version.
- URLs: https://github.com/huggingface/lerobot · docs https://huggingface.co/docs/lerobot/index · SO-101 https://huggingface.co/docs/lerobot/so101 · LeKiwi https://huggingface.co/docs/lerobot/lekiwi · Koch https://huggingface.co/docs/lerobot/koch · compute guide https://huggingface.co/docs/lerobot/hardware_guide · paper (ICLR 2026) https://arxiv.org/abs/2602.22818 · tutorial https://arxiv.org/abs/2510.12403

### A.2 Physical Intelligence — pi0 / pi0-FAST / pi0.5 (openpi) and newer closed models

- What: pi0 (Oct 2024) = flow-matching VLA (PaliGemma VLM + action expert). pi0-FAST (Jan/Feb 2025) = autoregressive VLA with FAST DCT-based action tokenizer. pi0.5 (Apr 2025, open-sourced Sep 2025) = co-trained on web data + multi-robot data for open-world generalization, trained with "knowledge insulation".
- openpi status (2026-09): open weights only for pi0, pi0-FAST, pi0.5 (base + DROID/ALOHA/LIBERO fine-tunes). JAX primary; PyTorch port for pi0 and pi0.5 (no pi0-FAST in PyTorch, no PyTorch LoRA yet).
- Compute (openpi README): inference >8 GB (RTX 4090); LoRA fine-tune >22.5 GB (RTX 4090); full fine-tune >70 GB (A100/H100 80 GB).
- Newer, closed (no weights): pi*0.6 (2025-11-17; RL from experience + corrections via "Recap", arXiv 2511.14759); pi0.7 (2026-04-16; steerable generalist with compositional generalization, prompts carry quality/strategy metadata and visual subgoals, arXiv 2604.15483). Other PI posts: real-time chunking (2506.07339), human-to-robot transfer (Dec 2025), memory for VLAs (Mar 2026), efficient online RL (Mar 2026).
- License: openpi Apache-2.0; Gemma-derived weights also carry Gemma terms (LICENSE_GEMMA.txt).
- Jetson Orin Nano: no (3B-class model; server inference).
- Maturity: pi0/pi0.5 via openpi or LeRobot = USABLE-BY-HOBBYIST only with a 24 GB GPU for LoRA (else rent); pi*0.6 / pi0.7 = RESEARCH (read the papers, cannot run).
- URLs: https://github.com/Physical-Intelligence/openpi · https://www.pi.website/blog/pi0 · https://www.pi.website/blog/pi05 · https://www.pi.website/blog/pistar06 · https://www.pi.website/blog/pi07 · LeRobot pi0.5 docs https://huggingface.co/docs/lerobot/pi05

### A.3 NVIDIA Isaac GR00T N1.x

- What: open "generalist humanoid" VLA foundation model family. Dual-system design: VLM (System 2) + diffusion-transformer action head (System 1).
- Versions: N1 (Mar 2025, arXiv 2503.14734) → N1.5 (2025) → N1.6 (announced around CoRL 2025 / CES 2026; Cosmos-Reason-2B-variant VLM) → N1.7 (early access / HF blog 2026-04-17; current GA in the repo). N1.7: 3B params, VLM backbone Cosmos-Reason2-2B (Qwen3-VL based), state/action dims expanded to 132, action horizon 40, pre-trained on ~20.8k hours of human egocentric video ("EgoScale", claimed first dexterity scaling law), commercially licensed.
- Compute (repo README): inference 16 GB+ GPU (RTX 4090, L40, H100, Jetson AGX Thor/Orin); fine-tuning 40 GB+ recommended (single GPU works slowly on A6000-class). Jetson Orin Nano (8 GB): not listed → effectively no.
- Integration: available as `groot` policy in LeRobot; fine-tunes on LeRobotDataset format; `NEW_EMBODIMENT` tag for custom robots.
- License: code Apache-2.0; weights NVIDIA Open Model License (commercial use permitted for N1.7).
- Maturity: USABLE-BY-HOBBYIST for inference and small fine-tunes on rented 40 GB+ GPUs; RESEARCH-grade for humanoid work.
- URLs: https://github.com/NVIDIA/Isaac-GR00T · https://huggingface.co/blog/nvidia/gr00t-n1-7 · https://research.nvidia.com/labs/gear/gr00t-n1_6/ · LeRobot docs https://huggingface.co/docs/lerobot/groot

### A.4 Google DeepMind — Gemini Robotics family

- Gemini Robotics (VLA) — v1 Mar 2025 (arXiv 2503.20020), 1.5 Sep/Oct 2025 (arXiv 2510.03342: "thinking" VLA + motion transfer across embodiments), now **Gemini Robotics 2** listed on the model page. Access: trusted testers and research partners only (Apptronik, Agile Robots, Boston Dynamics etc.). A Gemini Robotics 2 technical report: UNVERIFIED (not found on arXiv).
- Gemini Robotics On-Device — v1 June 2025; **On-Device 2** model card dated 2026-07-30 (built on Gemini Robotics 1.5 tech + on-device Gemma). Access: trusted testers via the Gemini Robotics SDK (fine-tune with 50–100 demos, MuJoCo sim eval). Not public.
- Gemini Robotics-ER (embodied reasoning VLM, the "brain") — ER 1.5 (Sep 2025, first public API robotics model) → ER 1.6 preview (Dec 2025; shut down 2026-08-31) → **Gemini Robotics-ER 2** (2026-07-30). Public via Gemini API + Google AI Studio (preview). Model IDs: `gemini-robotics-er-2-preview`, `gemini-robotics-er-2-streaming-preview` (Live API streaming). Capabilities: pointing, 2D boxes, trajectories, object tracking, task decomposition/orchestration, function calling, code execution ("agentic vision"), structured output, video progress classification and moment finding, multi-robot coordination. 131k input / 65k output tokens. Designed to hand motor execution off to any lower-level VLA or skill API. Pricing: see Gemini API pricing page (free tier specifics UNVERIFIED).
- Compute: cloud API (ER); no local weights for anything in this family.
- License: proprietary (Gemini API terms).
- Maturity: ER 2 = USABLE-BY-HOBBYIST (it is the easiest way to give an SO-101 or LeKiwi a high-level planner/perception brain with just an API key); VLA / On-Device = RESEARCH/closed.
- Related: "Evaluating Gemini Robotics Policies in a Veo World Simulator" (arXiv 2512.10675) — world-model-based policy eval; ASIMOV semantic safety benchmark (arXiv 2503.08663).
- URLs: https://deepmind.google/models/gemini-robotics/ · https://blog.google/innovation-and-ai/models-and-research/google-deepmind/gemini-robotics-er-2/ · https://ai.google.dev/gemini-api/docs/robotics-overview · https://ai.google.dev/gemini-api/docs/models/gemini-robotics-er-2-preview · SDK https://github.com/google-deepmind/gemini-robotics-sdk · On-Device 2 card https://deepmind.google/models/model-cards/gemini-robotics-on-device-2/

### A.5 OpenVLA / OpenVLA-OFT

- What: OpenVLA (Jun 2024) = 7B VLA (Prismatic: DINOv2 + SigLIP vision, Llama-2 7B), trained on ~970k Open X-Embodiment episodes, actions as discretized tokens. OpenVLA-OFT (Feb 2025) = "Optimized Fine-Tuning" recipe: parallel decoding, action chunking, continuous actions, L1 regression → LIBERO 76.5% → 97.1%, ~26× faster action generation.
- Status: historically the reference open VLA; repo last significant updates ~2025. Superseded in practice by pi0.5 / GR00T / SmolVLA / X-VLA but OFT's lessons (chunking + continuous regression beat token decoding) are now standard.
- Compute: inference ~16 GB in bf16 (4-bit quantized fits ~7 GB — quantization is documented in the paper; exact figure UNVERIFIED for OFT); LoRA fine-tune documented at ~72 GB for the recommended batch (can reduce batch + grad accumulation to fit a 24 GB GPU slowly). Not for Orin Nano.
- License: code MIT; weights under Llama 2 Community License.
- Maturity: RESEARCH (good to read, less good to build on in 2026).
- URLs: https://openvla.github.io/ · https://github.com/openvla/openvla · https://arxiv.org/abs/2406.09246 · https://github.com/moojink/openvla-oft · https://arxiv.org/abs/2502.19645

### A.6 Octo

- What: open generalist transformer diffusion policy (May 2024) trained on 800k OXE trajectories; Octo-Small 27M, Octo-Base 93M; flexible observation/action heads for fine-tuning to new robots.
- Status: JAX; not actively developed in 2026 (UNVERIFIED exact last-commit date); still a clean example of cross-embodiment design.
- Compute: small — fine-tunes on a single consumer GPU; inference feasible on Jetson (small model; JAX-on-Jetson setup effort UNVERIFIED).
- License: MIT.
- Maturity: RESEARCH (historical baseline).
- URLs: https://octo-models.github.io/ · https://github.com/octo-models/octo · https://arxiv.org/abs/2405.12213

### A.7 RT-1 / RT-2 / RT-X and Open X-Embodiment (OXE)

- RT-1 (Dec 2022): EfficientNet + TokenLearner + transformer, 130k episodes, 700+ tasks — showed scale works for real-robot BC. Weights/code partially released (TF).
- RT-2 (Jul 2023): co-fine-tuned PaLI-X / PaLM-E VLMs emitting actions as text tokens — coined "VLA". Closed.
- Open X-Embodiment + RT-X (Oct 2023): pooled 60 datasets from 21 institutions, 22 embodiments, 1M+ trajectories in RLDS format; RT-1-X / RT-2-X showed positive transfer across robots. The dataset remains the standard pre-training mix (subsets used by Octo, OpenVLA, pi0).
- Compute: dataset is TB-scale; students should stream a subset (LeRobot hosts converted subsets on the Hub).
- License: OXE constituent datasets carry their own licenses (mostly permissive, varies per dataset — check each); RT-1 code Apache-2.0.
- Maturity: dataset = MATURE (as a resource); models = RESEARCH/historical.
- URLs: https://robotics-transformer-x.github.io/ · https://github.com/google-deepmind/open_x_embodiment · https://arxiv.org/abs/2212.06817 · https://arxiv.org/abs/2307.15818 · https://arxiv.org/abs/2310.08864

### A.8 Diffusion Policy

- What: (Chi et al., RSS 2023) policy = conditional denoising diffusion over action sequences; receding-horizon action chunks; handles multimodal demonstrations. Now the default "strong baseline" and the action head in many VLAs (GR00T, Octo; pi0 uses the closely related flow matching).
- Compute: trains from scratch on a single consumer GPU (~2–4 h for 50 demos on an RTX 4090 per LeRobot guide); inference needs several denoising steps (DDIM ~10–16) — OK on desktop GPU, marginal on Orin Nano.
- License: MIT (original repo); Apache-2.0 in LeRobot.
- Maturity: MATURE / USABLE-BY-HOBBYIST.
- URLs: https://diffusion-policy.cs.columbia.edu/ · https://github.com/real-stanford/diffusion_policy · https://arxiv.org/abs/2303.04137

### A.9 ACT / ALOHA / Mobile ALOHA

- What: ALOHA = ~$20k-class open-source bimanual teleop rig (ViperX/WidowX); ACT (Action Chunking with Transformers, CVAE + transformer predicting k-step action chunks with temporal ensembling), Apr 2023. Mobile ALOHA (Jan 2024) = whole-body mobile bimanual teleop + co-training with static ALOHA data.
- Why it matters for the course: ACT is LeRobot's recommended first policy and the thing students will actually train on an SO-101.
- Compute: ~2–6 GB VRAM training; 30–60 min on a 4090 for 50 demos; real-time inference on Jetson Orin Nano is feasible (small ResNet-18 + transformer; ~80M params).
- License: MIT (act repo); ALOHA hardware open-source.
- Maturity: MATURE / USABLE-BY-HOBBYIST.
- URLs: https://tonyzhaozh.github.io/aloha/ · https://github.com/tonyzhaozh/act · https://arxiv.org/abs/2304.13705 · https://mobile-aloha.github.io/ · https://arxiv.org/abs/2401.02117

### A.10 SmolVLA

- What: Hugging Face's ~450M-param VLA (Jun 2025): SmolVLM2 backbone with layer skipping + flow-matching action expert; pre-trained only on community LeRobot datasets; async inference.
- Compute: trains/fine-tunes on a single GPU (~10–16 GB); inference on consumer GPU or even CPU; Jetson Orin-class ~1 s per chunk without optimization (third-party measurement) → use async inference.
- License: Apache-2.0.
- Maturity: USABLE-BY-HOBBYIST (the natural "first VLA" for the course hardware).
- URLs: https://arxiv.org/abs/2506.01844 · https://huggingface.co/lerobot/smolvla_base · https://huggingface.co/docs/lerobot/smolvla

### A.11 DROID dataset

- What: (Mar 2024) 76k teleoperated trajectories / ~350 h / 564 scenes / 84 tasks on a standardized Franka + Robotiq + ZED setup, collected by 50 data collectors across North America, Asia and Europe over 12 months — the main "in the wild" diverse manipulation dataset; pi0/pi0.5 and GR00T ship DROID fine-tunes.
- License: CC-BY 4.0 (per project page; UNVERIFIED re-check recommended before redistribution).
- Maturity: MATURE (resource).
- URLs: https://droid-dataset.github.io/ · https://github.com/droid-dataset/droid · https://arxiv.org/abs/2403.12945

### A.12 UMI — Universal Manipulation Interface

- What: (Feb 2024) hand-held 3D-printed gripper with GoPro + fisheye/side mirrors; collect demos without a robot, SLAM-recover 6DoF trajectories, train Diffusion Policy, deploy on different arms. Spawned many follow-ups (hand-held data collection is now mainstream; GR00T N1.7 and pi use human/egocentric data too).
- Compute: the training side = Diffusion Policy (single GPU); the SLAM pipeline runs on a desktop.
- License: MIT (repo).
- Maturity: USABLE-BY-HOBBYIST for motivated students (hardware build + calibration effort is real); RESEARCH for the method.
- URLs: https://umi-gripper.github.io/ · https://github.com/real-stanford/universal_manipulation_interface · https://arxiv.org/abs/2402.10329

### A.13 Other open VLAs worth knowing (2025–2026)

All wrapped in LeRobot as of v0.6.x: X-VLA (soft-prompted cross-embodiment transformer, arXiv 2510.10274), MolmoAct / MolmoAct2 (Ai2, "action reasoning" in space, arXiv 2508.07917), EO-1, WALL-OSS, EVO1, VLA-JEPA, LingBot-VA, FastWAM. Maturity: RESEARCH; compute: 24–40 GB class for fine-tuning.

### A.14 World models for robotics

| Model | What | Access / license | Compute | Maturity |
|---|---|---|---|---|
| NVIDIA Cosmos (1 → Predict/Transfer/Reason 2.5 → **Cosmos 3**) | World foundation models: video prediction (Predict), sim-to-real style transfer with control inputs (Transfer), physical-reasoning VLM (Reason). Cosmos 3 (launched 2026-05-31/06-01, GTC Taipei) unifies reasoning, world generation and action prediction in one mixture-of-transformers "omnimodel"; Super and Nano variants, Edge (4B) added 2026-07-20. Cosmos-Reason2-2B is GR00T N1.7's VLM backbone. | Weights on Hugging Face; code Apache-2.0 (repos under github.com/nvidia-cosmos, moving to github.com/NVIDIA/Cosmos); weights under NVIDIA Open Model License (confirmed for earlier Cosmos releases; Cosmos 3 exact license UNVERIFIED) | Predict/Transfer: data-center GPUs (multi-H100 for big variants); Reason 2B / Cosmos 3 Nano/Edge: single GPU | RESEARCH (Reason-2B: USABLE) |
| Meta V-JEPA 2 / V-JEPA 2-AC (Jun 2025) and V-JEPA 2.1 (Mar 2026) | Self-supervised video encoder pre-trained on 1M+ h of video; action-conditioned latent world model post-trained on <62 h of robot video, used for zero-shot MPC-style planning on Franka arms. | Code + weights via GitHub; repo license MIT (weights terms: check model card — UNVERIFIED) | Inference on a single GPU; pre-training data-center | RESEARCH |
| Google DeepMind Genie 3 (Aug 2025) | Real-time (24 fps, 720p) interactive text-to-world model with minutes-long consistency; pitched for agent/embodied training. Project Genie rolled out to AI Ultra subscribers in the US 2026-01-29. | Closed; research preview + consumer Labs product | Cloud only | RESEARCH |
| Veo-based policy evaluation (DeepMind, arXiv 2512.10675) | Use a video world model to evaluate Gemini Robotics policies | Paper only | — | RESEARCH |
| LeRobot world-model policies (VLA-JEPA, LingBot-VA, FastWAM) | "Imagine then act" policies packaged in LeRobot v0.6 | Open (Apache-2.0 wrapper; underlying licenses vary) | 24 GB+ | RESEARCH |

URLs: https://www.nvidia.com/en-us/ai/cosmos/ · https://nvidianews.nvidia.com/news/nvidia-launches-cosmos-3-the-open-frontier-foundation-model-for-physical-ai · https://github.com/nvidia-cosmos · https://github.com/NVIDIA/Cosmos · https://arxiv.org/abs/2501.03575 (Cosmos WFM platform) · https://arxiv.org/abs/2503.15558 (Cosmos-Reason1) · https://arxiv.org/abs/2506.09985 (V-JEPA 2) · https://arxiv.org/abs/2603.14482 (V-JEPA 2.1) · https://github.com/facebookresearch/vjepa2 · https://deepmind.google/blog/genie-3-a-new-frontier-for-world-models/ · https://deepmind.google/models/genie/

### A.15 Simulation frameworks

| Framework | Current state (2026-09) | License | Compute | Maturity | URLs |
|---|---|---|---|---|---|
| **NVIDIA Isaac Sim / Isaac Lab** | Isaac Lab stable v2.3.2 (Feb 2026, Isaac Sim 5.x); **Isaac Lab 3.0 beta2** (Jun/Jul 2026) on Isaac Sim 6.0: multi-backend physics (incl. Newton/MuJoCo-Warp), pluggable renderers, kit-less install; Isaac Sim 6.1 deps landing (Sep 2026). Paper arXiv 2511.04831 (successor to Orbit / Isaac Gym). | Isaac Lab BSD-3-Clause (GitHub); Isaac Sim free to use under NVIDIA license | RTX GPU required (RTX 3070+ practical; 4090 comfortable), Linux/Windows; no Jetson | MATURE (2.3) / beta (3.0) | https://github.com/isaac-sim/IsaacLab · https://isaac-sim.github.io/IsaacLab/ · https://arxiv.org/abs/2511.04831 |
| **MuJoCo / MJX / MuJoCo Warp / MuJoCo Playground** | MuJoCo 3.13.0 (2026-09-09). MJX (JAX) and MuJoCo Warp (NVIDIA Warp, co-developed by DeepMind + NVIDIA, also inside Newton) ship with the same version. Playground 0.2.0 (2026-03-16): Warp is now the default backend for all envs, vision-based training; locomotion, dexterous hand and manipulation envs with sim-to-real results. | Apache-2.0 | CPU MuJoCo runs anywhere (even Orin Nano); Playground trains locomotion policies in minutes on a single consumer GPU; Colab notebooks | MATURE (MuJoCo) / USABLE-BY-HOBBYIST (Playground) | https://github.com/google-deepmind/mujoco · https://mujoco.readthedocs.io/en/latest/mjwarp/ · https://playground.mujoco.org/ · https://github.com/google-deepmind/mujoco_playground · https://arxiv.org/abs/2502.08844 |
| **Genesis** | `genesis-world` 1.4.1 on PyPI (2026-09-12); 1.0 in May 2026; now backed by Genesis AI company. Multi-physics (rigid, FEM, MPM, particles, IPC), own compiler targeting CUDA/ROCm/Metal/Vulkan/CPU, sensors incl. tactile/LiDAR. No arXiv technical report found (UNVERIFIED — early speed claims were disputed by the community in Dec 2024). | Apache-2.0 | GPU (NVIDIA/AMD/Apple) or CPU | USABLE-BY-HOBBYIST (young) | https://github.com/Genesis-Embodied-AI/Genesis · https://genesis-world.readthedocs.io/ |
| **ManiSkill3** | `mani-skill` 3.0.1 (Apr 2026); GPU-parallel SAPIEN-based sim + rendering; RSS 2025 paper; sim2real examples incl. SO-100-class arms (UNVERIFIED exact robot list). | Code Apache-2.0; assets CC BY-NC 4.0 | GPU sim Linux + NVIDIA only; CPU elsewhere | USABLE-BY-HOBBYIST | https://github.com/haosulab/ManiSkill · https://arxiv.org/abs/2410.00425 |
| **robosuite / robomimic** | robosuite 1.5.2 (Dec 2025, MuJoCo-based, more robots incl. humanoids); robomimic 0.3.0 (2023, low activity) — still the canonical offline-IL study and dataset suite. | MIT | CPU / single GPU | MATURE (robosuite) / RESEARCH-maintenance (robomimic) | https://github.com/ARISE-Initiative/robosuite · https://arxiv.org/abs/2009.12293 · https://github.com/ARISE-Initiative/robomimic · https://arxiv.org/abs/2108.03298 |

Benchmarks now wrapped in LeRobot: LIBERO, LIBERO-plus, Meta-World, RoboTwin 2.0, RoboCasa365, RoboCerebra, RoboMME, VLABench, IsaacLab-Arena.

### A.16 Summary table

| Item | Label | Single consumer GPU / Colab fine-tune? | Jetson Orin Nano inference? | License |
|---|---|---|---|---|
| LeRobot | USABLE-BY-HOBBYIST | yes (ACT/Diffusion/SmolVLA) | ACT yes; VLAs offload | Apache-2.0 |
| ACT | MATURE | yes (Colab T4 OK) | yes | MIT / Apache-2.0 |
| Diffusion Policy | MATURE | yes | marginal | MIT |
| SmolVLA | USABLE-BY-HOBBYIST | yes | marginal (~1 s/chunk) | Apache-2.0 |
| pi0 / pi0-FAST / pi0.5 (openpi) | USABLE (24 GB+) | LoRA on 24 GB; full FT 80 GB | no | Apache-2.0 + Gemma terms |
| pi*0.6 / pi0.7 | RESEARCH (closed) | no | no | proprietary |
| GR00T N1.7 | USABLE (40 GB rented) / RESEARCH | FT needs 40 GB+ | no (AGX Orin/Thor yes) | Apache-2.0 code, NVIDIA Open Model License weights |
| Gemini Robotics-ER 2 | USABLE-BY-HOBBYIST (API) | n/a | API call from Jetson fine | proprietary API |
| Gemini Robotics 2 VLA / On-Device 2 | RESEARCH (trusted testers) | no | no | proprietary |
| OpenVLA / OFT | RESEARCH | LoRA slow on 24 GB | no | MIT code, Llama-2 weights |
| Octo | RESEARCH (historical) | yes | probably (UNVERIFIED) | MIT |
| RT-1/RT-2/RT-X | RESEARCH (historical) | — | — | mixed |
| OXE / DROID datasets | MATURE resources | stream subsets | — | per-dataset / CC-BY |
| UMI | USABLE (hardware effort) | yes | — | MIT |
| Cosmos 3 / Reason 2 | RESEARCH (Reason-2B usable) | Reason-2B yes | Edge variant targets edge (UNVERIFIED on Orin Nano) | Apache-2.0 code / NVIDIA OML weights |
| V-JEPA 2 | RESEARCH | inference yes | no | MIT (repo) |
| Genie 3 | RESEARCH (closed) | no | no | proprietary |
| Isaac Lab | MATURE (2.3) | RL on 1 RTX GPU yes | no | BSD-3-Clause |
| MuJoCo Playground | USABLE-BY-HOBBYIST | yes (Colab) | CPU MuJoCo yes | Apache-2.0 |
| Genesis | USABLE (young) | yes | CPU possible | Apache-2.0 |
| ManiSkill3 | USABLE-BY-HOBBYIST | yes (Linux) | no | Apache-2.0 / CC BY-NC assets |
| robosuite / robomimic | MATURE / maintenance | yes | CPU | MIT |

---

## Part B — LLMs + robots

### B.1 Foundational papers (2022–2023), all arXiv-verified

| Paper | Year | Core idea | What survives in 2026 practice | URL |
|---|---|---|---|---|
| SayCan — "Do As I Can, Not As I Say: Grounding Language in Robotic Affordances" (Ahn et al., Google) | 2022 | LLM scores which pre-trained skill is *useful*; learned value functions score which is *feasible*; pick argmax of product. | "LLM picks from a fixed skill library" = today's tool calling; feasibility checks now done by preconditions/validators or a VLM. | https://arxiv.org/abs/2204.01691 · https://say-can.github.io/ |
| Inner Monologue (Huang et al., Google) | 2022 | Closed-loop: feed success detection, scene descriptions and human feedback back into the LLM prompt as text. | The agent loop with observations/tool results appended to context. | https://arxiv.org/abs/2207.05608 · https://innermonologue.github.io/ |
| Code as Policies (Liang et al., Google) | 2022 | LLM writes Python that calls perception + control APIs, with hierarchical function generation. | Still the dominant pattern for LLM → robot (Gemini Robotics-ER "agentic code execution", ChatGPT-for-Robotics). | https://arxiv.org/abs/2209.07753 · https://code-as-policies.github.io/ |
| ProgPrompt (Singh et al., NVIDIA/USC) | 2022 | Prompt as a Python program: import available actions, list objects, few-shot example programs, assertions for precondition checking. | Typed tool schemas + assertions = function-calling specs. | https://arxiv.org/abs/2209.11302 · https://progprompt.github.io/ |
| VoxPoser (Huang et al., Stanford) | 2023 | LLM writes code that queries a VLM to build 3D affordance/avoidance value maps; motion planner optimizes trajectory over them; zero-shot. | Using LLM/VLM for spatial *constraints* and classical planners for motion; ancestor of ER-model pointing/trajectory outputs. | https://arxiv.org/abs/2307.05973 · https://voxposer.github.io/ |
| ChatGPT for Robotics: Design Principles and Model Abilities (Vemprala et al., Microsoft) | 2023 | Practical recipe: define a high-level function library, describe it in the prompt, let the model write code, human in the loop in simulation, iterate via dialog. | Almost exactly today's MCP/tool-calling workflow. (Microsoft article page returns HTTP 403 to bots; arXiv is canonical.) | https://arxiv.org/abs/2306.17582 |
| SayPlan (Rana et al.) | 2023 | LLM planning over large 3D scene graphs with iterative replanning via a verifier/simulator. | Scene graphs / semantic maps as the LLM's world state. | https://arxiv.org/abs/2307.06135 |

### B.2 LLMs + behavior trees

- Colledanchise & Ögren, "Behavior Trees in Robotics and AI: An Introduction" (book, arXiv 1709.00084) and Iovino et al. survey (arXiv 2005.05842) — background. BTs are the execution layer in Nav2 (BehaviorTree.CPP).
- BTGenBot (Izzo et al., 2024) — lightweight fine-tuned LLMs generate BehaviorTree.CPP XML for ROS 2 robots: https://arxiv.org/abs/2403.12761
- LLM-as-BT-Planner (Ao et al., 2024) — LLM generates BTs for assembly task planning: https://arxiv.org/abs/2409.10444
- Practice: LLM generates or selects a BT (a verifiable, inspectable artifact) instead of emitting raw motor commands; BT is validated against a schema and executed by BehaviorTree.CPP. This gives a reviewable plan and deterministic execution. https://github.com/BehaviorTree/BehaviorTree.CPP

### B.3 ROS agents and MCP servers (verified 2026-09)

- **ROSA — NASA JPL Robot Operating System Agent.** LangChain-based agent that inspects and operates ROS 1 (Noetic) and ROS 2 (Humble/Iron/Jazzy) systems via natural language, with a tool set for topics/services/params/logs and custom tools. `pip install jpl-rosa` — v1.0.10 (PyPI, 2026-03-17). Apache-2.0. Paper: "Enabling Novel Mission Operations and Interactions with ROSA: The Robot Operating System Agent" (Royce et al., 2024) https://arxiv.org/abs/2410.06472 · https://github.com/nasa-jpl/rosa. Maturity: USABLE-BY-HOBBYIST (diagnostics/introspection more than control).
- **ros-mcp-server (robotmcp).** MCP server connecting any MCP client (Claude Code/Desktop, Codex CLI, Gemini CLI, ChatGPT, Cursor) to ROS 1/ROS 2 via **rosbridge** (no robot code changes): list/introspect topics, services, actions and types; publish/subscribe; call services and actions; set params; read sensors. Latest release v3.1.0 (June 17, 2026); v3.0.0 major refactor (Jan 29, 2026). Apache-2.0. No safety layer documented in the README — anything the rosbridge exposes, the LLM can command. https://github.com/robotmcp/ros-mcp-server. Maturity: USABLE-BY-HOBBYIST.
- Other ROS 2 MCP servers (existence verified; maturity not assessed): wise-vision/ros2_mcp (native ROS 2, stdio) https://github.com/wise-vision/ros2_mcp; kakimochi/ros2-mcp-server; LCAS/ros2_mcp; "Amazing ROS 2 MCP" (native rclpy, announced on ROS Discourse).
- **RAI (Robotec.ai).** Vendor-agnostic agentic framework for ROS 2 (Humble/Jazzy): multi-agent, ASR/TTS, perception tools, benchmarks. Apache-2.0. https://github.com/RobotecAI/rai
- **ROS-LLM** (Mower et al., 2024) — ROS framework with task feedback and structured reasoning: https://arxiv.org/abs/2406.19741
- **ReMEmbR** (NVIDIA, 2024) — long-horizon spatio-temporal memory (VLM captions in a vector DB) + LLM agent for robot navigation Q&A: https://arxiv.org/abs/2409.13682
- **Gemini Robotics-ER 2** (see A.4) — first-party "robot brain" API with native function calling, code execution and orchestration of a lower-level VLA or skill API.

### B.4 Safety of LLM-controlled robots

| Paper | Finding | URL |
|---|---|---|
| Jailbreaking LLM-Controlled Robots — RoboPAIR (Robey, Ravichandran, Kumar, Hassani, Pappas; 2024) | Adapted the PAIR jailbreak to robots; near-100% attack success on NVIDIA Dolphins self-driving LLM, Clearpath Jackal + GPT-4o planner, and Unitree Go2's built-in GPT-3.5 — in white-, gray- and black-box settings. Physical harm is now a jailbreak outcome. | https://arxiv.org/abs/2410.13691 · https://robopair.org/ |
| Safety Guardrails for LLM-Enabled Robots — RoboGuard (Ravichandran, Robey et al.; 2025) | Two-stage guardrail: a root-of-trust LLM grounds safety rules in the robot's world model into temporal-logic specs; a control-synthesis step resolves conflicts with the proposed plan. Cuts unsafe plan execution drastically under jailbreaks. | https://arxiv.org/abs/2503.07885 |
| BadRobot: Jailbreaking Embodied LLM Agents in the Physical World (Zhang et al.; 2024) | Taxonomy of embodied jailbreaks (contextual jailbreak, safety misalignment between words and actions, conceptual deception). | https://arxiv.org/abs/2407.20242 |
| LLM-Driven Robots Risk Enacting Discrimination, Violence, and Unlawful Actions (Hundt et al.; 2024) | Popular LLMs, placed in HRI scenarios, produce discriminatory and harmful action choices; argues current LLMs are unsafe as general robot controllers. | https://arxiv.org/abs/2406.08824 |
| Generating Robot Constitutions & Benchmarks for Semantic Safety — ASIMOV (Sermanet et al., Google DeepMind; 2025) | Data-driven "robot constitutions" and the ASIMOV benchmark for semantic (common-sense) safety of embodied reasoning models; used for Gemini Robotics. | https://arxiv.org/abs/2503.08663 |

### B.5 Current practice for tool-calling robot agents (synthesis, Sep 2026)

This is a synthesis of the sources above plus the official docs; treat as course guidance rather than a single citable standard.

1. **Layering.** LLM/VLM plans at 0.1–1 Hz; skills (Nav2 actions, MoveIt/cuRobo motions, a VLA policy for "pick X") run at 10–1000 Hz underneath. The LLM never streams joint commands.
2. **Skills as typed tools.** Expose a small, curated skill API (`navigate_to(pose|place_name)`, `pick(object_id)`, `look_at`, `get_scene_graph`) with JSON-schema args — not raw `publish(topic, msg)`. Generic ROS-introspection MCP servers (ros-mcp-server, ROSA) are excellent for debugging/dev, dangerous as production control surfaces.
3. **Grounding.** Use a VLM (Gemini Robotics-ER 2, open VLMs, Grounding DINO / OWLv2 / SAM 3) to turn language references into object IDs, points, boxes, masks; keep a scene graph or semantic map as world state (SayPlan, ReMEmbR).
4. **Code vs. calls.** Code-as-Policies-style code generation is powerful for spatial logic; run it in a sandbox with only the skill API importable. Otherwise use plain function calling or LLM-generated behavior trees validated against a schema (BTGenBot).
5. **Closed loop.** Every tool returns success/failure + observation; the agent re-plans (Inner Monologue). Add VLM success detectors / progress classifiers (ER 2 "progress classification", LeRobot reward models).
6. **Guardrails outside the LLM.** Hard limits in the controller (velocity, workspace, force), geofences in Nav2 costmaps/keepout zones, action validators and allow-lists, human confirmation for irreversible or high-energy actions, emergency stop independent of software. Assume the prompt can be jailbroken (RoboPAIR) — safety must hold even if the LLM is adversarial (RoboGuard).
7. **Audit + simulate first.** Log every tool call; dry-run plans in simulation (Isaac Sim/MuJoCo/Gazebo) before hardware.

---

## Sources checked but left out or flagged

- Microsoft "ChatGPT for Robotics" article page (https://www.microsoft.com/en-us/research/articles/chatgpt-for-robotics/) — HTTP 403 to automated check; use arXiv 2306.17582.
- Gemini Robotics 2 (VLA) technical report — UNVERIFIED (none found).
- Genesis technical paper — UNVERIFIED (none found on arXiv).
- Cosmos 3 weights license — UNVERIFIED (earlier Cosmos weights: NVIDIA Open Model License).
- Exact free-tier pricing for Gemini Robotics-ER 2 — UNVERIFIED.
- Some date strings returned by GitHub page summaries showed wrong years (e.g. "2024" for 2026 releases); versions/dates above were cross-checked against PyPI upload times or official blog dates.
