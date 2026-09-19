# Curated paper progression — classical robotics to embodied AI (raw, verified 2026-09-16)

Raw material for the course reading list. Ordered within each track from foundations to current.

Verification: arXiv IDs were checked against the arXiv API (title + first authors + date match); DOIs were checked against Crossref (title + authors + year match); project/repo/doc URLs were HTTP-checked (200) on 2026-09-16. Section guidance refers to section *names* rather than numbers where numbering varies between versions. Anything not confirmed is marked UNVERIFIED.

Entry format:
- Title — authors — year — URL
- Why: why it matters
- Prereqs: course topics needed first
- Understand: what the student should walk away with
- Read: ALL / PARTIAL (+ which sections) / SKIM

Core entries are numbered (C1.1 …). "Further" lists at the end of each track are optional extras, also verified.

Suggested global order for a self-study engineer: T1 → T2 (C2.1–C2.5) → T3 (C3.1–C3.8) → T5 (C5.1–C5.6) → T4 → T6 (C6.1–C6.5) → T7 → T8 → back to the modern ends of T2/T5/T6.

---

## Track 1 — Probabilistic state estimation

**C1.1 A New Approach to Linear Filtering and Prediction Problems** — R. E. Kalman — 1960 — https://doi.org/10.1115/1.3662552
- Why: The origin of the Kalman filter; every IMU/odometry fusion node (e.g. robot_localization) descends from it.
- Prereqs: linear algebra, probability (Gaussians, conditional expectation), state-space models.
- Understand: state-space model; predict/update as optimal linear estimation; covariance propagation; duality with control.
- Read: SKIM. The original notation is Wiener-filter-era and hard; read the introduction and problem statement for history, then learn the algorithm from C1.2.

**C1.2 An Introduction to the Kalman Filter (TR 95-041)** — G. Welch, G. Bishop — 1995 (revised 2006) — https://www.cs.utexas.edu/~pstone/Courses/393Rfall15/readings/Welch+Bishop-TR-95.pdf (course mirror; the original UNC URL now 404s)
- Why: The standard short tutorial; covers both the discrete KF and the Extended KF in ~16 pages.
- Prereqs: C1.1 context, matrix calculus basics, Jacobians.
- Understand: the five KF equations; process vs. measurement noise (Q, R) and tuning intuition; EKF linearization via Jacobians and why it can diverge.
- Read: ALL (short). Do the worked "estimating a random constant" example in code.

**C1.3 Unscented Filtering and Nonlinear Estimation** — S. J. Julier, J. K. Uhlmann — 2004 (Proc. IEEE) — https://doi.org/10.1109/JPROC.2003.823141
- Why: Explains the EKF's failure modes and the sigma-point (UKF) alternative; robot_localization ships both EKF and UKF.
- Prereqs: C1.2 (EKF), multivariate Gaussians.
- Understand: why linearizing a nonlinear model biases mean/covariance; unscented transform; when UKF is worth its cost.
- Read: PARTIAL — introduction, the EKF problems discussion, the unscented transform, and the UKF algorithm; skip the extended applications.

**C1.4 Monte Carlo Localization for Mobile Robots** — F. Dellaert, D. Fox, W. Burgard, S. Thrun — 1999 (ICRA) — https://doi.org/10.1109/ROBOT.1999.772544
- Why: Introduced particle-filter localization (MCL) — the algorithm inside ROS/Nav2 AMCL.
- Prereqs: Bayes filter, Markov localization, sampling / importance weighting, occupancy grid maps.
- Understand: represent belief as weighted samples; motion model sampling; sensor model weighting; resampling; global localization and multimodal beliefs vs. KF.
- Read: ALL (short conference paper).

**C1.5 Adapting the Sample Size in Particle Filters Through KLD-Sampling** — D. Fox — 2003 (IJRR) — https://doi.org/10.1177/0278364903022012001
- Why: The "A" in AMCL — adaptive particle count; explains the `min_particles`/`max_particles`/`pf_err`/`pf_z` parameters students will tune in Nav2.
- Prereqs: C1.4, KL divergence, chi-square basics.
- Understand: why a fixed particle count is wasteful; KLD bound on particles needed given occupied histogram bins; how this maps to AMCL parameters.
- Read: PARTIAL — introduction, the KLD-sampling derivation at an intuitive level, and experiments; skip detailed proofs.
- Companion: Nav2 AMCL documentation (navigate from https://docs.nav2.org/ — deep links moved in 2026, specific page URL UNVERIFIED).

Further (Track 1):
- Robust Monte Carlo Localization for Mobile Robots — Thrun, Fox, Burgard, Dellaert — 2001 (AIJ) — https://doi.org/10.1016/S0004-3702(01)00069-8 (Mixture-MCL; journal-length MCL treatment.)
- Particle Filters in Robotics (invited talk) — S. Thrun — 2002 (UAI) — https://arxiv.org/abs/1301.0607 (Short overview of PF uses in robotics.)
- Book: Probabilistic Robotics — Thrun, Burgard, Fox — 2005. The canonical text for Tracks 1–2. (Official site http://www.probabilistic-robotics.org/ returned HTTP 403 to automated check — UNVERIFIED reachability.)

---

## Track 2 — SLAM

**C2.1 Simultaneous Localization and Mapping: Part I** — H. Durrant-Whyte, T. Bailey — 2006 (IEEE RAM) — https://doi.org/10.1109/MRA.2006.1638022 · Part II (Bailey, Durrant-Whyte) https://doi.org/10.1109/MRA.2006.1678144
- Why: The classic two-part tutorial; defines the SLAM problem, its probabilistic form, and EKF-SLAM / Rao-Blackwellized particle filter solutions.
- Prereqs: Track 1 (KF, EKF, particle filters), coordinate frames.
- Understand: joint pose+map posterior; why landmark correlations matter; convergence properties; data association and loop closure as the hard parts; computational complexity issues (Part II).
- Read: Part I ALL; Part II PARTIAL — computational complexity, data association, and environment representation sections.

**C2.2 FastSLAM: A Factored Solution to the Simultaneous Localization and Mapping Problem** — M. Montemerlo, S. Thrun, D. Koller, B. Wegbreit — 2002 (AAAI) — https://cdn.aaai.org/AAAI/2002/AAAI02-089.pdf
- Why: Rao-Blackwellization — particles over trajectories, small independent EKFs per landmark; the idea behind GMapping, the long-time ROS default.
- Prereqs: C1.4, C2.1, conditional independence.
- Understand: why conditioning on the path factorizes the map; O(M log N) data structure; per-particle data association.
- Read: ALL (short), skim the tree data structure details.

**C2.3 A Tutorial on Graph-Based SLAM** — G. Grisetti, R. Kümmerle, C. Stachniss, W. Burgard — 2010 (IEEE ITS Magazine) — https://doi.org/10.1109/MITS.2010.939925
- Why: The modern formulation used by Cartographer, SLAM Toolbox, ORB-SLAM backends, g2o, GTSAM: front-end builds a pose graph, back-end solves nonlinear least squares.
- Prereqs: C2.1, nonlinear least squares (Gauss-Newton / Levenberg-Marquardt), sparse linear algebra, SE(2)/SE(3) basics.
- Understand: nodes/edges/information matrices; error functions on manifolds; sparsity; why loop closures fix drift.
- Read: ALL — implement a tiny 2D pose-graph optimizer as an exercise.

**C2.4 Real-Time Loop Closure in 2D LIDAR SLAM (Cartographer)** — W. Hess, D. Kohler, H. Rapp, D. Andor — 2016 (ICRA) — https://doi.org/10.1109/ICRA.2016.7487258 · also https://research.google/pubs/real-time-loop-closure-in-2d-lidar-slam/ · code https://github.com/cartographer-project/cartographer
- Why: Submaps + scan matching + branch-and-bound loop closure; real-time 2D lidar SLAM at building scale.
- Prereqs: C2.3, scan matching (correlative / ICP), occupancy grids.
- Understand: local SLAM (submaps) vs. global SLAM (sparse pose adjustment); branch-and-bound scan matching for fast loop closure.
- Read: ALL (short). Note: Cartographer is effectively unmaintained upstream; SLAM Toolbox (C2.5) is the Nav2 default.

**C2.5 SLAM Toolbox: SLAM for the Dynamic World** — S. Macenski, I. Jambrecic — 2021 (JOSS) — https://doi.org/10.21105/joss.02783 · code https://github.com/SteveMacenski/slam_toolbox
- Why: The default 2D SLAM in ROS 2/Nav2 (Karto-based scan matcher + Ceres pose graph); lifelong mapping, map merging, localization mode — what students actually run.
- Prereqs: C2.3, C2.4, ROS 2 basics.
- Understand: synchronous vs. asynchronous mapping modes; serialization and continuing a map; lifelong mapping and pose-graph pruning.
- Read: ALL (2-page JOSS paper) + the repo README (which is the real documentation).

**C2.6 ORB-SLAM: A Versatile and Accurate Monocular SLAM System** — R. Mur-Artal, J. M. M. Montiel, J. D. Tardós — 2015 (T-RO) — https://arxiv.org/abs/1502.00956
- Why: The canonical feature-based visual SLAM architecture: tracking / local mapping / loop closing threads, ORB features, covisibility graph, bag-of-words relocalization.
- Prereqs: C2.3, camera models, epipolar geometry, feature detection/matching, bundle adjustment.
- Understand: the three-thread architecture; keyframe selection and culling ("survival of the fittest"); map initialization for monocular; scale drift.
- Read: PARTIAL — system overview, tracking, local mapping, loop closing; skim the experiments.

**C2.7 ORB-SLAM3: An Accurate Open-Source Library for Visual, Visual-Inertial and Multi-Map SLAM** — C. Campos, R. Elvira, J. J. Gómez Rodríguez, J. M. M. Montiel, J. D. Tardós — 2020/2021 (T-RO) — https://arxiv.org/abs/2007.11898 · code https://github.com/UZ-SLAMLab/ORB_SLAM3
- Why: Current reference for visual and visual-inertial SLAM; adds IMU MAP initialization and the Atlas multi-map system.
- Prereqs: C2.6 (and ORB-SLAM2 for stereo/RGB-D), IMU preintegration basics.
- Understand: tightly-coupled VI MAP estimation; multi-map Atlas and map merging; what "place recognition with high recall" buys.
- Read: PARTIAL — system overview, visual-inertial initialization, map merging; skim evaluation tables. Note: GPL-3.0 code.

**C2.8 LOAM: Lidar Odometry and Mapping in Real-time** — J. Zhang, S. Singh — 2014 (RSS) — https://doi.org/10.15607/RSS.2014.X.007 · PDF https://www.roboticsproceedings.org/rss10/p07.pdf
- Why: Edge/planar feature extraction and split high-rate odometry / low-rate mapping; ancestor of LeGO-LOAM, LIO-SAM, FAST-LIO — the 3D lidar lineage.
- Prereqs: point clouds, ICP, C2.3.
- Understand: curvature-based feature selection; point-to-edge and point-to-plane residuals; why splitting odometry and mapping frequencies works.
- Read: ALL (conference paper).

**C2.9 Past, Present, and Future of Simultaneous Localization and Mapping: Towards the Robust-Perception Age** — C. Cadena, L. Carlone, H. Carrillo, Y. Latif, D. Scaramuzza, J. Neira, I. Reid, J. J. Leonard — 2016 (T-RO) — https://arxiv.org/abs/1606.05830
- Why: The field-defining survey: factor graphs / MAP as the standard formulation, robustness, metric vs. semantic maps, open problems.
- Prereqs: C2.1–C2.3.
- Understand: the "SLAM is solved?" debate; robust back-ends; map representations; semantic SLAM; where deep learning fits.
- Read: PARTIAL — anatomy of a modern SLAM system, robustness, representation, semantic mapping; skim the rest.

**C2.10 How NeRFs and 3D Gaussian Splatting are Reshaping SLAM: a Survey** — F. Tosi, Y. Zhang, Z. Gong, E. Sandström, S. Mattoccia, M. R. Oswald, M. Poggi — 2024 — https://arxiv.org/abs/2402.13255
- Why: Map of the neural/radiance-field SLAM wave (iMAP, NICE-SLAM, SplaTAM, MonoGS, …) — dense, photorealistic maps that are becoming useful for sim-to-real and digital twins.
- Prereqs: C2.9, NeRF (arXiv 2003.08934), 3D Gaussian Splatting (arXiv 2308.04079), CNN/MLP basics.
- Understand: implicit vs. explicit (Gaussian) scene representations; tracking-by-rendering; trade-offs vs. classical SLAM (speed, memory, robustness).
- Read: SKIM — introduction, taxonomy figures and summary tables; dive into one method family only.

Further (Track 2):
- Improved Techniques for Grid Mapping With Rao-Blackwellized Particle Filters (GMapping) — Grisetti, Stachniss, Burgard — 2007 (T-RO) — https://doi.org/10.1109/TRO.2006.889486
- ORB-SLAM2 — Mur-Artal, Tardós — 2016/2017 — https://arxiv.org/abs/1610.06475
- LIO-SAM: Tightly-coupled Lidar Inertial Odometry via Smoothing and Mapping — Shan et al. — 2020 — https://arxiv.org/abs/2007.00258
- NeRF — Mildenhall et al. — 2020 — https://arxiv.org/abs/2003.08934
- 3D Gaussian Splatting for Real-Time Radiance Field Rendering — Kerbl et al. — 2023 — https://arxiv.org/abs/2308.04079

---

## Track 3 — Planning and navigation

**C3.1 A Note on Two Problems in Connexion with Graphs** — E. W. Dijkstra — 1959 (Numerische Mathematik) — https://doi.org/10.1007/BF01386390
- Why: Shortest paths on graphs — the base of grid planners (Nav2 NavFn is Dijkstra/A*).
- Prereqs: graphs, priority queues, big-O.
- Understand: greedy expansion by cost-to-come; correctness intuition; relation to BFS and A*.
- Read: ALL (3 pages) — mostly for history; implement on an occupancy grid.

**C3.2 A Formal Basis for the Heuristic Determination of Minimum Cost Paths (A*)** — P. E. Hart, N. J. Nilsson, B. Raphael — 1968 (IEEE TSSC) — https://doi.org/10.1109/TSSC.1968.300136
- Why: A* — admissible heuristics and optimality; still the backbone of Nav2's Smac planners.
- Prereqs: C3.1.
- Understand: f = g + h; admissibility and consistency; why A* expands fewer nodes than Dijkstra; tie-breaking.
- Read: PARTIAL — the algorithm and admissibility/optimality results; skip formal proofs on first pass.

**C3.3 Rapidly-Exploring Random Trees: A New Tool for Path Planning (TR 98-11)** — S. M. LaValle — 1998 — http://msl.cs.illinois.edu/~lavalle/papers/Lav98c.pdf
- Why: Sampling-based planning in high-dimensional configuration spaces — the default family in MoveIt/OMPL for arms.
- Prereqs: configuration space, collision checking, nearest-neighbor search.
- Understand: Voronoi bias; extend step; probabilistic completeness; why sampling beats grids in high-D; kinodynamic extension idea.
- Read: ALL (short tech report).

**C3.4 Sampling-based Algorithms for Optimal Motion Planning (RRT*, PRM*)** — S. Karaman, E. Frazzoli — 2011 (IJRR) — https://arxiv.org/abs/1105.1186
- Why: Shows RRT is not asymptotically optimal and introduces RRT*/PRM* with rewiring.
- Prereqs: C3.3, PRM (Kavraki et al. 1996, see Further).
- Understand: asymptotic optimality; connection radius ~ (log n / n)^(1/d); rewiring; cost of optimality.
- Read: PARTIAL — introduction, algorithm descriptions, and the main theorems' statements; skip proofs.

**C3.5 The Dynamic Window Approach to Collision Avoidance** — D. Fox, W. Burgard, S. Thrun — 1997 (IEEE RAM) — https://doi.org/10.1109/100.580977
- Why: Velocity-space local planning under dynamic constraints; ancestor of ROS DWB controller.
- Prereqs: differential-drive kinematics, costmaps, C3.2.
- Understand: admissible velocities; dynamic window from acceleration limits; the heading/clearance/velocity objective; local-minimum problems.
- Read: ALL.

**C3.6 Integrated Online Trajectory Planning and Optimization in Distinctive Topologies (TEB)** — C. Rösmann, F. Hoffmann, T. Bertram — 2017 (RAS) — https://doi.org/10.1016/j.robot.2016.11.007 · code https://github.com/rst-tu-dortmund/teb_local_planner
- Why: Timed Elastic Band — local trajectory optimization as a sparse graph problem, planning in parallel across homotopy classes; popular for car-like robots.
- Prereqs: C2.3 (graph optimization / g2o), C3.5, nonholonomic kinematics.
- Understand: trajectory as a sequence of timed poses; soft constraints; homotopy-class exploration to avoid local minima.
- Read: PARTIAL — the TEB formulation and topology exploration sections; skim experiments. (TEB is not maintained in Nav2 for recent ROS 2 releases — MPPI is the recommended successor; status per repo, UNVERIFIED for Jazzy+.)

**C3.7 Aggressive Driving with Model Predictive Path Integral Control** — G. Williams, P. Drews, B. Goldfain, J. M. Rehg, E. A. Theodorou — 2016 (ICRA) — https://doi.org/10.1109/ICRA.2016.7487277 · and Information Theoretic MPC for Model-Based Reinforcement Learning — G. Williams et al. — 2017 (ICRA) — https://doi.org/10.1109/ICRA.2017.7989202
- Why: MPPI — sampling-based MPC that handles non-convex costs and parallelizes on GPU/SIMD; Nav2's MPPI controller is the modern default local controller.
- Prereqs: MPC, stochastic optimal control intuition, importance sampling, C3.5.
- Understand: sample K noisy control sequences, roll out, weight by exp(-cost/λ), average; temperature λ; why it tolerates non-differentiable costs.
- Read: 2016 paper ALL (practical algorithm); 2017 paper PARTIAL (the information-theoretic derivation, at intuition level).

**C3.8 The Marathon 2: A Navigation System** — S. Macenski, F. Martín, R. White, J. Ginés Clavero — 2020 (IROS) — https://arxiv.org/abs/2003.00368 · code https://github.com/ros-navigation/navigation2 · docs https://docs.nav2.org/
- Why: The Nav2 architecture paper: behavior-tree navigator, lifecycle nodes, plugin servers (planner/controller/behaviors), layered costmaps; validated with long campus marathons.
- Prereqs: ROS 2 (nodes, actions, lifecycle), C1.4, C3.2, C3.5, C3.10.
- Understand: why BTs instead of an FSM; server/plugin decomposition; lifecycle management; how recovery behaviors are orchestrated.
- Read: ALL.

**C3.9 Open-Source, Cost-Aware Kinematically Feasible Planning for Mobile and Surface Robotics (Smac Planner)** — S. Macenski, M. Booker, J. Wallace, T. Fischer — 2024 — https://arxiv.org/abs/2401.13078
- Why: The Nav2 Smac planners: 2D A*, Hybrid-A* and State Lattice with cost-awareness — what replaced NavFn for car-like and legged robots.
- Prereqs: C3.2, Hybrid A* (Dolgov et al., see Further), motion primitives, costmaps.
- Understand: Hybrid-A* continuous-state expansions; state lattice primitives; heuristics (obstacle + Dubins/Reeds-Shepp); analytic expansions; smoothing.
- Read: PARTIAL — planner descriptions and heuristics; skim benchmarks.

**C3.10 Behavior Trees in Robotics and AI: An Introduction** — M. Colledanchise, P. Ögren — 2017/2018 (book, CRC Press; arXiv version) — https://arxiv.org/abs/1709.00084 · library https://github.com/BehaviorTree/BehaviorTree.CPP
- Why: The standard reference for BTs — the execution layer in Nav2 and an increasingly common target for LLM-generated plans (Track 8).
- Prereqs: finite state machines, basic software architecture.
- Understand: sequence/fallback/parallel/decorator nodes; tick semantics; reactivity; modularity vs. FSMs; BTs as generalization of subsumption/decision trees.
- Read: PARTIAL — the introduction/how BTs work chapter, BT design principles, and the comparison with FSMs; skip formal analysis chapters.

Further (Track 3):
- Probabilistic Roadmaps for Path Planning in High-Dimensional Configuration Spaces — Kavraki, Švestka, Latombe, Overmars — 1996 (T-RA) — https://doi.org/10.1109/70.508439
- Path Planning for Autonomous Vehicles in Unknown Semi-structured Environments (Hybrid A*) — Dolgov, Thrun, Montemerlo, Diebel — 2010 (IJRR) — https://doi.org/10.1177/0278364909359210
- Regulated Pure Pursuit for Robot Path Tracking — Macenski et al. — 2023 — https://arxiv.org/abs/2305.20026
- A Survey of Behavior Trees in Robotics and AI — Iovino, Scukins, Styrud, Ögren, Smith — 2020/2022 (RAS) — https://arxiv.org/abs/2005.05842
- From the Desks of ROS Maintainers: A Survey of Modern & Capable Mobile Robotics Algorithms in ROS 2 — Macenski, Moore, Lu, Merzlyakov, Ferguson — 2023 — https://arxiv.org/abs/2307.15236 (excellent map of what's actually deployed)
- Robot Operating System 2: Design, Architecture, and Uses in the Wild — Macenski, Foote, Gerkey, Lalancette, Woodall — 2022 (Science Robotics) — https://doi.org/10.1126/scirobotics.abm6074 · https://arxiv.org/abs/2211.07752

---

## Track 4 — Manipulation and kinematics

**C4.1 A Kinematic Notation for Lower-Pair Mechanisms Based on Matrices** — J. Denavit, R. S. Hartenberg — 1955 (J. Applied Mechanics) — https://doi.org/10.1115/1.4011045
- Why: DH parameters — the classic convention for serial-arm forward kinematics; still in datasheets and textbooks.
- Prereqs: homogeneous transforms, rotation matrices.
- Understand: four parameters per joint; frame-assignment rules; how FK chains transforms. Contrast with product-of-exponentials (Modern Robotics) and URDF, which is what ROS actually uses.
- Read: SKIM — learn DH from a textbook (Modern Robotics, Lynch & Park — free PDF https://hades.mech.northwestern.edu/images/7/7f/MR.pdf, book page https://hades.mech.northwestern.edu/index.php/Modern_Robotics); cite the paper for history.

**C4.2 Reducing the Barrier to Entry of Complex Robotic Software: a MoveIt! Case Study** — D. Coleman, I. Șucan, S. Chitta, N. Correll — 2014 (JOSER) — https://arxiv.org/abs/1404.3785 · MoveIt 2 docs https://moveit.picknik.ai/main/index.html · code https://github.com/moveit/moveit2
- Why: The architecture/design paper for MoveIt (move_group, planning scene, plugin planners via OMPL, Setup Assistant). There is no dedicated MoveIt 2 paper (searched; none found — treat any "MoveIt 2 paper" citation as UNVERIFIED); MoveIt 2 docs are the practical reference.
- Prereqs: C3.3, URDF/SRDF, ROS 2, IK basics.
- Understand: move_group pipeline; planning scene and collision checking; kinematics plugins; why the Setup Assistant mattered for adoption.
- Read: ALL (short), then MoveIt 2 "Getting Started" and "Motion Planning Pipeline" tutorials.

**C4.3 Data-Driven Grasp Synthesis — A Survey** — J. Bohg, A. Morales, T. Asfour, D. Kragic — 2013/2014 (T-RO) — https://arxiv.org/abs/1309.2660
- Why: The taxonomy bridging analytic grasping (force closure) and learning-based grasping; frames known / familiar / unknown objects.
- Prereqs: rigid-body contact basics, friction cones, point clouds.
- Understand: analytic vs. data-driven grasp synthesis; grasp representation choices; the known/familiar/unknown object split.
- Read: PARTIAL — introduction, analytic-vs-data-driven background, and the unknown-objects section.

**C4.4 Dex-Net 2.0: Deep Learning to Plan Robust Grasps with Synthetic Point Clouds and Analytic Grasp Metrics** — J. Mahler, J. Liang, S. Niyaz, et al., K. Goldberg — 2017 (RSS) — https://arxiv.org/abs/1703.09312 · code https://github.com/BerkeleyAutomation/dex-net
- Why: Train a grasp-quality CNN (GQ-CNN) on millions of synthetic depth images labeled by analytic robustness metrics — the template for "sim-labeled data + CNN" grasping.
- Prereqs: CNNs, C4.3, depth images, domain gap intuition.
- Understand: parallel-jaw planar grasps from depth; robust epsilon-quality under uncertainty; cross-entropy method for grasp sampling.
- Read: PARTIAL — problem statement, dataset generation, GQ-CNN, and physical experiments.

**C4.5 GraspNet-1Billion: A Large-Scale Benchmark for General Object Grasping** — H.-S. Fang, C. Wang, M. Gou, C. Lu — 2020 (CVPR) — https://openaccess.thecvf.com/content_CVPR_2020/html/Fang_GraspNet-1Billion_A_Large-Scale_Benchmark_for_General_Object_Grasping_CVPR_2020_paper.html · DOI https://doi.org/10.1109/CVPR42600.2020.01146 · project https://www.graspnet.net/
- Why: Standard 6-DoF grasp benchmark from real RGB-D scenes with dense annotations and an online evaluation metric.
- Prereqs: C4.3, point-cloud networks (PointNet++ basics), 6-DoF poses.
- Understand: 6-DoF grasp parameterization; the analytic evaluation protocol; the approach/operation network decomposition.
- Read: PARTIAL — dataset, evaluation metric, baseline network overview.

**C4.6 Contact-GraspNet: Efficient 6-DoF Grasp Generation in Cluttered Scenes** — M. Sundermeyer, A. Mousavian, R. Triebel, D. Fox — 2021 (ICRA) — https://arxiv.org/abs/2103.14127 · code https://github.com/NVlabs/contact_graspnet
- Why: Grasps anchored on observed contact points (4-DoF per point) — efficient, works in clutter from a single depth view; widely used as an off-the-shelf grasp proposer.
- Prereqs: C4.5, PointNet++, segmentation masks.
- Understand: contact-point grasp representation reduces dimensionality; training on synthetic scenes (ACRONYM); combining with instance masks for target grasping.
- Read: ALL (short).

**C4.7 AnyGrasp: Robust and Efficient Grasp Perception in Spatial and Temporal Domains** — H.-S. Fang, C. Wang, H. Fang, et al., C. Lu — 2022/2023 (T-RO) — https://arxiv.org/abs/2212.08333 · SDK https://github.com/graspnet/anygrasp_sdk
- Why: Dense 6-DoF grasp detection plus temporal grasp tracking for dynamic objects; commonly used as a "grasp skill" beneath LLM/VLM planners.
- Prereqs: C4.5, C4.6.
- Understand: geometry-based grasp learning with center-of-mass awareness; temporal correspondence; real-world clutter clearing results.
- Read: PARTIAL — method overview and real-robot experiments. Note: SDK requires a license key (per repo; check terms).

Further (Track 4):
- cuRobo: Parallelized Collision-Free Minimum-Jerk Robot Motion Generation — Sundaralingam et al. (NVIDIA) — 2023 — https://arxiv.org/abs/2310.17274 · https://github.com/NVlabs/curobo (GPU motion generation; integrates with MoveIt/Isaac)
- FoundationPose (see C5.13) — 6D object pose for manipulation.

---

## Track 5 — Vision for robots

**C5.1 ImageNet Classification with Deep Convolutional Neural Networks (AlexNet)** — A. Krizhevsky, I. Sutskever, G. E. Hinton — 2012 (NeurIPS) — https://proceedings.neurips.cc/paper/2012/hash/c399862d3b9d6b76c8436e924a68c45b-Abstract.html · CACM reprint https://doi.org/10.1145/3065386
- Why: The 2012 moment deep learning took over vision; context for everything in Tracks 5–7.
- Prereqs: CNNs, SGD, backprop.
- Understand: ReLU, dropout, data augmentation, GPU training — and why scale mattered.
- Read: SKIM — historical; an AI engineer likely knows this already.

**C5.2 Deep Residual Learning for Image Recognition (ResNet)** — K. He, X. Zhang, S. Ren, J. Sun — 2015 (CVPR 2016) — https://arxiv.org/abs/1512.03385
- Why: Residual connections; ResNet-18 is still the vision encoder in ACT and Diffusion Policy.
- Prereqs: CNNs, C5.1.
- Understand: the degradation problem; identity shortcuts; bottleneck blocks.
- Read: PARTIAL — introduction, residual learning, architectures.

**C5.3 You Only Look Once: Unified, Real-Time Object Detection (YOLO)** — J. Redmon, S. Divvala, R. Girshick, A. Farhadi — 2015 (CVPR 2016) — https://arxiv.org/abs/1506.02640 · current practice: Ultralytics docs https://docs.ultralytics.com/ (YOLO26 page https://docs.ultralytics.com/models/yolo26/)
- Why: Single-shot real-time detection — the practical detector on robots and Jetsons. As of 2026-09 Ultralytics recommends YOLO26 (NMS-free end-to-end option, edge-optimized) or YOLO11; YOLO27 announced as in development. Ultralytics code/models are AGPL-3.0 (enterprise license for closed products) — a real licensing constraint for course projects.
- Prereqs: CNNs, bounding boxes, IoU, NMS.
- Understand: grid-based detection as regression; speed/accuracy trade-off; then, from the docs: export to TensorRT/ONNX for Jetson, tasks (detect/segment/pose/OBB/track).
- Read: paper PARTIAL (unified detection section, limitations); docs: quickstart, predict, export, Jetson guide.

**C5.4 An Image is Worth 16x16 Words: Transformers for Image Recognition at Scale (ViT)** — A. Dosovitskiy, L. Beyer, A. Kolesnikov, et al. — 2020 (ICLR 2021) — https://arxiv.org/abs/2010.11929
- Why: Vision transformers — the encoders inside CLIP, SigLIP, DINOv2, SAM, and every VLA.
- Prereqs: transformers ("Attention Is All You Need", Vaswani et al. 2017 — https://arxiv.org/abs/1706.03762), C5.2.
- Understand: patch embedding; class token; why ViTs need scale/pre-training; inductive bias vs. CNNs.
- Read: PARTIAL — method section and the key scaling experiments.

**C5.5 End-to-End Object Detection with Transformers (DETR)** — N. Carion, F. Massa, G. Synnaeve, N. Usunier, A. Kirillov, S. Zagoruyko — 2020 (ECCV) — https://arxiv.org/abs/2005.12872
- Why: Detection as set prediction with bipartite matching and object queries — the design reused in Grounding DINO, SAM 3, and ACT's transformer decoder.
- Prereqs: C5.4, Hungarian matching.
- Understand: object queries; Hungarian loss; no anchors/NMS; slow convergence and later fixes (Deformable DETR, DINO).
- Read: PARTIAL — the DETR model and set-prediction loss sections.

**C5.6 Learning Transferable Visual Models From Natural Language Supervision (CLIP)** — A. Radford, J. W. Kim, C. Hallacy, et al. — 2021 (ICML) — https://arxiv.org/abs/2103.00020 · code https://github.com/openai/CLIP
- Why: Image–text contrastive pre-training → zero-shot recognition; the root of open-vocabulary perception and of VLM vision towers (SigLIP in PaliGemma/pi0).
- Prereqs: C5.4, contrastive learning (InfoNCE).
- Understand: dual encoders + contrastive loss; prompt templates; zero-shot transfer; known failure modes (counting, spatial relations — exactly what robots need).
- Read: PARTIAL — approach section, zero-shot transfer, limitations. Skip the long appendix of benchmarks.

**C5.7 Simple Open-Vocabulary Object Detection with Vision Transformers (OWL-ViT)** — M. Minderer, A. Gritsenko, A. Stone, et al. — 2022 (ECCV) — https://arxiv.org/abs/2205.06230 · and Scaling Open-Vocabulary Object Detection (OWLv2) — M. Minderer, A. Gritsenko, N. Houlsby — 2023 (NeurIPS) — https://arxiv.org/abs/2306.09683 · code https://github.com/google-research/scenic/tree/main/scenic/projects/owl_vit · HF https://huggingface.co/docs/transformers/model_doc/owlv2
- Why: Text- or image-query detection built on CLIP-style backbones; OWLv2 self-trains on web-scale pseudo-labels. Easy "find the red cup" for robot agents.
- Prereqs: C5.5, C5.6.
- Understand: per-token box + class-embedding heads; image-conditioned one-shot detection; self-training at scale (OWL-ST).
- Read: OWL-ViT PARTIAL (method); OWLv2 SKIM (self-training recipe and results).

**C5.8 Grounding DINO: Marrying DINO with Grounded Pre-Training for Open-Set Object Detection** — S. Liu, Z. Zeng, T. Ren, et al. — 2023 (ECCV 2024) — https://arxiv.org/abs/2303.05499 · code https://github.com/IDEA-Research/GroundingDINO
- Why: Strong open-set detector with referring-expression support; "Grounded-SAM" (Grounding DINO + SAM) became the default open-vocabulary segmentation pipeline in robotics.
- Prereqs: C5.5, C5.6.
- Understand: tight language–vision fusion (feature enhancer, language-guided query selection, cross-modality decoder); open-set vs. closed-set evaluation.
- Read: PARTIAL — method overview; skim ablations.

**C5.9 Segment Anything (SAM)** — A. Kirillov, E. Mintun, N. Ravi, et al. — 2023 (ICCV) — https://arxiv.org/abs/2304.02643 · code https://github.com/facebookresearch/segment-anything
- Why: Promptable segmentation foundation model (points/boxes → masks) with the SA-1B data engine.
- Prereqs: C5.4, segmentation basics.
- Understand: heavy image encoder + light prompt encoder/mask decoder (encode once, prompt many times — good for robots); ambiguity-aware multi-mask output; the data engine.
- Read: PARTIAL — task, model, data engine; skim zero-shot experiments.
  - **SAM 2: Segment Anything in Images and Videos** — N. Ravi, V. Gabeur, Y.-T. Hu, et al. — 2024 — https://arxiv.org/abs/2408.00714 · https://github.com/facebookresearch/sam2 — adds streaming memory for video object tracking; directly useful for tracking a grasped object. Read: model section (memory attention/bank).
  - **SAM 3: Segment Anything with Concepts** — N. Carion, L. Gustafson, Y.-T. Hu, et al. — 2025 (arXiv 2025-11-20) — https://arxiv.org/abs/2511.16719 · https://github.com/facebookresearch/sam3 — "promptable concept segmentation": short noun phrase or exemplar → all matching instances in images/video, unifying detection + segmentation + tracking. Read: task definition and model overview; SKIM the rest.

**C5.10 DINOv2: Learning Robust Visual Features without Supervision** — M. Oquab, T. Darcet, T. Moutakanni, et al. — 2023 (TMLR) — https://arxiv.org/abs/2304.07193 · code https://github.com/facebookresearch/dinov2
- Why: General-purpose self-supervised dense features; used in OpenVLA's fused encoder, Depth Anything, FoundationPose-style pipelines and many policy encoders.
- Prereqs: C5.4, self-supervised learning (self-distillation).
- Understand: why SSL features transfer to dense tasks (depth, correspondence); data curation (LVD-142M); frozen-feature linear probing.
- Read: PARTIAL — data pipeline and results on dense tasks.
  - **DINOv3** — O. Siméoni, H. V. Vo, M. Seitzer, et al. — 2025 (arXiv 2025-08-13) — https://arxiv.org/abs/2508.10104 · https://github.com/facebookresearch/dinov3 — scales to 7B with "Gram anchoring" to fix dense-feature degradation in long training; distilled smaller variants. Read: SKIM (intro, Gram anchoring idea, dense results). License differs from DINOv2 (custom DINOv3 license per repo; UNVERIFIED details).

**C5.11 Depth Anything V2** — L. Yang, B. Kang, Z. Huang, Z. Zhao, X. Xu, J. Feng, H. Zhao — 2024 (NeurIPS) — https://arxiv.org/abs/2406.09414 · code https://github.com/DepthAnything/Depth-Anything-V2
- Why: Robust monocular (relative and metric) depth from a single RGB camera — lets a camera-only hobby robot get usable depth.
- Prereqs: C5.10, depth estimation basics, teacher–student distillation.
- Understand: synthetic-only labeled data for fine detail + large-scale pseudo-labeled real images; relative vs. metric depth heads; small models for edge devices. Licenses differ by model size (Small Apache-2.0, larger ones CC-BY-NC per repo — verify before use).
- Read: PARTIAL — the data-centric motivation and pipeline sections.

**C5.12 FoundationPose: Unified 6D Pose Estimation and Tracking of Novel Objects** — B. Wen, W. Yang, J. Kautz, S. Birchfield — 2023 (CVPR 2024) — https://arxiv.org/abs/2312.08344 · code https://github.com/NVlabs/FoundationPose
- Why: Model-based or model-free (few reference images) 6D pose estimation and tracking of unseen objects without fine-tuning; a building block for precise manipulation (and in NVIDIA Isaac ROS).
- Prereqs: 6D pose, render-and-compare, C5.10, transformers.
- Understand: neural object field for model-free setup; LLM-aided synthetic data; pose hypothesis generation + refinement + hierarchical ranking.
- Read: PARTIAL — method overview; skip implementation details. Note: NVIDIA source-code license (non-commercial, per repo; UNVERIFIED exact terms).

Further (Track 5):
- Attention Is All You Need — Vaswani et al. — 2017 — https://arxiv.org/abs/1706.03762 (prerequisite for C5.4 onward)
- Depth Anything (v1) repo — https://github.com/LiheYoung/Depth-Anything

---

## Track 6 — Reinforcement learning for robots

**C6.1 Playing Atari with Deep Reinforcement Learning (DQN)** — V. Mnih, K. Kavukcuoglu, D. Silver, et al. — 2013 (NeurIPS DL Workshop) — https://arxiv.org/abs/1312.5602 · Nature version: Human-level control through deep reinforcement learning — 2015 — https://doi.org/10.1038/nature14236
- Why: Start of deep RL: Q-learning with CNNs, experience replay, target networks.
- Prereqs: MDPs, Bellman equations, Q-learning, CNNs.
- Understand: why naive neural Q-learning diverges and how replay + target nets stabilize it; discrete action limitation (why robots need C6.2/C6.3).
- Read: 2013 arXiv ALL (short); Nature paper PARTIAL (methods).

**C6.2 Proximal Policy Optimization Algorithms (PPO)** — J. Schulman, F. Wolski, P. Dhariwal, A. Radford, O. Klimov — 2017 — https://arxiv.org/abs/1707.06347
- Why: The workhorse on-policy algorithm for massively parallel sim RL (legged locomotion in Isaac Lab / MuJoCo Playground uses PPO).
- Prereqs: policy gradients, advantage estimation (GAE), trust regions intuition.
- Understand: clipped surrogate objective; multiple epochs on the same batch; why it's robust and simple; key hyperparameters.
- Read: ALL (short). Pair with a clean implementation (e.g. rsl_rl or CleanRL).

**C6.3 Soft Actor-Critic: Off-Policy Maximum Entropy Deep RL with a Stochastic Actor (SAC)** — T. Haarnoja, A. Zhou, P. Abbeel, S. Levine — 2018 (ICML) — https://arxiv.org/abs/1801.01290
- Why: Sample-efficient off-policy continuous control; the base of real-world RL systems (SERL/HIL-SERL, LeRobot's RL stack).
- Prereqs: C6.1, C6.2, actor-critic, entropy.
- Understand: maximum-entropy objective; soft Q-functions; reparameterized stochastic actor; automatic temperature (in the follow-up "SAC Algorithms and Applications").
- Read: PARTIAL — preliminaries, soft policy iteration intuition, the SAC algorithm; skip proofs.

**C6.4 Domain Randomization for Transferring Deep Neural Networks from Simulation to the Real World** — J. Tobin, R. Fong, A. Ray, J. Schneider, W. Zaremba, P. Abbeel — 2017 (IROS) — https://arxiv.org/abs/1703.06907
- Why: Randomize rendering (textures, lighting, camera) so the real world looks like "just another variation"; the core sim-to-real idea.
- Prereqs: CNNs, simulation basics.
- Understand: the reality gap; visual randomization vs. dynamics randomization; why variety beats fidelity for perception.
- Read: ALL (short).

**C6.5 Learning Dexterous In-Hand Manipulation** — OpenAI (M. Andrychowicz, B. Baker, M. Chociej, et al.) — 2018 (IJRR 2020) — https://arxiv.org/abs/1808.00177
- Why: PPO + massive domain randomization (physics and vision) + LSTM memory → cube reorientation on a real Shadow Hand; the landmark sim-to-real RL result. (OpenAI blog page returns 403 to automated check; arXiv is canonical.)
- Prereqs: C6.2, C6.4, recurrent policies, distributed training.
- Understand: what was randomized; memory as implicit system identification; asymmetric actor-critic (privileged critic); emergent human-like finger gaits.
- Read: PARTIAL — task/system overview, transfer via randomization, results and analysis; skim infrastructure.

**C6.6 Learning Agile and Dynamic Motor Skills for Legged Robots** — J. Hwangbo, J. Lee, A. Dosovitskiy, D. Bellicoso, V. Tsounis, V. Koltun, M. Hutter — 2019 (Science Robotics) — https://arxiv.org/abs/1901.08652
- Why: ANYmal sim-to-real locomotion; the key trick was a learned actuator network to close the dynamics gap.
- Prereqs: C6.2, C6.4, system identification, legged robot basics.
- Understand: actuator net (learn motor dynamics from data); curriculum; why policies transferred where hand-built models failed.
- Read: PARTIAL — results overview and methods (actuator modeling, training).

**C6.7 Learning to Walk in Minutes Using Massively Parallel Deep Reinforcement Learning** — N. Rudin, D. Hoeller, P. Reist, M. Hutter — 2021 (CoRL) — https://arxiv.org/abs/2109.11978 · code https://github.com/leggedrobotics/legged_gym
- Why: Thousands of GPU-parallel envs + PPO → a walking policy in minutes on one GPU; the template for Isaac Lab / MuJoCo Playground locomotion (student-reproducible).
- Prereqs: C6.2, C6.6, GPU simulation.
- Understand: how PPO hyperparameters change with massive parallelism (batch size, horizon); terrain curriculum; reward shaping terms; time-out bootstrapping.
- Read: ALL (short), then run legged_gym-style training in Isaac Lab or MuJoCo Playground.

**C6.8 Precise and Dexterous Robotic Manipulation via Human-in-the-Loop Reinforcement Learning (HIL-SERL)** — J. Luo, C. Xu, J. Wu, S. Levine — 2024 (Science Robotics 2025) — https://arxiv.org/abs/2410.21845 · code https://github.com/rail-berkeley/hil-serl
- Why: Real-world RL (no sim) reaching near-perfect success in 1–2.5 h per task using demos + human corrections + learned reward classifier; implemented in LeRobot — a realistic RL project for the course arm.
- Prereqs: C6.3, C7.1 (DAgger idea), reward classifiers, SERL (see Further).
- Understand: RLPD-style mixing of demos and online data; human interventions as training data; binary reward classifiers; why pretrained vision backbones make real-world RL feasible.
- Read: PARTIAL — system overview, method, and the real-robot results/analysis.

Further (Track 6):
- Learning Quadrupedal Locomotion over Challenging Terrain — Lee, Hwangbo, Wellhausen, Koltun, Hutter — 2020 (Science Robotics) — https://arxiv.org/abs/2010.11251 (teacher–student privileged learning)
- SERL: A Software Suite for Sample-Efficient Robotic RL — Luo et al. — 2024 — https://arxiv.org/abs/2401.16013
- Mastering Diverse Domains through World Models (DreamerV3) — Hafner et al. — 2023 — https://arxiv.org/abs/2301.04104
- TD-MPC2: Scalable, Robust World Models for Continuous Control — Hansen, Su, Wang — 2023 — https://arxiv.org/abs/2310.16828 (TD-MPC v1: https://arxiv.org/abs/2203.04955 — in LeRobot)
- MuJoCo: A physics engine for model-based control — Todorov, Erez, Tassa — 2012 (IROS) — https://doi.org/10.1109/IROS.2012.6386109
- MuJoCo Playground — Zakka et al. — 2025 — https://arxiv.org/abs/2502.08844
- Isaac Lab: A GPU-Accelerated Simulation Framework for Multi-Modal Robot Learning — NVIDIA (Mittal et al.) — 2025 — https://arxiv.org/abs/2511.04831 (successor to Orbit, https://arxiv.org/abs/2301.04195)
- ManiSkill3 — Tao et al. — 2024 (RSS 2025) — https://arxiv.org/abs/2410.00425

---

## Track 7 — Imitation learning and vision-language-action models

**C7.1 A Reduction of Imitation Learning and Structured Prediction to No-Regret Online Learning (DAgger)** — S. Ross, G. J. Gordon, J. A. Bagnell — 2010/2011 (AISTATS) — https://arxiv.org/abs/1011.0686
- Why: Formalizes compounding error in behavior cloning (O(T²ε)) and fixes it by aggregating expert labels on the learner's own states; the theory behind "human corrections" in HIL-SERL, pi*0.6 and LeRobot's rollout tool.
- Prereqs: supervised learning, MDPs, distribution shift.
- Understand: covariate shift; why BC errors compound; the DAgger loop; no-regret guarantee (intuition only).
- Read: PARTIAL — introduction, preliminaries, the DAgger algorithm; skip the structured-prediction theory.

**C7.2 BC-Z: Zero-Shot Task Generalization with Robotic Imitation Learning** — E. Jang, A. Irpan, M. Khansari, D. Kappler, F. Ebert, C. Lynch, S. Levine, C. Finn — 2021 (CoRL) — https://arxiv.org/abs/2202.02005
- Why: Large-scale multi-task BC conditioned on language or human video embeddings; generalization to unseen tasks; shared-autonomy data collection with corrections.
- Prereqs: C7.1, C5.2, sentence embeddings.
- Understand: task conditioning (FiLM); data collection with interventions (HG-DAgger style); what zero-shot generalization means and its limits.
- Read: PARTIAL — system/data collection, model, and generalization results.

**C7.3 Learning Fine-Grained Bimanual Manipulation with Low-Cost Hardware (ALOHA + ACT)** — T. Z. Zhao, V. Kumar, S. Levine, C. Finn — 2023 (RSS) — https://arxiv.org/abs/2304.13705 · project https://tonyzhaozh.github.io/aloha/ · code https://github.com/tonyzhaozh/act
- Why: The policy students will train first (LeRobot's recommended starter). Action chunking + CVAE + transformer, from ~50 demos.
- Prereqs: C5.2, C5.5 (transformer encoder–decoder, queries), VAEs, C7.1.
- Understand: action chunking reduces effective horizon and compounding error; temporal ensembling; why a CVAE (multimodal human demos); the low-cost teleop hardware design.
- Read: ALL (the whole paper is practical).

**C7.4 Diffusion Policy: Visuomotor Policy Learning via Action Diffusion** — C. Chi, Z. Xu, S. Feng, E. Cousineau, Y. Du, B. Burchfiel, R. Tedrake, S. Song — 2023 (RSS; IJRR 2024) — https://arxiv.org/abs/2303.04137 · project https://diffusion-policy.cs.columbia.edu/ · code https://github.com/real-stanford/diffusion_policy
- Why: Denoising diffusion over action sequences handles multimodality and high-dimensional actions; now the standard action head (GR00T, Octo) and baseline.
- Prereqs: DDPM/diffusion models, C7.3, receding-horizon control, CNNs/transformers.
- Understand: action sequence prediction + receding horizon; visual conditioning (FiLM); CNN vs. transformer denoisers; why it beats explicit/implicit (energy-based) BC on multimodal demos; inference-step latency trade-off.
- Read: PARTIAL — method, key design decisions, and the "intriguing properties" discussion; skim the large benchmark tables.

**C7.5 RT-1: Robotics Transformer for Real-World Control at Scale** — A. Brohan, N. Brown, J. Carbajal, et al. (Google) — 2022 (RSS 2023) — https://arxiv.org/abs/2212.06817
- Why: Showed data scale (130k episodes, 700+ tasks, 13 robots, 17 months) + a transformer policy gives real-world generalization — the start of "robot foundation models".
- Prereqs: C5.2, C5.4, C7.2, tokenization.
- Understand: EfficientNet + FiLM language conditioning → TokenLearner → transformer → discretized action tokens; data diversity > quantity findings.
- Read: PARTIAL — model, data, and the generalization/data-ablation experiments.

**C7.6 RT-2: Vision-Language-Action Models Transfer Web Knowledge to Robotic Control** — A. Brohan, N. Brown, J. Carbajal, et al. (Google DeepMind) — 2023 (CoRL) — https://arxiv.org/abs/2307.15818
- Why: Coined "VLA": fine-tune a web-scale VLM to output actions as text tokens, co-training with web data; emergent semantic generalization.
- Prereqs: C7.5, VLMs (PaLI-X / PaLM-E), C5.6.
- Understand: actions-as-tokens; co-fine-tuning to avoid forgetting; emergent capabilities (symbol understanding, reasoning); latency limits of big models.
- Read: PARTIAL — method and emergent-capability evaluations.

**C7.7 Open X-Embodiment: Robotic Learning Datasets and RT-X Models** — Open X-Embodiment Collaboration (A. O'Neill et al.) — 2023 (ICRA 2024) — https://arxiv.org/abs/2310.08864 · project https://robotics-transformer-x.github.io/
- Why: 22 robots / 21 institutions / 527 skills / 1M+ trajectories standardized in RLDS; showed positive cross-embodiment transfer. The pre-training corpus behind Octo, OpenVLA, pi0.
- Prereqs: C7.5, C7.6, dataset formats.
- Understand: action-space heterogeneity and how it's normalized; RT-1-X / RT-2-X transfer results; dataset-mixture design.
- Read: SKIM — dataset section and main transfer results; the author list is most of the paper.

**C7.8 Octo: An Open-Source Generalist Robot Policy** — Octo Model Team (D. Ghosh, H. Walke, K. Pertsch, et al.) — 2024 (RSS) — https://arxiv.org/abs/2405.12213 · https://octo-models.github.io/
- Why: First well-documented open generalist policy designed for fine-tuning to new sensors/action spaces (27M/93M params); many design lessons for cross-embodiment models.
- Prereqs: C7.4, C7.7, transformers.
- Understand: block-wise attention with readout tokens; diffusion action head; adding new observation/action heads at fine-tune time; design-decision ablations.
- Read: PARTIAL — architecture, fine-tuning, and the design-decisions appendix/ablations.

**C7.9 OpenVLA: An Open-Source Vision-Language-Action Model** — M. J. Kim, K. Pertsch, S. Karamcheti, et al. — 2024 (CoRL) — https://arxiv.org/abs/2406.09246 · https://openvla.github.io/
- Why: The first strong open 7B VLA (Prismatic VLM: DINOv2 + SigLIP + Llama 2), beat RT-2-X with 7× fewer params; LoRA fine-tuning and quantization studies.
- Prereqs: C7.6, C7.7, C5.6, C5.10, LLM fine-tuning (LoRA), quantization.
- Understand: VLM → VLA conversion (action binning into tokens); fused vision encoders; parameter-efficient fine-tuning results; inference-speed limits.
- Read: PARTIAL — model, training data, fine-tuning & quantization experiments.
  - **Fine-Tuning Vision-Language-Action Models: Optimizing Speed and Success (OpenVLA-OFT)** — M. J. Kim, C. Finn, P. Liang — 2025 — https://arxiv.org/abs/2502.19645 · https://github.com/moojink/openvla-oft — parallel decoding + action chunking + continuous actions + L1 loss: LIBERO 76.5% → 97.1%, 26× faster. Read: ALL of the design-study sections — it's the clearest explanation of *why* modern VLAs use chunked continuous action heads.

**C7.10 π0: A Vision-Language-Action Flow Model for General Robot Control** — K. Black, N. Brown, D. Driess, et al. (Physical Intelligence) — 2024 — https://arxiv.org/abs/2410.24164 · https://www.pi.website/blog/pi0 · code https://github.com/Physical-Intelligence/openpi
- Why: PaliGemma VLM + separate "action expert" trained with flow matching, producing 50-step action chunks at up to 50 Hz; cross-embodiment pre-training on 10k h (7 robot types); laundry folding / box assembly. Open weights; runs in LeRobot.
- Prereqs: C7.4 (diffusion), flow matching, C7.6, mixture-of-experts style attention, C7.7.
- Understand: why continuous flow-matching action head instead of tokens; action expert with blockwise causal attention; pre-training vs. post-training (high-quality curated data) recipe.
- Read: PARTIAL — model, data & training recipe, and evaluation overview.
  - **FAST: Efficient Action Tokenization for Vision-Language-Action Models** — K. Pertsch, K. Stachowicz, B. Ichter, et al. — 2025 — https://arxiv.org/abs/2501.09747 · https://www.pi.website/research/fast — DCT + BPE compression of action chunks makes autoregressive VLAs (pi0-FAST) train ~5× faster. Read: the tokenization method section.

**C7.11 π0.5: a Vision-Language-Action Model with Open-World Generalization** — Physical Intelligence (K. Black, N. Brown, et al.) — 2025 — https://arxiv.org/abs/2504.16054 · https://www.pi.website/blog/pi05
- Why: Co-training on heterogeneous data (multiple robots, web data, high-level subtask prediction, verbal instructions) → cleaning entirely new homes; hierarchical inference (predict subtask text, then actions) in one model. Open weights (openpi, LeRobot).
- Prereqs: C7.10, C7.7, multi-task co-training.
- Understand: what each data source contributes (the ablations); high-level + low-level inference in one model; discrete-token pre-training then flow-matching action expert.
- Read: PARTIAL — method, co-training data, and the data-ablation experiments.
  - **Knowledge Insulating Vision-Language-Action Models: Train Fast, Run Fast, Generalize Better** — D. Driess, J. T. Springenberg, B. Ichter, et al. — 2025 — https://arxiv.org/abs/2505.23705 · https://www.pi.website/research/knowledge_insulation — stop gradients from the action expert into the VLM backbone so VLM knowledge is preserved. Read: method section.
  - **Real-Time Execution of Action Chunking Flow Policies (RTC)** — K. Black, M. Y. Galliker, S. Levine — 2025 — https://arxiv.org/abs/2506.07339 — inpainting-style chunk blending so slow VLAs move smoothly; relevant for Jetson/async inference. Read: method.

**C7.12 GR00T N1: An Open Foundation Model for Generalist Humanoid Robots** — NVIDIA (J. Bjorck et al.) — 2025 — https://arxiv.org/abs/2503.14734 · code https://github.com/NVIDIA/Isaac-GR00T · N1.7 blog (2026-04-17) https://huggingface.co/blog/nvidia/gr00t-n1-7
- Why: Open dual-system VLA (VLM "System 2" + diffusion-transformer "System 1") trained on a "data pyramid" of web/human video, synthetic (sim + neural-generated) data and real robot data; latent actions for action-less video. Current release N1.7 (3B, Cosmos-Reason2-2B backbone, 20k+ h egocentric human video, commercial license).
- Prereqs: C7.4, C7.10, C6.4 (sim data), world-model video generation basics.
- Understand: data pyramid; latent-action pseudo-labels (LAPA-style) and inverse-dynamics labels; embodiment-specific encoders/decoders; how N1.x changed (backbone, action dims/horizon, human-video scaling law).
- Read: PARTIAL — model architecture, data pyramid/training data sections; then skim the N1.7 blog for what changed.

**C7.13 SmolVLA: A Vision-Language-Action Model for Affordable and Efficient Robotics** — M. Shukor, D. Aubakirova, F. Capuano, et al. (Hugging Face) — 2025 — https://arxiv.org/abs/2506.01844 · model https://huggingface.co/lerobot/smolvla_base
- Why: ~450M-param VLA trained only on community LeRobot datasets, competitive with 10× larger VLAs, trainable on one GPU; asynchronous inference stack. The VLA students can actually fine-tune on the course arm.
- Prereqs: C7.10 (flow-matching action expert), C7.3, VLMs.
- Understand: size/efficiency choices (layer skipping, fewer visual tokens, interleaved cross/self-attention expert); community-data curation issues; async inference decoupling perception from action execution.
- Read: ALL (practical and short-ish).

**C7.14 Gemini Robotics: Bringing AI into the Physical World** — Gemini Robotics Team (S. Abeyruwan et al.), Google DeepMind — 2025 — https://arxiv.org/abs/2503.20020 · and Gemini Robotics 1.5: Pushing the Frontier of Generalist Robots with Advanced Embodied Reasoning, Thinking, and Motion Transfer — 2025 — https://arxiv.org/abs/2510.03342 · model page https://deepmind.google/models/gemini-robotics/
- Why: The closed frontier: Gemini-based VLA (cloud backbone + on-robot decoder) plus Gemini Robotics-ER embodied reasoning model; 1.5 adds "thinking before acting" and motion transfer across embodiments (ALOHA, Franka, Apptronik Apollo). As of 2026-09: Gemini Robotics 2 VLA and On-Device 2 are trusted-tester only; Gemini Robotics-ER 2 is public in the Gemini API (preview).
- Prereqs: C7.6, C7.10, VLMs, C8.x (embodied reasoning as planner).
- Understand: ER (pointing, 3D boxes, trajectories, success detection) vs. VLA roles; latency architecture; specialization via fine-tuning; semantic safety evaluation (ASIMOV).
- Read: PARTIAL — ER capability section and VLA architecture overview of the 2025 report; 1.5 report: thinking + motion-transfer sections. Both are long; skip most benchmark tables.

**C7.15 LeRobot: An Open-Source Library for End-to-End Robot Learning** — R. Cadene, S. Aliberts, F. Capuano, et al. (Hugging Face) — 2026 (ICLR 2026) — https://arxiv.org/abs/2602.22818 · code https://github.com/huggingface/lerobot
- Why: The system students use; explains LeRobotDataset design, hardware abstraction, policy zoo and async inference in one place.
- Prereqs: C7.3, C7.4, C7.13, basic ML engineering.
- Understand: dataset format (Parquet + MP4, streaming); robot/teleoperator abstractions; training/eval pipeline; async inference.
- Read: ALL — then the companion tutorial "Robot Learning: A Tutorial" (Capuano et al., 2025) https://arxiv.org/abs/2510.12403 as a textbook-style bridge from classical robotics to learned policies (read its imitation-learning and generalist-policy chapters).

Further (Track 7):
- What Matters in Learning from Offline Human Demonstrations for Robot Manipulation (robomimic) — Mandlekar et al. — 2021 (CoRL) — https://arxiv.org/abs/2108.03298
- Mobile ALOHA — Fu, Zhao, Finn — 2024 — https://arxiv.org/abs/2401.02117
- Universal Manipulation Interface (UMI) — Chi et al. — 2024 (RSS) — https://arxiv.org/abs/2402.10329
- DROID: A Large-Scale In-The-Wild Robot Manipulation Dataset — Khazatsky, Pertsch, Nair, et al. — 2024 (RSS) — https://arxiv.org/abs/2403.12945
- Behavior Generation with Latent Actions (VQ-BeT) — Lee et al. — 2024 (ICML) — https://arxiv.org/abs/2403.03181
- X-VLA: Soft-Prompted Transformer as Scalable Cross-Embodiment VLA — Zheng et al. — 2025 — https://arxiv.org/abs/2510.10274
- MolmoAct: Action Reasoning Models that can Reason in Space — Lee, Duan, Fang, et al. (Ai2) — 2025 — https://arxiv.org/abs/2508.07917
- π*0.6: a VLA That Learns From Experience (Recap: RL with experience & corrections) — Physical Intelligence (Amin et al.) — 2025 — https://arxiv.org/abs/2511.14759
- π0.7: a Steerable Generalist Robotic Foundation Model with Emergent Capabilities — Physical Intelligence (Ai, Amin, et al.) — 2026 — https://arxiv.org/abs/2604.15483 (closed weights; current frontier of compositional generalization)
- Cosmos World Foundation Model Platform for Physical AI — NVIDIA (Agarwal et al.) — 2025 — https://arxiv.org/abs/2501.03575
- Cosmos-Reason1: From Physical Common Sense to Embodied Reasoning — NVIDIA (Azzolini et al.) — 2025 — https://arxiv.org/abs/2503.15558
- V-JEPA 2: Self-Supervised Video Models Enable Understanding, Prediction and Planning — Assran, Bardes, Fan, et al. (Meta) — 2025 — https://arxiv.org/abs/2506.09985 (V-JEPA 2.1: https://arxiv.org/abs/2603.14482)
- Genie 3 (no paper; blog) — Google DeepMind — 2025 — https://deepmind.google/blog/genie-3-a-new-frontier-for-world-models/

---

## Track 8 — LLM agents for robots

**C8.1 Do As I Can, Not As I Say: Grounding Language in Robotic Affordances (SayCan)** — M. Ahn, A. Brohan, N. Brown, et al. (Google) — 2022 (CoRL) — https://arxiv.org/abs/2204.01691 · https://say-can.github.io/
- Why: First influential LLM-as-planner over a skill library, with learned affordance values grounding the choice in what's feasible.
- Prereqs: LLM prompting/likelihood scoring, value functions (C6.1–C6.3), skill libraries.
- Understand: score = LLM usefulness × value-function feasibility; limits (fixed skills, open-loop, slow).
- Read: PARTIAL — method and the failure analysis.

**C8.2 Inner Monologue: Embodied Reasoning through Planning with Language Models** — W. Huang, F. Xia, T. Xiao, et al. — 2022 (CoRL) — https://arxiv.org/abs/2207.05608 · https://innermonologue.github.io/
- Why: Closed-loop language feedback (success detection, scene description, human input) — the agent loop pattern.
- Prereqs: C8.1.
- Understand: feedback types and where they come from; replanning on failure; emergent behaviors (e.g. changing goals mid-task).
- Read: PARTIAL — method and feedback sources; skim experiments.

**C8.3 Code as Policies: Language Model Programs for Embodied Control** — J. Liang, W. Huang, F. Xia, P. Xu, K. Hausman, B. Ichter, P. Florence, A. Zeng — 2022 (ICRA 2023) — https://arxiv.org/abs/2209.07753 · https://code-as-policies.github.io/
- Why: LLM writes Python calling perception/control APIs, including spatial-geometric reasoning with numpy; hierarchical code generation for undefined functions. Still the dominant pattern (ER-2 agentic code execution).
- Prereqs: C8.1, Python APIs, few-shot prompting.
- Understand: prompt = API imports + examples; hierarchical generation; why code gives precise numeric/spatial behavior; sandboxing implications.
- Read: ALL.

**C8.4 ProgPrompt: Generating Situated Robot Task Plans using Large Language Models** — I. Singh, V. Blukis, A. Mousavian, et al. — 2022 (ICRA 2023) — https://arxiv.org/abs/2209.11302 · https://progprompt.github.io/
- Why: Program-like prompts listing available actions and objects, with assertions for precondition checking and recovery — early typed "tool spec".
- Prereqs: C8.3.
- Understand: situating the plan in the available action/object set; assertion-based feedback; evaluation in VirtualHome and a real arm.
- Read: PARTIAL — prompt design section and a skim of results.

**C8.5 ChatGPT for Robotics: Design Principles and Model Abilities** — S. Vemprala, R. Bonatti, A. Bucker, A. Kapoor (Microsoft) — 2023 (IEEE Access 2024) — https://arxiv.org/abs/2306.17582
- Why: A practitioner's recipe: define a high-level function library, describe it, let the model write code, keep a human in the loop in simulation, iterate — essentially today's MCP/tool-calling workflow.
- Prereqs: C8.3.
- Understand: API design for LLM consumption (descriptive names, docstrings); dialog-based correction; where the LLM fails (low-level control, unsafe assumptions).
- Read: PARTIAL — design principles section; skim the case studies.

**C8.6 VoxPoser: Composable 3D Value Maps for Robotic Manipulation with Language Models** — W. Huang, C. Wang, R. Zhang, Y. Li, J. Wu, L. Fei-Fei — 2023 (CoRL) — https://arxiv.org/abs/2307.05973 · https://voxposer.github.io/
- Why: LLM writes code that queries a VLM to compose 3D affordance/constraint value maps; an MPC planner optimizes trajectories through them — zero-shot manipulation without skill primitives.
- Prereqs: C8.3, C5.8/C5.9 (open-vocab perception), C3.7 (MPC), voxel grids.
- Understand: LLM for *constraints*, planner for *motion*; composing affordance/avoidance/rotation/velocity maps; online dynamics learning for contact-rich cases.
- Read: PARTIAL — method; skim experiments.

**C8.7 Enabling Novel Mission Operations and Interactions with ROSA: The Robot Operating System Agent** — R. Royce, M. Kaufmann, J. Becktor, et al. (NASA JPL) — 2024 — https://arxiv.org/abs/2410.06472 · code https://github.com/nasa-jpl/rosa (`jpl-rosa` 1.0.10, Mar 2026, Apache-2.0)
- Why: A deployed LangChain-based ReAct agent over ROS 1/ROS 2 introspection and control tools; a concrete template for the course's "LLM talks to ROS" lab. Contrast with MCP-based ros-mcp-server (https://github.com/robotmcp/ros-mcp-server, v3.1.0, Jun 2026, rosbridge-based).
- Prereqs: ROS 2 (topics, services, params), LLM tool calling / ReAct, C8.3.
- Understand: tool design for ROS introspection; prompt/"robot system prompt" customization; safety considerations and where the agent should not have authority.
- Read: ALL (short), then the repo README and tool definitions.

**C8.8 Jailbreaking LLM-Controlled Robots (RoboPAIR)** — A. Robey, Z. Ravichandran, V. Kumar, H. Hassani, G. J. Pappas — 2024 (ICRA 2025) — https://arxiv.org/abs/2410.13691 · https://robopair.org/
- Why: Demonstrated jailbreaks eliciting harmful physical actions from an NVIDIA Dolphins self-driving LLM (white-box), a Clearpath Jackal with GPT-4o planner (gray-box), and Unitree Go2's built-in GPT-3.5 (black-box), often at 100% attack success. Essential reading before giving an LLM actuators.
- Prereqs: C8.1–C8.3, LLM jailbreaks (PAIR), threat modeling.
- Understand: why text-only alignment doesn't transfer to embodied harm; attacker access levels; syntax checker to keep attacks executable; implications for guardrail design.
- Read: ALL.

**C8.9 Safety Guardrails for LLM-Enabled Robots (RoboGuard)** — Z. Ravichandran, A. Robey, V. Kumar, G. J. Pappas, H. Hassani — 2025 — https://arxiv.org/abs/2503.07885
- Why: The defensive follow-up: a root-of-trust LLM grounds predefined safety rules in the robot's world model into temporal-logic constraints; control synthesis then enforces them against a possibly-jailbroken planner.
- Prereqs: C8.8, temporal logic (LTL) basics, scene graphs.
- Understand: separating a trusted safety reasoner from the task planner; contextual grounding of rules; formal synthesis as the enforcement layer; overhead/latency trade-offs.
- Read: PARTIAL — method and evaluation of attack success reduction.

Further (Track 8):
- SayPlan: Grounding LLMs using 3D Scene Graphs for Scalable Robot Task Planning — Rana et al. — 2023 (CoRL) — https://arxiv.org/abs/2307.06135
- BTGenBot: Behavior Tree Generation for Robotic Tasks with Lightweight LLMs — Izzo, Bardaro, Matteucci — 2024 — https://arxiv.org/abs/2403.12761
- LLM-as-BT-Planner: Leveraging LLMs for Behavior Tree Generation in Robot Task Planning — Ao et al. — 2024 — https://arxiv.org/abs/2409.10444
- ROS-LLM: A ROS framework for embodied AI with task feedback and structured reasoning — Mower et al. — 2024 — https://arxiv.org/abs/2406.19741
- ReMEmbR: Long-Horizon Spatio-Temporal Memory for Robot Navigation — Anwar et al. (NVIDIA) — 2024 — https://arxiv.org/abs/2409.13682
- LLM-Driven Robots Risk Enacting Discrimination, Violence, and Unlawful Actions — Hundt et al. — 2024 — https://arxiv.org/abs/2406.08824
- BadRobot: Jailbreaking Embodied LLM Agents in the Physical World — Zhang et al. — 2024 — https://arxiv.org/abs/2407.20242
- Generating Robot Constitutions & Benchmarks for Semantic Safety (ASIMOV) — Sermanet et al. (Google DeepMind) — 2025 — https://arxiv.org/abs/2503.08663
- Gemini Robotics-ER 2 API docs (current first-party "robot brain" with function calling) — 2026 — https://ai.google.dev/gemini-api/docs/robotics-overview
- RAI (Robotec.ai) agent framework for ROS 2 — https://github.com/RobotecAI/rai

---

## Count

Core numbered entries: T1 5 · T2 10 · T3 10 · T4 7 · T5 12 (+SAM 2, SAM 3, DINOv3, OWLv2 as sub-entries) · T6 8 · T7 15 (+OFT, FAST, KI, RTC sub-entries) · T8 9 = 76 core headings (~84 with sub-entries). Further lists: ~50 optional items. For a ~50-entry course list, trim by taking every core entry marked ALL/PARTIAL and demoting SKIM entries (C1.1, C2.10, C4.1, C5.1, C7.7) to "Further".

## Verification notes / UNVERIFIED items

- Nav2 documentation deep links (AMCL, Smac, MPPI, BT pages) returned 404 on 2026-09-16 after a docs restructure; only the root https://docs.nav2.org/ is verified.
- http://www.probabilistic-robotics.org/ and the OpenAI "Learning Dexterity" blog returned HTTP 403 to automated checks (likely bot-blocking) — not linked as primary sources.
- No dedicated MoveIt 2 paper found; C4.2 uses the 2014 MoveIt! paper.
- License details marked UNVERIFIED in C5.10 (DINOv3), C5.11 (Depth Anything V2 per-size licenses), C5.12 (FoundationPose), C4.7 (AnyGrasp SDK license key), and TEB maintenance status in C3.6.
- Numeric facts quoted from paper bodies rather than abstracts (e.g. RT-1 "130k episodes / 13 robots / 17 months", Hwangbo actuator-net details) come from pre-2024 knowledge and were not re-read line by line; everything post-2024 was checked against abstracts, official blogs or READMEs.
