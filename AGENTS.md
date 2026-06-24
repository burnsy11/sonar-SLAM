# Repository Guidelines

## Motivation & Problem Definition
This repository supports autonomous inspection of underwater pile structures — vertical cylindrical elements common in marine infrastructure such as jetties, piers, and offshore platforms. Inspecting these structures requires a robot to navigate and localise within a spatially constrained, visually degraded underwater environment where optical sensing is unreliable.

The core problem is sonar-based localisation and mapping within a known pile field. A prior map of the pile layout is available: piles are cylindrical, uniform in diameter, and fixed at known spacings. The robot must exploit this structure — matching sonar observations against the prior pile map — to localise itself accurately enough to navigate between and around individual piles during inspection.

This prior knowledge of pile geometry and layout is a deliberate constraint: rather than treating the environment as unknown, the system leverages the regularity of the pile field to make localisation tractable in a GPS-denied, acoustically complex underwater setting.

## Project Structure & Module Organization
This repository is a ROS 2 workspace for sonar-based SLAM. Core runtime code lives in `bruce_slam/`: Python modules are under `bruce_slam/src/bruce_slam`, pybind11 C++ extensions are in `bruce_slam/src/bruce_slam/cpp`, launch files are in `bruce_slam/launch`, and tunable parameters are in `bruce_slam/config`. ROS interfaces live in `bruce_msgs/msg` and `bruce_msgs/srv`. `libnabo/` and `libpointmatcher/` are vendored dependencies; treat them as upstream code unless a task explicitly targets them.

## Build, Test, and Development Commands
Use the dev container or an equivalent ROS 2 Humble environment with submodules checked out:

```bash
git clone --recurse-submodules <repo>
colcon build --packages-up-to bruce_slam
source install/setup.bash
ros2 launch bruce_slam test_launch.py
ros2 launch bruce_slam slam_launch.py
colcon test --packages-select bruce_slam bruce_msgs
colcon test-result --verbose
```

`test_launch.py` is the usual local launch target. `slam_launch.py` starts the full stack. Use `ros2 bag play ... --clock` when running with `use_sim_time:=true`.

## Coding Style & Naming Conventions
Follow the existing style in each package. Python uses 4-space indentation, `snake_case` for modules, functions, parameters, and launch arguments, and `PascalCase` for classes. C++ in `bruce_slam/src/bruce_slam/cpp` also uses 4-space indentation with braces on the next line and should remain compatible with C++17. Keep ROS package names, node scripts, and config files lowercase with underscores, for example `feature_extraction_node.py` and `dead_reckoning.yaml`.

## Testing Guidelines
Current first-party testing is light. `bruce_slam` and `bruce_msgs` enable `ament_lint_auto` under `BUILD_TESTING`, so run `colcon test` before opening a PR. When adding tests, place Python tests near the owning package and use names that start with `test_`. For launch validation, prefer reproducible bag-based checks and document the bag, topics, and parameter overrides used.

## Commit & Pull Request Guidelines
Recent history uses short, imperative summaries such as `readded downsampling` and `tuned sfar settings, changed clustering and preprocessing`. Keep commits focused and descriptive, ideally under 72 characters. Pull requests should state the subsystem changed, note any config or launch-file impacts, link related issues, and include screenshots or RViz output when behavior or visualization changes.

## Configuration & Data Notes
Most tuning happens in `bruce_slam/config/feature.yaml` and `bruce_slam/config/slam.yaml`. Do not commit large bag files; reference their source and playback commands instead.
