# Repository Guidelines

Contributor guide for `form_nav`, a ROS (catkin) package for multi-UAV formation navigation: global path planning, local MPC control with ESDF/CasADi obstacle avoidance, and formation tracking.

## Project Structure & Module Organization

- `src/` — node entry points and implementations (`mpc_node.cpp`, `kino_astar_node.cpp`, `dwa_node.cpp`)
- `include/form_nav/` — headers sharing the base name of their source files
- `msg/` — custom ROS message definitions (e.g. `PredictedTrajectory.msg`)
- `launch/` — `roslaunch` files for the full stack and individual nodes
- `config/` — YAML parameters, e.g. `formations.yaml` for formation offsets and `single_uav.yaml`
- `worlds/` and `rviz/` — Gazebo worlds and RViz view configurations
- `docs/` — reference papers and experiment output
- `old/` — deprecated code; do not extend it

## Build, Test, and Development Commands

Run these from the catkin workspace root (`~/catkin_ws`):

- `catkin_make` — builds the package; C++17 is set in `CMakeLists.txt`
- `source devel/setup.bash` — load the built workspace into the shell
- `roslaunch form_nav formation_planning.launch` — start the formation planning stack
- `roslaunch form_nav mpc_esdf.launch` — start MPC + ESDF obstacle avoidance
- `roslaunch form_nav multi_uav_gazebo.launch` — start the multi-UAV Gazebo simulation

## Coding Style & Naming Conventions

- C++17, 4-space indentation, no tabs
- `snake_case` for files, functions, and variables; member variables often carry a trailing underscore (`is_tracked_`)
- Node entry points are named `<node>_node.cpp`
- Comments are typically written in Chinese — match the language of the code you are editing
- Include guards are uppercase file names (`MPC_H`), and headers live in `include/form_nav/`
- No formatter or linter is configured; match the existing style

## Testing Guidelines

No automated test framework is configured yet — there is no `test/` directory. Validate changes by building with `catkin_make` and running the relevant launch file in Gazebo/RViz. If you add unit tests, use `gtest`: declare `<test_depend>gtest</test_depend>` in `package.xml` and register targets with `catkin_add_gtest`.

## Commit & Pull Request Guidelines

The Git history uses concise, descriptive one-line messages, in Chinese or English, stating what changed. Longer change series are prefixed with a consistent tag (e.g. `编队扩展修改2-...`, `低速适配修改1-...`). Work on a feature branch (current mainline is `develop-mpc`), keep commits small and focused, and merge through pull requests. Each PR should summarize the change, reference related issues, and note how it was validated (which launch file or scenario was run).
