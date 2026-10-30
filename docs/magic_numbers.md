# Magic-number and constant inventory

This is an inventory of likely tunable values in the `physical_robot/` Python
package. It focuses on physical dimensions, algorithm thresholds, operating
parameters, and values repeated across modules. Array indices, matrix dimensions,
mathematical identities, generated protobuf code, and test/demo fixtures are
generally excluded unless they expose a production default.

Units are noted where they are apparent from the code. Confirm the units and
intended behavior before centralizing or changing any value.

## Highest-priority configuration candidates

### Robot dimensions and motion limits

| Value(s) | Location | Why it may need configuration |
|---|---|---|
| Wheel diameter `66.5` (radius is `66.5 / 2`); wheelbase `210`; robot radius derived as `wheelbase / 2` | [robot.py](../physical_robot/robot/robot.py#L41), [robot_motor_interface.py](../physical_robot/hardware/motors/robot_motor_interface.py#L56) | Physical calibration values. Wheel radius and wheelbase are duplicated in the robot and motor interface. |
| Active-distance threshold `300`; local-planner distance threshold `400` | [robot.py](../physical_robot/robot/robot.py#L53) | Robot obstacle/safety behavior thresholds. |
| Linear velocity `10`; profile velocity default `100`; encoder pulses/revolution `4096` | [robot_motor_interface.py](../physical_robot/hardware/motors/robot_motor_interface.py#L47), [robot_motor_interface.py](../physical_robot/hardware/motors/robot_motor_interface.py#L59), [robot_motor_interface.py](../physical_robot/hardware/motors/robot_motor_interface.py#L80) | Hardware operating and calibration settings. |
| Simulated lidar resolution `360` and max range `12000` | [robot.py](../physical_robot/robot/robot.py#L61) | Simulation sensor parameters; max-range units should be confirmed. |
| Lidar scan filtering threshold `90` and auto-retry threshold `400` | [robot.py](../physical_robot/robot/robot.py#L140), [robot.py](../physical_robot/robot/robot.py#L161) | Sensor-data filtering and retry behavior. |
| Robot rendering dimensions: radius `100`, wheel width `50`, wheel height `20`, heading line length `500` | [robot.py](../physical_robot/robot/robot.py#L742) | Visualization geometry, currently separate from the robot's physical dimensions. |
| Circumference computation uses `66.5` directly | [robot.py](../physical_robot/robot/robot.py#L678), [motion_computation.py](../physical_robot/debugging/motion_computation.py#L18) | Duplicated wheel-diameter calibration; consider deriving from the same wheel setting. |

### Map dimensions, resolution, and inflation

| Value(s) | Location | Why it may need configuration |
|---|---|---|
| Resolution default `10.0`; initial map extent `20000 × 20000` | [map.py](../physical_robot/maps/map.py#L52), [map.py](../physical_robot/maps/map.py#L58) | Map memory use and spatial precision. The extent is explicitly marked as needing computation. |
| Occupancy threshold `0.5` | [map.py](../physical_robot/maps/map.py#L169) | Used to select occupied map points. Similar `0.5` thresholds are used in advanced-map and planning logic. |
| Obstacle inflation radius `210` | [map.py](../physical_robot/maps/map.py#L178), [advanced_map.py](../physical_robot/maps/advanced_map.py#L127), [semantic_map.py](../physical_robot/maps/semantic_map.py#L75) | Robot-clearance behavior. The default is repeated across map classes. |
| Frontier exploration overrides inflation radius to `250` | [run_frontier_exploration.py](../physical_robot/processes/run_frontier_exploration.py#L70) | Different behavior from the map default; verify whether this difference is intentional. |
| Map-expansion buffer `0.3` | [map.py](../physical_robot/maps/map.py#L232) | Controls how much extra map area is allocated when expanding. |
| Resolution defaults: `10.0` in map builders; adjustment defaults `100`/`100.0` | [map_builder.py](../physical_robot/map_builder/map_builder.py#L9), [advanced_map_builder.py](../physical_robot/map_builder/advanced_map_builder.py#L6), [semantic_map_builder.py](../physical_robot/map_builder/semantic_map_builder.py#L12), [map.py](../physical_robot/maps/map.py#L260), [advanced_map.py](../physical_robot/maps/advanced_map.py#L278) | Defaults for map construction and downsampling. |

### Localization and mapping algorithms

| Value(s) | Location | Why it may need configuration |
|---|---|---|
| Particle-filter scale factor `50`; map-range buffer `1.0` | [particle_filter.py](../physical_robot/algorithms/particle_filter.py#L15), [particle_filter.py](../physical_robot/algorithms/particle_filter.py#L19) | Localization uncertainty and initial search region. |
| Fallback particle bounds `[-2000, -2000]` to `[3500, 3500]` | [particle_filter.py](../physical_robot/algorithms/particle_filter.py#L53) | Explicitly marked in code as a candidate for configuration. |
| Particle-filter noise standard deviations `[10, 10, 0.5]` and `[10, 10, 0.1]` | [particle_filter.py](../physical_robot/algorithms/particle_filter.py#L96), [particle_filter.py](../physical_robot/algorithms/particle_filter.py#L107) | Localization and motion-model tuning; position and angle components have different units. |
| Resample count default `1000`; common initialization count `10000` | [particle_filter.py](../physical_robot/algorithms/particle_filter.py#L181), [robot_utils.py](../physical_robot/robot/robot_utils.py#L19), [run_localization_robot.py](../physical_robot/processes/run_localization_robot.py#L24) | Accuracy and runtime/memory tradeoff. |
| ICP initial outlier cutoff `200`; inlier cutoff `100`; convergence ratio `0.999`; iteration count `15` | [icp.py](../physical_robot/algorithms/icp.py#L69), [icp.py](../physical_robot/algorithms/icp.py#L75), [icp.py](../physical_robot/algorithms/icp.py#L88), [icp.py](../physical_robot/algorithms/icp.py#L161) | Scan-matching acceptance and convergence behavior. Several are repeated as function defaults or loop limits. |
| Advanced-map frontier smoothing sigma `1.0`; frontier-cluster size threshold `100`; minimum obstacle distance `400 mm` | [advanced_map.py](../physical_robot/maps/advanced_map.py#L153), [advanced_map.py](../physical_robot/maps/advanced_map.py#L184), [advanced_map.py](../physical_robot/maps/advanced_map.py#L266) | Frontier detection and filtering behavior. The `100` threshold at [advanced_map.py](../physical_robot/maps/advanced_map.py#L176) appears unused. |

### Planning and navigation

| Value(s) | Location | Why it may need configuration |
|---|---|---|
| Path lookahead `4`; close-to-goal threshold `100`; minimum motion threshold `0.09` | [motion_planner.py](../physical_robot/motion_planner.py#L20), [motion_planner.py](../physical_robot/motion_planner.py#L27), [motion_planner.py](../physical_robot/motion_planner.py#L66) | Replanning cadence, goal tolerance, and skipped-motion behavior. The close-to-goal threshold `100` is also in [robot_utils.py](../physical_robot/robot/robot_utils.py#L14). |
| Edge-validity distance `200.0`; collision-check batch size `1000` | [robot_space.py](../physical_robot/robot/robot_space.py#L112), [robot_space.py](../physical_robot/robot/robot_space.py#L148) | Planner edge checking and computation/memory tradeoff. |
| Start-clear radius `100` | [task_planner.py](../physical_robot/task_planner/task_planner.py#L105), [run_frontier_exploration.py](../physical_robot/processes/run_frontier_exploration.py#L75) | Clears nearby map cells during frontier selection. |
| Frontier path horizon `40`; minimum motion threshold `0.09` | [run_frontier_exploration.py](../physical_robot/processes/run_frontier_exploration.py#L132), [run_frontier_exploration.py](../physical_robot/processes/run_frontier_exploration.py#L139) | Exploration progress and motion execution. The motion threshold duplicates the motion-planner default. |
| RRT step size `500.0`; PRM sample count `10000` and neighbor count `10` | [run_navigation.py](../physical_robot/processes/run_navigation.py#L23), [run_localize_and_plan.py](../physical_robot/processes/run_localize_and_plan.py#L28), [run_semantic_motion_planning.py](../physical_robot/processes/run_semantic_motion_planning.py#L122) | Planner performance and path quality. |
| Free-space probability threshold `0.5` | [run_semantic_motion_planning.py](../physical_robot/processes/run_semantic_motion_planning.py#L22) | Semantic planner's free/occupied classification. Compare with map thresholds before consolidating. |

## Hardware, service, and model settings

| Value(s) | Location | Why it may need configuration |
|---|---|---|
| Lidar baud rate `460800`; startup delay `5 s` | [run_lidar.py](../physical_robot/hardware/sensors/physical/run_lidar.py#L24) | Device startup/communication settings. |
| Camera depth percentiles `1` and `99`; IMU rates `480`/`400`; max batch reports `10`; camera sizes `640 × 480` or `640 × 400` | [run_camera.py](../physical_robot/hardware/sensors/physical/run_camera.py#L12), [run_camera.py](../physical_robot/hardware/sensors/physical/run_camera.py#L30), [run_camera.py](../physical_robot/hardware/sensors/physical/run_camera.py#L40), [run_camera_elite.py](../physical_robot/hardware/sensors/physical/run_camera_elite.py#L21), [run_camera_elite.py](../physical_robot/hardware/sensors/physical/run_camera_elite.py#L42) | Sensor rates, output dimensions, and depth visualization behavior. Settings are repeated in both camera launchers. |
| Redis host/port (`localhost:6379`) | [config.py](../physical_robot/config.py), [run_camera.py](../physical_robot/hardware/sensors/physical/run_camera.py#L18), [run_camera_elite.py](../physical_robot/hardware/sensors/physical/run_camera_elite.py#L8), [run_lidar.py](../physical_robot/hardware/sensors/physical/run_lidar.py#L21) | Already partly present in config, but the sensor launchers hard-code their own connection values. |
| Dynamixel protocol version `2.0`, baud rate `57600`, and RPM conversion factor `0.229` | [dxl_controller.py](../physical_robot/hardware/motors/dxl_controller.py#L33), [dxl_controller.py](../physical_robot/hardware/motors/dxl_controller.py#L36), [dxl_controller.py](../physical_robot/hardware/motors/dxl_controller.py#L134) | Hardware/protocol-specific constants; configurable if the motor/controller model can vary. |
| Segmentation minimum component size `1%` of image | [image_segmentation.py](../physical_robot/models/segmentation/image_segmentation.py#L50) | Image filtering behavior. |
| VLM temperature `0.5` | [vlm_client.py](../physical_robot/models/vlm/vlm_client.py#L71) | Model-generation behavior; repeated across four calls. |
| UI JPEG quality `70`, depth scale `0.03`, update interval `0.1 s`, server port `9090` | [ui_server.py](../physical_robot/ui/ui_server.py#L23), [ui_server.py](../physical_robot/ui/ui_server.py#L59), [ui_server.py](../physical_robot/ui/ui_server.py#L100), [ui_server.py](../physical_robot/ui/ui_server.py#L108) | UI throughput, display, and deployment settings. |

## Other operational and visualization constants

These are less likely to need robot-wide configuration, but may be worth making
local options if their workflows are used regularly:

- Monitoring runtime `300 s` in [viz_imu.py](../physical_robot/monitoring/viz_imu.py#L9), [viz_point_cloud.py](../physical_robot/monitoring/viz_point_cloud.py#L9), and [viz_robot_lidar.py](../physical_robot/monitoring/viz_robot_lidar.py#L9).
- Robot service timestamp multiplier `100000`, worker count `10`, and sleep duration `86400 s` in [robot_server.py](../physical_robot/robot/robot_server.py#L32), [robot_server.py](../physical_robot/robot/robot_server.py#L212), and [robot_server.py](../physical_robot/robot/robot_server.py#L221).
- ICP visualization plot padding `1000` and animation pause `0.1 s` in [icp.py](../physical_robot/algorithms/icp.py). These affect visualization rather than the algorithm's intended output.
- UI demo telemetry values (battery, CPU/RAM, Wi-Fi) are fixed in [ui_server.py](../physical_robot/ui/ui_server.py#L76); these appear to be sample data rather than robot settings.
- Hard-coded poses, targets, and test trajectories in process scripts such as [run_robot.py](../physical_robot/processes/run_robot.py) and [run_localize_and_plan.py](../physical_robot/processes/run_localize_and_plan.py) appear scenario-specific.

## Existing configuration and suggested next steps

The package already has a small config dictionary in [config.py](../physical_robot/config.py)
for connection type, Redis, serial ports, and the client address. A practical
next step would be to centralize the repeated physical calibration values,
Redis settings, map defaults, and navigation thresholds first. Keep
algorithm-specific parameters grouped separately from hardware calibration,
and preserve function-level overrides where callers need per-run tuning.

This inventory identifies candidates; it does not assert that every listed
value should be user-configurable. Protocol constants, fixed schema values,
visualization-only settings, and example parameters may be better left as
named constants or local defaults.
