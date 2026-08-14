```
physical_robot/
├── physical_robot/
    ├──generated/
        ├──robot_data_pb2_grpc.py...

    ├──saves/
        ├──scenes/
            ├──tmp/
                ├──map/
                ├──map_imgs/
                ├──planner_data/
    ├──maps/
        ├──basic_map.py
        ├──map.py
        ├──advanced_map.py
        ├──abstract_map.py
        ├──semantic_map.py

    ├──shell_scripts/
        ├──proto_gen.sh
        ├──start_lidar_process.sh
        ├──start_camera_process.sh
        ├──start_sensor_suite.sh

    ├──debugging/
        ├──debug.py
        ├──debug_lidar_angles.py

    ├──monitoring/
        ├──viz_camera.py
        ├──viz_robot_lidar.py
        ├──viz_imu.py
        ├──viz_point_cloud.py

    ├──utils/
        ├──utils.py
        ├──test_utils.py

    ├──hardware/
        ├──robot_interface.py (robot_motor_interface.py)
        ├──dxl_controller.py

    ├──sensors/
        ├──physical/
            ├──run_camera.py
            ├──run_camera_elite.py
            ├──run_lidar.py
        ├──simulated/
            ├──simualte_lidar.py
    
    ├──processes/
        ├──run_interactive_robot.py (run_slam.py)
        ├──run_localization_robot.py
        ├──run_robot.py
        ├──run_localize_and_plan.py
        ├──run_nlp_mp_robot.py (TODO)

    ├──machine_learning/ (TODO)
        ├──image_processing/
            ├──image_segmentator.py
        ├──language_processing/
            ├──language_processor.py

    ├──robot/
        ├──robot.py
        ├──robot_space.py
        ├──robot_server.py
        ├──robot_controller.py

    ├──algorithms/
        ├──icp.py
        ├──particle_filter.py

    ├──config/
        ├──config.py

    ├──protos/
        ├──robot_data.proto

    ├──tests/
        ├──test_sensors/
            ├──test_lidar.py
            ├──test_camera.py
            ├──test_imu.py
        ├──test_processes/
            ├──test_particle_filter.py
            ├──test_slam.py??

    ├──tests/
        ├──sensors/
            ├──test_lidar.py
            ├──test_camera.py
            ├──test_imu.py
        ├──processes/
            ├──test_particle_filter.py
            ├──test_slam.py??
            ├──test_vlm_client.py
        ├──visual/
            ├──tbd.py??
        ├──computational/
            ├──test_robot_controller.py
            ├──test_geometry_utils.py
            ├──etc...

    # Potential Future Planning
    ├──mcp/
        ├──server.py
        ├──tools.py
    
    ├──main.py??


Remaining files:
- icp.py
- particle_filter.py

Planned files:
- image_segmentator.py
- language_processor.py
- run_nlp_mp_robot.py
- start_sensor_suite.sh
- start_camera_process.sh
- semantic_map.py
- run_slam_semantic_mapping.py

Files to Rename:
- run_interactive_robot.py -> run_slam.py
- run_localization_robot.py -> run_global_localization.py
- run_localize_and_plan.py -> run_motion_planning.py??

- run_camera.py -> run_oakd_lite.py or run_oakd_camera.py

│   ├── components/
│   │   ├── Button.js
│   │   └── Header.js
│   ├── pages/
│   │   ├── HomePage.js
│   │   └── AboutPage.js
│   └── App.js
├── public/
│   ├── index.html
│   └── favicon.ico
├── package.json
└── README.md
```


## Installation

```sh
conda create --name <name> python=3.10.18

sh env_setup.sh
```
