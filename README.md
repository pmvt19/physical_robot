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

    ├──shell_scripts/
        ├──proto_gen.sh
        ├──start_lidar_process.sh

    ├──debugging/
        ├──debug.py
        ├──debug_lidar_angles.py

    ├──monitoring/
        ├──viz_camera.py
        ├──viz_robot_lidar.py
        ├──viz_imu.py

    ├──utils/
        ├──utils.py
        ├──test_utils.py

    ├──hardware/
        ├──robot_interface.py (robot_motor_interface.py)
        ├──dxl_controller.py

    ├──sensors/
        ├──physical/
            ├──run_camera.py
            ├──run_lidar.py
        ├──simulated/
            ├──simualte_lidar.py
    
    ├──processes/
        ├──run_interactive_robot.py (run_slam.py)
        ├──run_localization_robot.py
        ├──run_robot.py
        ├──run_localize_and_plan.py

    ├──robot/
        ├──robot.py
        ├──robot_space.py
        ├──robot_server.py

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


Remaining files:
- icp.py
- particle_filter.py

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