# skidSteeringLocalization

This repo contains the src folder of a ROS2 workspace which functions as backup storage for the files used for a MSc thesis named "Localization and mapping in agricutltural applications". The aim of the thesis is to study whether a differential motion model could be implemented in the the *robot_localization* package, and what would be the advantages and drawbacks of this method. As a testing robot a Scout Mini by Agilex has been used but due to excessively unreliable simulations, even in perfect conditions, Turtlebot3, a much simpler differential robot, has also been used in several phases of the simulation part.

## Scout Mini
### Localization with Odometry and IMU
* **Launch file**: `scout_mini_localization.launch.py` Launch file that spawns a Scout Mini in an empty world and localize it with and EKF filter of the *robot_localization* package. 
The namespace of the robot has been set to null , meaning ''. \
* **Model**: `scout_mini.sdf` Removed the lidar and GPS plugins for and easier starting implementation. The tf between odom and base_link is published.\
* **Parameter file**: `scoutMini.yaml` yaml file used by the *robot_localization* package, carrying initial parameters. 
Having only one *odom* sensor and one *imu* sensor it is not possible to measure a displacement in Z, therefore the flag *two_d_mode* is true, which obscure any Z movement to the EKF filter. \
* **Coordinates**: via Candiani 125, Milano, Italy. Lat: 45.503154; Lon: 9.162539; Elev: 136.0; Magnetic declination [rad]: 0.06318092 \
* **World**: `empty.world`

### Localization with Odometry, IMU and GPS. Dual Method
* **Launch file**: `dual_scout_mini_localization.launch.py` Launch file that spawns a Scout Mini in an empty world and localize it with and EKF filter of the *robot_localization* package. 
The namespace of the robot has been set to null , meaning ''. \
* **Model**: `scout_mini.sdf` Removed the lidar and GPS plugins for and easier starting implementation. The tf between odom and base_link is published.\
* **Parameter file**: `dual_ekf_navsat_example_turtlebot.yaml` yaml file used by the *robot_localization* package, carrying initial parameters. 
Having only one *odom* sensor and one *imu* sensor it is not possible to measure a displacement in Z, therefore the flag *two_d_mode* is true, which obscure any Z movement to the EKF filter. \
* **Coordinates**: via Candiani 125, Milano, Italy. Lat: 45.503154; Lon: 9.162539; Elev: 136.0; Magnetic declination [rad]: 0.06318092 \
* **World**: `empty.world`


## Turtlebot 3
### Localization with Odometry and IMU
* **Launch file**: `turtlebot_localization.launch.py` \
* **Model**: `model.sdf` \
* **Parameter file**: `turtlebot.yaml` \
* **World**: `empty_world.world`

### Localization with Odometry, IMU and GPS. Dual Method
* **Launch file**: `dual_turtlebot_localization.launch.py` \
* **Model**: `model.sdf` \
* **Parameter file**: `dual_ekf_navsat_example_turtlebot.yaml` \
* **Coordinates**: via Candiani 125, Milano, Italy. Lat: 45.503154; Lon: 9.162539; Elev: 136.0; Magnetic declination [rad]: 0.06318092 \
* **World**: `empty_world_gps.world`