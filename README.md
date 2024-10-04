# skidSteeringLocalization

This repo contains the src folder of a ROS2 workspace. The packages inside the src folder aim to localize a Scout Mini skid steering robot using the *robot_localization* package.

## scout_mini_localization.launch.py
Launch file that spawns a Scout Mini in an empty world and localize it with and EKF filter of the *robot_localization* package. 
The namespace of the robot has been set to null , meaning ''. 

## scoutMini.yaml
yaml file used by the *robot_localization* package, carrying initial parameters. 
Having only one *odom* sensor and one *imu* sensor it is not possible to measure a displacement in Z, therefore the flag *two_d_mode* is true, which obscure any Z movement to the EKF filter.

## scout_mini.sdf
Removed the lidar and GPS plugins for and easier starting implementation. The tf between odom and base_link is published.