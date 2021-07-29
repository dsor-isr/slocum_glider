## Unreleased

## v0.2.1 - July 29, 2021

+ Fix sending Byte sensor messages from the serial driver.

## v0.2.0 - July 27, 2021

+ Goto list improvements
+ Report thruster to simulator
+ Better surface behavior with `END_ACTION_QUIT`
+ Added a bsd status mission
+ python3 support for extctl_serial

## v0.1.1 - May 11, 2021

+ Separated Kaniko caches for images. There seemed to be some cross talk.

## v0.1.0 - May 11, 2021

+ Renamed `serial_backseat_interface.launch` -> `serial_driver.launch`
+ Renamed `slocum_glider_backseat` -> `slocum_glider_extctl_serial`
+ All glider sensors declared as bools are transmitted in ROS messages and
  services as `Float32`s.
+ Added an extctl driver for completely simulated gliders.
+ Added the `slocum_glider_mission_controller` package.
+ Made smaller, multiarch Docker images for easy deployment.

## v0.0.1 - September 15, 2020

First release
