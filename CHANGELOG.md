## Unreleased

+ Renamed `serial_backseat_interface.launch` -> `serial_driver.launch`
+ Renamed `slocum_glider_backseat` -> `slocum_glider_extctl_serial`
+ All glider sensors declared as bools are transmitted in ROS messages and
  services as `Float32`s.
+ Added an extctl driver for completely simulated gliders.

## v0.0.1 - September 15, 2020

First release
