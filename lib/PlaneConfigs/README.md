# IMPORTANT

This directory is created to make switching the Xpilot flight stabilization system from one RC airplane to another easier by eliminating the use of one configuration file for all airplanes.  
Every airplane will have it's own configuration file which can be selected using macro definitions in [Plane Selector](src/PlaneSelector.h) and the individual configuration files conditionally compiled in [Plane Config](src/PlaneConfig.h). The process for creating and using these configuration files is explained below;

## Steps

- In the same directory as [Plane Selector](src/PlaneSelector.h) and [Plane Config](src/PlaneConfig.h), create a new config header file for every airplane to be controlled by Xpilot.
- Copy the contents of [config.h](src/config.h), and paste into your new airplane config header file.
- The settings in [config.h](src/config.h) are default and will probably require some modifications to work in your airplane. Be sure to read the entire file.
- Follow the instructions in both [Plane Selector](src/PlaneSelector.h) and [Plane Config](src/PlaneConfig.h)
