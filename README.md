kvn_robot
==========================================

$DESCRIPTION$

![Licence](https://img.shields.io/badge/License-$LICENSE$-blue.svg)

## Build status



ROS 2 Distro | Branch | Build status | Documentation | Released packages
:---------: | :----: | :----------: | :-----------: | :---------------:
**Jazzy** | [`jazzy`](https://github.com/kvn_robot/kvn_robot/tree/jazzy) | [![Jazzy Binary Build](https://github.com/kvn_robot/kvn_robot/actions/workflows/jazzy-binary-build-main.yml/badge.svg?branch=main)](https://github.com/kvn_robot/kvn_robot/actions/workflows/jazzy-binary-build-main.yml?branch=main) <br /> [![Jazzy Semi-Binary Build](https://github.com/kvn_robot/kvn_robot/actions/workflows/jazzy-semi-binary-build-main.yml/badge.svg?branch=main)](https://github.com/kvn_robot/kvn_robot/actions/workflows/jazzy-semi-binary-build-main.yml?branch=main) | [![Doxygen Doc Deployment](https://github.com/kvn_robot/kvn_robot/actions/workflows/doxygen-deploy.yml/badge.svg)](https://github.com/kvn_robot/kvn_robot/actions/workflows/doxygen-deploy.yml) <br /> [Generated Doc](https://kvn_robot.github.io/kvn_robot_Documentation/jazzy/html/index.html) | [kvn_robot](https://index.ros.org/p/kvn_robot/#jazzy)

### Explanation of different build types

**NOTE**: There are three build stages checking current and future compatibility of the package.

[Detailed build status](.github/workflows/README.md)

1. Binary builds - against released packages (main and testing) in ROS distributions. Shows that direct local build is possible.

   Uses repos file: `$NAME$-not-released.<ros-distro>.repos`

1. Semi-binary builds - against released core ROS packages (main and testing), but the immediate dependencies are pulled from source.
   Shows that local build with dependencies is possible and if fails there we can expect that after the next package sync we will not be able to build.

   Uses repos file: `$NAME$.repos`

1. Source build - also core ROS packages are build from source. It shows potential issues in the mid future.
