#!/bin/bash

docker run -it --rm -p 8090:8090 -p 8091:8091 -p 1735:1735 -p 5810:5810 -v $(pwd)/gradle_cache:/root/.gradle -v $(pwd):/repos/ros_bridge -e DISPLAY=192.168.1.23:0.0 ninjineers-2383/wpilib_devel /bin/bash -c "cd /repos/ros_bridge && ./gradlew simulateNative"