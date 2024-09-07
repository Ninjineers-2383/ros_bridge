Set-Variable -Name "IP" -Value (Get-NetIPAddress -AddressFamily IPV4 -InterfaceAlias Wi-Fi).IPAddress

docker run -it --rm -p 8090:8090 -p 1735:1735 -p 5810:5810 -e DISPLAY=$IP":0.0" -v ${PWD}/gradle_cache:/root/.gradle -v ${PWD}:/repos/ros_bridge ninjineers-2383/wpilib_devel /bin/bash -c "cd /repos/ros_bridge && ./gradlew simulateNative"