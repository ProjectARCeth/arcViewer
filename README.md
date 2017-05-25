# arcViewer

## Controller - Dictonary
B - Emergency button (notstop)

Y - Shutdown button (slow system shutdown)

Right pin - Accelerating and breaking (v_should, controlled by VCU)

Left pin - Steering angle

## GMAPS - INSTALLATION AND USAGE:
### first get rosbridge:
sudo apt-get install ros-groovy-rosauth

git clone https://github.com/RobotWebTools/rosbridge_suite.git

catkin build

MORE: http://wiki.ros.org/rosbridge_suite

### then get roslibjs
follow: https://github.com/RobotWebTools/roslibjs/blob/develop/CONTRIBUTING.md

if grunt build not working -> use "ln -s /usr/bin/nodejs /usr/bin/node" inside the roslibjs folder!!!

if still error: change version of bson (via: pip uninstall bson and do:)

sudo apt-get update

sudo apt-get install python-bson

### launching & usage:
start rosbridge websocket on the master Pc with: roslaunch rosbridge_server rosbridge_websocket.launch

ENSURE THAT PORT FORWARDING IS ENABLED ON THE PORT WHERE ROSBRIDGE OPENS!!!

(Checking whether port is open can be done via http://www.portchecktool.com/)

then you can connect to this IP and Port by setting those params in the script!!!
