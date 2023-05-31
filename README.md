## Dependencies
1. Ubuntu 16.04 with [ROS Kinetic](http://wiki.ros.org/kinetic/Installation)
2. Eigen
3. Boost
4. [OMPL](https://ompl.kavrakilab.org/)

## Building
1. Create a [catkin workspace](https://catkin-tools.readthedocs.io/en/latest/quick_start.html).
2. Clone this repository:
```
git clone git@github.com:dhruvms/m4m.git
```
3. Clone other dependencies:
```
git clone --branch dev/raletters https://github.com/dhruvms/smpl.git
git clone https://github.com/dhruvms/comms.git
git clone https://github.com/aurone/leatherman.git
```
4. Build with `catkin build`!
