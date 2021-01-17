# Multi-sliding-Contacts

This repository contains the C++ code for the controller. It is provided as an open-source code for the researchers who are interested in the Multi-contatc scenarios.
Here, I give a tutorial on how one can start simulations properly. Please do no hesitate to contact me or share your opinion in case of any issues or difficulties in running the controller.

## Dependencies

The controller is tested in Ubuntu 18.04 and 20.04. Before running the controller, you need to install the following software:

- [ROS](https://www.ros.org/): robotics middleware

- [Eigen](https://eigen.tuxfamily.org/index.php?title=Main_Page): linear algebra

- [Eigen3ToPython](https://github.com/jrl-umi3218/Eigen3ToPython): Python bindings for Eigen

- [SpaceVecAlg](https://github.com/jrl-umi3218/SpaceVecAlg): spatial vector algebra

- [RBDyn](https://github.com/jrl-umi3218/RBDyn/): rigid body dynamics

- [eigen-qld](https://github.com/jrl-umi3218/eigen-qld): quadratic programming

- [eigen-quadprog](https://github.com/jrl-umi3218/eigen-quadprog): quadratic programming

- [sch-core](https://github.com/jrl-umi3218/sch-core): collision detection

- [sch-core-python](https://github.com/jrl-umi3218/sch-core-python): Python bindings for sch-core

- [Tasks](https://github.com/jrl-umi3218/Tasks/): inverse kinematics

- [mc_rbdyn_urdf](https://github.com/jrl-umi3218/mc_rbdyn_urdf): robot model loader

- [copra](https://github.com/jrl-umi3218/copra): linear model predictive control

- [mc_rtc](https://github.com/jrl-umi3218/mc_rtc): robot controller library

These are same dependencies for running other interesting controllers which are available as open-source in IDH team. For instance, you can check the [LIPM Walking Controller](https://github.com/stephane-caron/lipm_walking_controller/).
## Simulation

For simulation, we are using [Choreonoid](https://choreonoid.org/en/) software. 

## Installing the Controller

In order to install the controller, you need to create `build` folder and install the controller through `cmake`:

```sh
mkdir build && cd build
cmake ..
make 
sudo make install
```
# Running the Controller

In separate consoles, you need to run the following commands. For Running the `Choreonoid`:

```sh
choreonoidhrp4 ~/address-to-cnoid-file
```
You can use `~/wiping_cheb_admittance/cnoid/slope_Saeid_20deg.cnoid` for the cnoid file. Then, for running RVIZ:

```sh
roslaunch mc_rtc_ticker control_display.launch robot:=HRP4Comanoid
```
and finally, run the controller through:

```sh
MCUDPControl -h localhost -f ~/address-to-mc_rtc.conf-file
```
Enjoy!
