# Ceres

![ceres_robot](docs/images/ceres_robot.jpg?raw=true "ceres_robot")

## Maintainers
* [Julian Gaal](mailto:gjulian@uos.de)
* [Christopher Broecker](mailto:chbroecker@uos.de)
* [Sebastian Pütz](mailto:spuetz@uos.de)

## Installation

### 1. Install wstool
```
sudo apt install python-wstool
mkdir -p ~/ceres_ws
```

### 2. Use wstool to pull all required packages
```
wstool init src https://raw.githubusercontent.com/uos/uos_rosinstalls/master/ceres-kinetic.rosinstall
wstool update -t src
```

### 3. Pico Flexx Royale Libary
* download the royale SDK `libroyale.zip` from the [manufacturers website](http://pmdtec.com/picofamily/software/) with the costumer password provided in the pico flexx casing
* Extract the linux 64 bit archive from the extracted SDK to `ceres_ws/src/pico_flexx_driver/royale`

### 4. Update UDEV Rules
Install the udev rules for the pico_flexx
```
cd ~/ceres_ws/src/pico_flexx_driver/royale
sudo cp libroyale-<version_number>-LINUX-64Bit/driver/udev/10-royale-ubuntu.rules /etc/udev/rules.d/
```

Install the udev rules for the sick tim
```
sudo cp ~/ceres_ws/src/sick_tim/udev/81-sick-tim3xx.rules /etc/udev/rules.d
```
Install the udev rules for the phidgets driver
```
sudo cp ~/ceres_ws/src/phidgets_drivers/phidgets_api/share/udev/99-phidgets.rules /etc/udev/rules.d
```

## Ceres_Util
We provide a script for an easy remote connection setup in `ceres_util/ceres.rc`.
*Make sure it is sourced in `~/.bashrc`*

The script contains the following functions:
* `ceres-host`
  * exports the ROS_MASTER_URI
* `ceres-client [robot_name]`
  * sets your ROS to be a client on computers that have an RZ ID
* `ceres-client-ip [robot_name]`
  * sets your ROS to be a client on computers that do not have an RZ ID
* `ceres-ssh [robot_name]`
  * ssh's into the given robot
* `ssh-[robot_name]`
  * ssh's into the given robot
* `cmux`
  * starts a basic tmux session for ceres

## Remote Connection
* Example Host:
  * flamara
* Example Client:
  * your personal laptop


1. `ceres-host` on the host machine (in this case on flamara)
2. start the launch file `roslaunch ceres_bringup ceres.launch`
3. `ceres-client-ip flamara` on your personal laptop
4. start rviz on the client
