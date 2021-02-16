# MPL Graspit
It is an integration of the MPL (Modular Prosthetic Limb from Johns Hopkins) with Graspit, ROS and moveit.
The final goal is to have a service that report the Greas message needed in moveit to perform grasping of an object.


## Installation
This installation assume that you have ROS.

1. Install graspit. In your home/user folder do:
```
git clone https://github.com/graspit-simulator/graspit.git
cd graspit
mkdir build
cd build
cmake ..
make -j5
sudo make install
```

The previous code install Graspit. You can learn more about this software on their [manual](https://graspit-simulator.github.io/build/html/getting_started.html).
Note: to launch graspit, on a terminal execute:
```
graspit_simulator
```
We won't be using their graphical interface

2. Install [Jennifer Buehler graspit-pkg](https://github.com/JenniferBuehler/graspit-pkgs/wiki/Installation). 
First you need to have the following packages:
```
sudo apt-get install \
    libsoqt4-dev \
    libcoin80-dev \
    libqt4-dev \
    libblas-dev \
    liblapack-dev \
    libqhull-dev
```

To do so, go to your catkin_workspace/src and clone:
```
cd <path-to-your-catkin-ws>/src
git clone https://github.com/JenniferBuehler/convenience-pkgs.git
git clone https://github.com/JenniferBuehler/urdf-tools-pkgs.git
git clone https://github.com/JenniferBuehler/graspit-pkgs.git
```


3. Clone this repo in your catkin_workspace/src
```
https://github.com/gbravoi/mpl_graspit.git
```

4. catkin_make your workspace
now you need to use catkin_make as usual
```
cd ..
catkin_make
```
