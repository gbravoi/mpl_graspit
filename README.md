# MPL Graspit
It is an integration of the MPL (Modular Prosthetic Limb from Johns Hopkins) with Graspit, ROS and moveit.
The final goal is to have a service that report the Greas message needed in moveit to perform grasping of an object.


## Installation
This installation assume that you have ROS.

**1. Install graspit. In your home/user folder do:**
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


**2. Install** [Jennifer Buehler graspit-pkg](https://github.com/JenniferBuehler/graspit-pkgs/wiki/Installation). 
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


**3. Clone this repo in your catkin_workspace/src**
```
https://github.com/gbravoi/mpl_graspit.git
```

**4. catkin_make your workspace**
now you need to use catkin_make as usual
```
cd ..
catkin_make
```

## Launch graspit interface
The code that integrates Graspit-ROS-Moveit do not need this. 
But you can use this, if you want to work directly with the graspit interface

**1. First set the folder where you will save your graspit models**
**-Option 1: use folder created by graspit**
/home/user/.graspit  should be created automatically when installing graspit.
Add this folder to your bashrc  file:
```
echo "export GRASPIT=~/.graspit" >> ~/.bashrc    
```

If you do this, and place the models of the robot and the object in other folder (for example in *<path-to mpl_graspit>/resources*), you can create softlinks of the following way:

```
cd $GRASPIT/models/robots
ln -s <path-to mpl_graspit>/resources/models/robots/mpl_right_arm
cd $GRASPIT/models/objects
ln -s <path-to mpl_graspit>/resources/models/objects/<YOUR_OBJECT.xml>
ln -s <path-to mpl_graspit>/resources/models/objects/<YOUR_OBJECT.iv>
cd $GRASPIT/worlds
ln -s <path-to mpl_graspit>/resources/worlds/mpl_right_arm_world.xml
```

**-Option 2: use fordelr inside the package**
Add it to your bashrc  file:
```
echo "export GRASPIT=<path-to mpl_graspit>/resources" >> ~/.bashrc    
```


**3. Launch graspic graphical interface**
On a terminal execute:
```
graspit_simulator
```


## Choosing contact points for the planner:
When planning a gasrp with EigenGrasp, you need to select which contact file you want to use. 
The contact files indicate points that the planer will place in contact with the object.

Currently you have the following files (on folder *robots/mpl_right_arm/virtual/*):
-contacts_all.xml: include a contact in each finger phalanx and 3 on the palm
-Contacts_pinch: only on the finger tips of the thumb and index
-Contacts_tripod: only on the finger tips of the thumb, index and middle

To change the contacts file used by defauld change *model/robots/mpl_right_arm/mpl_right_arm.xlm* changing the following like
```
<virtualContacts>virtual/<YOUR_CONTACT_FILE>.xml</virtualContacts>
```


You can create your set of contacts using the *Virtual Grasp* tool of graspit.
1. On Graspit load *worlds/mpl_sphere.xml*
2. go to Grasp>Virtual Contacts.
3. Move the sphere to the point of contact, select "Mark Contact" (marked contacts should increate by one)
4. Once all contacts are done click on save

![Alt text](readme_img/create_contacts.png?raw=true "Create Hand contacts")