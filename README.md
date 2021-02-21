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

To know more about objects files check [Graspit manual](https://graspit-simulator.github.io/build/html/data_files_bodies.html)

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

**4. Search for the desired grasp using grapical interface**
1.  Load yor robot and object
    -Option 1: Load robot (File>import Robot) and object (File> Import object). Note: recomen to move the robot a little, to avoid object importing in collision with the object.
    -Option 2: Load a world file with the object and robot included (File> Open)
2.A. (Option 1) Use EigenGrasp planner (Grasp> EigenGrasp Planner) 
    - Select desired settings, then init and ">" [Check manual for more details](https://graspit-simulator.github.io/build/html/grasp_planning_eg.html)
2.B (Option 2) Move manualle hand to desired grasp
3. You can evaluate the grasp using Grasp>Quality Measure. Select quality, Add, Ok. For example epsilon =-1 is invalid grasp. The bigger the number the better (note epsilon<1).
4. Save the file (.world) In there you can find informaton like the joint posiiton and the orientation and posiitons of the hand and object (To use this information you will need to trasnform accordinly to the origin of Gazebo/Rviz)



## Choosing contact points for the planner:
When planning a gasrp with EigenGrasp, you need to select which contact file you want to use. 
The contact files indicate points that the planer will place in contact with the object.

Currently you have the following files (on folder *robots/mpl_right_arm/virtual/*):
-contacts_all.xml: include a contact in each finger phalanx and 3 on the palm
-Contacts_pinch: only on the finger tips of the thumb and index (note: rigid fingers can't perform force closure with 2 fingers)
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




## Getting grasping from Graspit using service written by Jennifer Buehler

The wiki of this code can be found [here](https://github.com/JenniferBuehler/graspit-pkgs/wiki/grasp_planning_graspit_ros)
This is a sumary of the steps shown in the example, with the only change of change the robot by the mpl_arm

**1. Install** [Jennifer Buehler graspit-pkg](https://github.com/JenniferBuehler/graspit-pkgs/wiki/Installation). 
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


**2.Start service node**
in a terminal run:
```
roslaunch grasp_planning_graspit_ros grasp_planning_service.launch \
    results_output_directory:=<folder-name>
```
This node launch the ncesary services. 

They can be used as in the example, with lunch files, or write a code that use this services in the same order:

**3. Load robot to the data base**
In another terminal run (use /graspit_add_to_database service)
```
roslaunch mpl_graspit load_robot2service.launch
```

**4. Load object to the data base**
For example load a cube (use /graspit_add_to_database service). type in a terminal
```
roslaunch mpl_graspit load_checker2service.launch
```

**5. Load robot and object from the database to graspit**
In a terminal (use /graspit_load_model service):
```
roslaunch grasp_planning_graspit_ros graspit_load_models_example.launch load_table:=true
```

**6. Run EigenGrasp to compute grasp on the object**
in a terminal (service /graspit_eg_planning)
```
roslaunch grasp_planning_graspit_ros graspit_eigengrasp_planner_client_example.launch robot_name:=mpl_right_arm
```

Note: this service if of type *moveit_msgs/GraspPlanning*. it returns a list with the possible grasps.