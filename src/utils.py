#!/usr/bin/env python2


"""
function that extract information from world file (grasp processed offline):
-joints position (and names?)
-hand position
-object posiiton
"""
import os
import rospkg 
from xml.dom import minidom

import tf2_ros
import rospy


def read_world_file(filename,package_name, package_folder=None):
    """
    Function that read a world file to 
    inputs: 
    ----filename: example 'worlds/mpl_checker_v2.xml'
    ----package_folder: if is in a folder in a ROS package, the name of the folder
    ----package_name: ROS package where the file is located
    return: robot_joints_position, robot_orientation, robot_position, object_orientation, object_posiiton.
    
    """
    #auxiliary functions
    def get_filename(xml_file,parent):
        """
        return filename 
        parent: robot or graspableBody
        """
        parent=xml_file.getElementsByTagName(parent)[0]
        name=str(parent.getElementsByTagName("filename")[0].childNodes[0].nodeValue)
        return name

    def get_transformation(xml_file,parent):
        """
        return orientation as quaternion and position
        """
        parent=xml_file.getElementsByTagName(parent)[0]
        fullTransform=parent.getElementsByTagName("fullTransform")[0].childNodes[0].nodeValue #string
        orientation=map(float,fullTransform.split("(")[1].split(")")[0].split())#quaternion
        translation=map(float,fullTransform.split("[")[1].split("]")[0].split())
        return orientation , translation


    def get_joints_names(robot_path):
        """
        need absolute path to robot filename
        """
        robot_xml = minidom.parse(robot_path)
        links=robot_xml.getElementsByTagName('link')
        links_names=[]
        for link in links:
            name=link.childNodes[0].nodeValue.replace('.xml','').encode("ascii", "ignore")
            links_names.append(name)
        return links_names

    def get_joints_position(xml_file,robot_path):
        """
        get joints position
        """
        robot=xml_file.getElementsByTagName('robot')[0]
        robot_joints_positions=map(float,robot.getElementsByTagName("dofValues")[0].childNodes[0].nodeValue.split())
        robot_joints_names=get_joints_names(robot_path)
        joints={}
        #create a disctionary
        for i in range(len(robot_joints_names)):
            joints[robot_joints_names[i]]=robot_joints_positions[i]
        return joints

    def get_absolute_path(package_name,package_folder,filename):
        """
        get absolute path of a file
        knowing the ROS package, and subfolder inside the ROS package
        """
        if package_folder is not None:
            path=os.path.join(rospkg.RosPack().get_path(package_name),package_folder, filename)
        else:
            path=os.path.join(rospkg.RosPack().get_path(package_name), filename)
        return path


    #get file absolute path
    world_path=get_absolute_path(package_name,package_folder,filename)
    world_xml = minidom.parse(world_path)

    #robot information
    robot_filename=get_filename(world_xml,'robot')
    robot_path=get_absolute_path(package_name,package_folder,robot_filename)
    robot_joints= get_joints_position(world_xml, robot_path)
    robot_orientation , robot_translation= get_transformation(world_xml,'robot')

    #object information
    object_filename=get_filename(world_xml,'graspableBody')
    obj_orientation , obj_translation= get_transformation(world_xml,'graspableBody')

    return robot_joints, robot_orientation , robot_translation,  obj_orientation , obj_translation



# def get_transformation(tf2_buffer, target_frame, source_frame):
#     return tf2_buffer.lookup_transform(target_frame, source_frame, rospy.Time(), rospy.Duration(10))


#main for testing
if __name__ == "__main__":
    filename='worlds/mpl_checker_v2.xml'
    package_folder='resources'
    package_name='mpl_graspit'

    robot_joints, robot_orientation , robot_translation,  obj_orientation , obj_translation= read_world_file(filename,package_name, package_folder)
    print(robot_joints)
    print(robot_orientation)
    print(robot_translation)
    print(obj_orientation)
    print(obj_translation)



    # """
    # get robot hand position and orientation (in gazebo world frame) based on real object position, and the position/orientations given in Graspit.
    # """
    # #nodes to handle transformaitons// they will need to be running?
    # rospy.init_node("transformations")
    # tf2_buffer = tf2_ros.Buffer()
    # tf2_listener = tf2_ros.TransformListener(tf2_buffer)
    # #note distances in graspit are in mm, in Gazebo are meters.





