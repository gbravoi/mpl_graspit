#!/usr/bin/env python2


"""
function that extract information from world file (grasp processed offline):
-joints position (and names)
-enf effector position in graspit
-object position in graspit

aditional functions
-Handle transformations (orientation and position) between diferent frames
-compute pose of the end effetcor in world frame if object pose in world frame is known.
"""
import os
import rospkg 
from xml.dom import minidom


from scipy.spatial.transform import Rotation as R
import numpy as np
from geometry_msgs.msg import Pose

import rospy
import tf2_ros
import tf2_geometry_msgs  # **Do not use geometry_msgs. Use this instead for PoseStamped

from geometry_msgs.msg import Pose
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from moveit_msgs.msg import   GripperTranslation


#import custom services
from mpl_utils.srv import checkIK 




def get_transformation_matrix(q,t):
    """
    return a transformation matrix. Transform vector from frame B to frame A
    q: quaternion  (qw,qx,qy,qz) Describing orientation of B seen from A (Or rotation matrix that transform vector from B to A)
    t: translation (x,y,z) in frame A. position of frame B
    """
    r=(R.from_quat([q[1],q[2],q[3],q[0]])).as_dcm()#rotation matriz from quaternion. note quaternion in scipy is (x,w,z,w)
    t=np.expand_dims(t,axis=1)
    bottom_row=np.zeros((1,4))
    bottom_row[0][3]=1
    T=np.hstack((r,t))
    T=np.vstack((T,bottom_row))
    return T

def get_transformation_matrix_from_pose(pose):
    """
    transformaiton matrix, where the input is a pose
    q=[w,x,y,z]
    """
    q=np.array([pose.orientation.w,pose.orientation.x,pose.orientation.y,pose.orientation.z])
    t=np.array([pose.position.x,pose.position.y,pose.position.z])
    return get_transformation_matrix(q,t)

def transformation_inverse(T):
    """
    Computes the inverse of a transformation matrix
    """
    r=T[0:3,0:3]
    t=T[0:3,3]
    r_inv=np.transpose(r)
    t_inv=-1*np.matmul(r_inv,t)
    t_inv=np.expand_dims(t_inv,axis=1)
    bottom_row=np.zeros((1,4))
    bottom_row[0][3]=1
    T_inv=np.hstack((r_inv,t_inv))
    T_inv=np.vstack((T_inv,bottom_row))
    return T_inv

def from_transformation_to_pose(T):
    """
    Extract the pose from a transformation matrix
    """
    r=R.from_dcm(T[0:3,0:3]) #scipy rotation
    q=r.as_quat()#note quaternion in scipy is (x,y,z,w)
    t=T[0:3,3]
    pose=Pose()
    pose.orientation.x=q[0]
    pose.orientation.y=q[1]
    pose.orientation.z=q[2]
    pose.orientation.w=q[3]
    pose.position.x=t[0]
    pose.position.y=t[1]
    pose.position.z=t[2]
    return pose



def transform_pose_between_frames(input_pose, from_frame, to_frame):
    """
    Transform pose from one frame to another
    """
    #init tf2 to transform between frames
    tf_buffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tf_buffer)
    pose_stamped = tf2_geometry_msgs.PoseStamped()
    pose_stamped.pose = input_pose
    pose_stamped.header.frame_id = from_frame
    # pose_stamped.header.stamp = rospy.Time.now()

    try:
        # ** It is important to wait for the listener to start listening. Hence the rospy.Duration(1)
        output_pose_stamped = tf_buffer.transform(pose_stamped, to_frame, rospy.Duration(1))
        return output_pose_stamped.pose

    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
        raise



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
        return transformation matrix
        """
        scale_factor=0.001#note graspit works in mm. gazebo in meters.
        parent=xml_file.getElementsByTagName(parent)[0]
        fullTransform=parent.getElementsByTagName("fullTransform")[0].childNodes[0].nodeValue #string
        q=map(float,fullTransform.split("(")[1].split(")")[0].split())#quaternion  (qw,qx,qy,qz)
        t=np.asarray(map(float,fullTransform.split("[")[1].split("]")[0].split()) )#translation. 
        t=t*scale_factor #scale from mm to meters
        #transformation matrix
        T=get_transformation_matrix(q,t)
        return T

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
    T_robot_graspit= get_transformation(world_xml,'robot')

    #object information
    object_filename=get_filename(world_xml,'graspableBody')
    T_object_graspit= get_transformation(world_xml,'graspableBody')

    return robot_joints, T_robot_graspit ,T_object_graspit


def get_robot_pose(T_robot_graspit ,T_object_graspit, object_pose_in_world,aditional_rotation=None, aditional_translation=None):
    """
    Compute pose of the robot
    It uses the transformation matrix of robot_graspit, object_graspit and the pose of the object in the world
    aditional rotation and translation can be include in case of unconsidered transformations (not needed of them if frame if the same when object in graspit and gazebo when no rotation and translation is the same)
    """
    #use pose to get transformation from object to world
    T_object_world=get_transformation_matrix_from_pose(object_pose_in_world)
    
    aditional_transformation=None
    #option to add extra transformaiton in case someting is off
    if aditional_rotation is not None or aditional_translation is not None:
        #if only one specified, make other 0
        if  aditional_rotation is None:
            aditional_rotation=np.array([1,0,0,0])
        elif aditional_translation is None:
            aditional_translation=np.zeros((3))
        aditional_transformation=get_transformation_matrix(aditional_rotation,aditional_translation)

    #compute inverse tranformation object_graspit
    T_graspit_object=transformation_inverse(T_object_graspit)
    
    #compute transformation robot_world
    T_robot_world=None
    if aditional_transformation is None:
        T_robot_world=np.matmul(T_object_world,np.matmul(T_graspit_object,T_robot_graspit))
    else:
        T_robot_world=np.matmul(T_object_world,np.matmul(aditional_transformation,np.matmul(T_graspit_object,T_robot_graspit)))

    #now get robot pose from the transformation matrix
    robot_pose=from_transformation_to_pose(T_robot_world)

    return robot_pose




def get_gripper_translation(direction,desired_dis, min_dis,reference_frame):
    """
    Return "pre_grasp_approach"/"post_grasp_retreat"/"post_place_retreat" for the moveit_msgs/Grasp
    input: direction of translation
    desired_dis: distance should translate in that direction
    min_dis: minimum distance to consider before changing grasp posture
    """    
    translation=GripperTranslation()
    translation.direction.header.frame_id=reference_frame #directions defined with respect of this reference frame
    translation.direction.vector.x=direction[0]
    translation.direction.vector.y=direction[1]
    translation.direction.vector.z=direction[2]
    translation.desired_distance=desired_dis
    translation.min_distance=min_dis
    return translation





def get_posture(robot_joints,time):
    """
    Return "grasp_posture"/"pre_grasp_posture" for the moveit_msgs/Grasp.
    robot_joints: robot joints during that posture
    """
    posture=JointTrajectory()
    way_point=JointTrajectoryPoint()
    way_point.time_from_start=rospy.Duration(time)#time for reach position.
    for joint_name in robot_joints:
        posture.joint_names.append(joint_name)
        way_point.positions.append(robot_joints[joint_name])

    posture.points.append(way_point) #we are defining the grasp posture with only one waypoint.
    
    return posture




#main for testing
# if __name__ == "__main__":
#     filename='worlds/mpl_checker_v3.xml'
#     package_folder='resources'
#     package_name='mpl_graspit'


#     #read world file
#     robot_joints, T_robot_graspit ,T_object_graspit= read_world_file(filename,package_name, package_folder)
#     # print(robot_joints)
#     print(T_robot_graspit)
#     print(T_object_graspit)

#     #transform robot to world
#     #recover pose from gazebo of the object usign col/row (in a other software). here receive that pose.
#     #pose for testing purposes
#     object_pose=Pose()
#     object_pose.position.x = 0
#     object_pose.position.y = 0
#     object_pose.position.z = 0
#     object_pose.orientation.x = 0
#     object_pose.orientation.y = 0
#     object_pose.orientation.z = 0
#     object_pose.orientation.w = 1

#     #option to add extra transformaiton in case someting is off
#     aditional_rotation = np.array([1,0,0,0])
#     aditional_translation=np.array([0,0,0])

#     #now get robot pose from the transformation matrix
#     robot_pose=get_robot_pose(T_robot_graspit ,T_object_graspit, object_pose,aditional_rotation, aditional_translation)
#     print(robot_pose)








