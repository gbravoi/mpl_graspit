#!/usr/bin/env python2
"""
This file create grasp message given a world file, and the real position of the object in the world.
"""
import utils
from moveit_msgs.msg import  Grasp
from geometry_msgs.msg import Pose
from moveit_commander import  MoveGroupCommander

#define which hrasp which object
grasp_world_files={
                    "checkers": 'worlds/mpl_checker_v3.xml'
}

def compute_grasp(object_id,grasp_type, object_pose,aditional_rotation=None,aditional_translation=None):
    """
    Compute the grasp of an object, given the type and object position/orientation in world
    inputs:
    --object_id: ID in the Scene on moveit
    --grasp_type: name in dictionary that references the world file where the robot is grasping the desired object.
    --object_pose: object Pose in world frame (gazebo)
    --aditional rotation and translation: can be include in case of unconsidered transformations (not needed of them if frame if the same when object in graspit and gazebo when no rotation and translation is the same)
    
    """
    filename=grasp_world_files[grasp_type]
    package_folder='resources'
    package_name='mpl_graspit'
    #read world file
    robot_joints, T_robot_graspit ,T_object_graspit= utils.read_world_file(filename,package_name, package_folder)
    # print(robot_joints)
    # print(T_robot_graspit)
    # print(T_object_graspit)
    #now get robot pose from the transformation matrix
    robot_pose=utils.get_robot_pose(T_robot_graspit ,T_object_graspit, object_pose,aditional_rotation, aditional_translation)
    # print(robot_pose)

    #create moveit Grasp
    grasp= Grasp()
    #define pre-grasp joints posture
    move_group_hand = MoveGroupCommander("mpl_hand")
    openhand_joints=move_group_hand.getNamedTargetValues("extended_hand")
    print(openhand_joints)


#main for testing
if __name__ == "__main__":
    object_pose=Pose()
    object_pose.position.x = 0
    object_pose.position.y = 0
    object_pose.position.z = 0
    object_pose.orientation.x = 0
    object_pose.orientation.y = 0
    object_pose.orientation.z = 0
    object_pose.orientation.w = 1
    object_id="example"
    compute_grasp(object_id,"checkers", object_pose,aditional_rotation=None,aditional_translation=None)
    

