#!/usr/bin/env python2
"""
This file create grasp message given a world file, and the real position of the object in the world.
"""
import utils
import rospy
from moveit_msgs.msg import  Grasp





#define which hrasp which object


def compute_grasp(
    joints_open_position,open_hand_time,
    close_hand_time,
    pre_grasp_aproach_direction, pre_grasp_desired_dis, pre_gras_min_dis,
    post_grasp_aproach_direction, post_grasp_desired_dis, post_gras_min_dis,
    world_file,
    package_name,
    package_folder,
    world_object_pose,
    aditional_rotation=None,
    aditional_translation=None,
    max_contact_force=-1,
    allowed_touch_objects=[],
    reference_frame="world"):
    """
    Compute the grasp of an object
    inputs:
    --joints_open_position: dictionary of the joints and angle in open position (pre-grasp)
    --open_hand_time: time desired to open hand
    --close_hand_time: time to reach grasp position with the fingers.
    --pre_grasp_aproach_direction:  (x,y,z) direction end effector will aproach grasp_poses in world frame
    --pre_grasp_desired_dis: distance will travel when aproaching 
    --pre_gras_min_dis: the min distance that must be considered feasible before the grasp is even attempted
    --post_grasp_aproach_direction: (x,y,z) direction end effector will retreat after closing the hand (from grasp_pose). in world frame
    --post_grasp_desired_dis: distance that will retreat
    --post_gras_min_dis: min distance to consider is enough far away?
    --max_contact_force: the maximum contact force to use while grasping (<=0 to disable)
    --allowed_touch_objects: objects that can be touched/pushed/moved in the course of grasping. list of strings.
    --world_file: world file of the hand holding the object. For example 'mpl_checker_v3.xml'
    --package_name: name of the package where the world file is located
    --package_folder: location of the folders "moldels" and "worlds" from graspit, inside the package
    --world_object_pose: object Pose in world frame (gazebo/moveit)
    --aditional rotation and translation: can be include in case of unconsidered transformations between the object in gazebo and graspit (not needed of them if frame if the same when object in graspit and gazebo when no rotation and translation is the same)
    --reference_frame: directions and object position reference frame
    """
    filename="worlds/"+world_file
    #read world file
    robot_joints, T_robot_graspit ,T_object_graspit= utils.read_world_file(filename,package_name, package_folder)

    #create moveit Grasp
    grasp= Grasp()

    #grasp id (optional)

    #define pre-grasp joints posture. basically open hand.
    grasp.pre_grasp_posture=utils.get_posture(joints_open_position,open_hand_time)

    #define hand finger posture during grasping
    grasp.grasp_posture=utils.get_posture(robot_joints,close_hand_time)

    #define grasp pose: position of the end effector during grasp
    grasp.grasp_pose.header.frame_id=reference_frame #pose in this reference frame
    grasp.grasp_pose.pose=utils.get_robot_pose(T_robot_graspit ,T_object_graspit, world_object_pose,aditional_rotation, aditional_translation)
    
    #grasp quality (no needed/used. can be obtained from graspit)

    #pre_grasp_approach.The approach direction the robot will move when aproaching the object
    grasp.pre_grasp_approach=utils.get_gripper_translation(pre_grasp_aproach_direction,pre_grasp_desired_dis, pre_gras_min_dis,reference_frame)

    #post_grasp_retreat. The retreat direction to take after a grasp has been completed (object is attached)
    grasp.post_grasp_retreat=utils.get_gripper_translation(post_grasp_aproach_direction,post_grasp_desired_dis, post_gras_min_dis,reference_frame)

    #max contact force
    grasp.max_contact_force=max_contact_force

    #allowd_tocuh_force
    grasp.allowed_touch_objects=allowed_touch_objects

    return grasp













#main for testing
# if __name__ == "__main__":
#     object_pose=Pose()
#     object_pose.position.x = 0
#     object_pose.position.y = 0
#     object_pose.position.z = 0
#     object_pose.orientation.x = 0
#     object_pose.orientation.y = 0
#     object_pose.orientation.z = 0
#     object_pose.orientation.w = 1
#     object_id="example"

#     #get joints open hand
#     # move_group_hand = MoveGroupCommander("mpl_hand")
#     # openhand_joints=move_group_hand.get_named_target_values("extended_hand")
#     joints_open_position={'mpl_right_arm__thumb2': 0.0, 'mpl_right_arm__thumb3': 0.0, 'mpl_right_arm__thumb0': 0.0, 'mpl_right_arm__ring0': 0.0, 'mpl_right_arm__index2': 0.0, 'mpl_right_arm__index3': 0.0, 'mpl_right_arm__index0': 0.0, 'mpl_right_arm__index1': 0.0, 'mpl_right_arm__thumb1': 0.0, 'mpl_right_arm__middle2': 0.0, 'mpl_right_arm__middle3': 0.0, 'mpl_right_arm__middle0': 0.0, 'mpl_right_arm__middle1': 0.0, 'mpl_right_arm__pinky1': 0.0, 'mpl_right_arm__pinky0': 0.0, 'mpl_right_arm__pinky3': 0.0, 'mpl_right_arm__pinky2': 0.0, 'mpl_right_arm__ring2': 0.0, 'mpl_right_arm__ring3': 0.0, 'mpl_right_arm__ring1': 0.0}
#     # print(joints_open_position)
#     open_hand_time=0.5
#     close_hand_time=0.5
#     pre_grasp_aproach_direction=[1,0,0]
#     pre_grasp_desired_dis=10
#     pre_gras_min_dis=1
#     post_grasp_aproach_direction=[1,0,0]
#     post_grasp_desired_dis=9
#     post_gras_min_dis=0.5
#     world_file="mpl_checker_v3.xml"
#     package_folder='resources'
#     package_name='mpl_graspit'
#     world_object_pose=object_pose

#     grasp=compute_grasp(
#     joints_open_position=joints_open_position,
#     open_hand_time=0.5,
#     close_hand_time=0.5,
#     pre_grasp_aproach_direction=[1,0,0],
#     pre_grasp_desired_dis=10,
#     pre_gras_min_dis=1,
#     post_grasp_aproach_direction=[1,0,0],
#     post_grasp_desired_dis=9,
#     post_gras_min_dis=0.5,
#     world_file="mpl_checker_v3.xml",
#     package_folder='resources',
#     package_name='mpl_graspit',
#     world_object_pose=object_pose)

#     print(grasp)
    

