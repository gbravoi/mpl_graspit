#!/usr/bin/env python2
"""
This fille allow to compute the grasp of a specific checker, given board position
"""
import rospy
from gazebo_checkers.srv import CellInfo #import custom services
import utils

import math
import numpy as np

from mpl_utils.ik_client import IK_client
from moveit_msgs.msg import  Grasp, GripperTranslation

def compute_checkers_grasp(checker_row,checker_col,
                        joints_open_position,open_hand_time,
                        close_hand_time,
                        pre_grasp_aproach_direction, pre_grasp_desired_dis, pre_gras_min_dis,
                        post_grasp_aproach_direction, post_grasp_desired_dis, post_gras_min_dis,
                        max_contact_force,
                        allowed_touch_objects,
                        robot_base_frame):
    def cell_info_client(from_row,from_col):
        """
        function to get checker information from gazebo
        """
        rospy.wait_for_service('checkers/cell_info/')
        try:
            cell_info = rospy.ServiceProxy('checkers/cell_info/', CellInfo)
            resp = cell_info(from_row,from_col)
            return resp
        except rospy.ServiceException as e:
            rospy.loginfo("Service call checker info failed: %s"%e)
    
    #define some parameters
    world_file="mpl_checker_v3.xml"
    package_folder='resources'
    package_name='mpl_graspit'

    #create moveit Grasp
    grasp= Grasp()
    
    #find object information
    cell_info=cell_info_client(checker_row,checker_col)
    if not cell_info.available:

        #object pose in world frame
        objects_pose_w=cell_info.pose 
        print("object pose")
        print(str(objects_pose_w))

        objects_pose_r=utils.transform_pose_between_frames(objects_pose_w, "world", robot_base_frame)#object pose in robot frame


        #read world file
        filename="worlds/"+world_file
        robot_joints, T_robot_graspit ,T_object_graspit= utils.read_world_file(filename,package_name, package_folder)

        #check robot pose
        aditional_translation=None
        #find hand rotation that have inverse kinematic solution
        step=1
        robot_pose=None

        for deg in range(0,360,step):#check all circle. Rotation in Z axis
            aditional_rotation=np.array([math.cos(math.radians(deg/2)),0,0,math.sin(math.radians(deg/2))])
            robot_pose_r=utils.get_robot_pose(T_robot_graspit ,T_object_graspit, objects_pose_r,aditional_rotation, aditional_translation)
            #tranform pose to robot frame
            

            #check is pose is reachabe using inverse kinematics (service must be available)
            is_valid=(IK_client(robot_pose_r)).is_solution
            if is_valid:
                rospy.loginfo("Pose can be achived: %i" %deg)

                #----complete grasp information---------------
                #define pre-grasp joints posture. basically open hand.
                grasp.pre_grasp_posture=utils.get_posture(joints_open_position,open_hand_time)

                #define hand finger posture during grasping
                grasp.grasp_posture=utils.get_posture(robot_joints,close_hand_time)
                
                #define grasp pose: position of the end effector during grasp
                grasp.grasp_pose.header.frame_id=robot_base_frame #pose in this reference frame
                grasp.grasp_pose.pose=robot_pose_r

                #pre_grasp_approach.The approach direction the robot will move when aproaching the object
                grasp.pre_grasp_approach=utils.get_gripper_translation(pre_grasp_aproach_direction,pre_grasp_desired_dis, pre_gras_min_dis,"world")

                #post_grasp_retreat. The retreat direction to take after a grasp has been completed (object is attached)
                grasp.post_grasp_retreat=utils.get_gripper_translation(post_grasp_aproach_direction,post_grasp_desired_dis, post_gras_min_dis,"world")

                #max contact force
                grasp.max_contact_force=max_contact_force

                #allowd_tocuh_force
                grasp.allowed_touch_objects=allowed_touch_objects

                return grasp





        rospy.loginfo("Pose CAN NOT  be achived")
        return None
        
            

        

        # grasp_result=grasps.compute_grasp(
        #     joints_open_position,open_hand_time,
        #     close_hand_time,
        #     pre_grasp_aproach_direction, pre_grasp_desired_dis, pre_gras_min_dis,
        #     post_grasp_aproach_direction, post_grasp_desired_dis, post_gras_min_dis,
        #     world_file,
        #     package_name,
        #     package_folder,
        #     world_object_pose,
        #     aditional_rotation,
        #     aditional_translation,
        #     max_contact_force,
        #     allowed_touch_objects,
        #     reference_frame)
        # return grasp_result

    else:
        rospy.loginfo("Cell "+cell_info.cell_name+" doesn't have a checker piece")


    