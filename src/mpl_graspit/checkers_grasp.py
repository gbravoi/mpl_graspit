#!/usr/bin/env python2
"""
This fille allow to compute the grasp of a specific checker, given board position
"""
import rospy
from gazebo_checkers.srv import CellInfo #import custom services
import utils

import math
import numpy as np

def compute_checkers_grasp(checker_row,checker_col,
                        joints_open_position,open_hand_time,
                        close_hand_time,
                        pre_grasp_aproach_direction, pre_grasp_desired_dis, pre_gras_min_dis,
                        post_grasp_aproach_direction, post_grasp_desired_dis, post_gras_min_dis,
                        world_file,
                        package_name,
                        package_folder,
                        aditional_rotation,
                        aditional_translation,
                        max_contact_force,
                        allowed_touch_objects,
                        reference_frame="world"):
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
    

    
    #find object information
    cell_info=cell_info_client(checker_row,checker_col)
    if not cell_info.available:
        # print("Cell "+ cell_info.cell_name+ " Pose:")
        # print(cell_info.pose)
        world_object_pose=cell_info.pose

        
        #read world file
        filename="worlds/"+world_file
        robot_joints, T_robot_graspit ,T_object_graspit= utils.read_world_file(filename,package_name, package_folder)

        #check robot pose
        aditional_translation=None
        #find hand rotation that have inverse kinematic solution
        step=5
        robot_pose=None
        for deg in range(0,360,step):#check all circle. Rotation in Z axis
            aditional_rotation=np.array([math.cos(math.radians(deg/2)),0,math.sin(math.radians(deg/2)),0])
            robot_pose=utils.get_robot_pose(T_robot_graspit ,T_object_graspit, world_object_pose,aditional_rotation, aditional_translation)
            #check is pose is reachabe using inverse kinematics (service must be available)
            is_valid=(utils.IK_client(robot_pose)).is_solution
            if is_valid:
                rospy.loginfo("Pose can be achived: %i" %deg)
                return robot_pose
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


    