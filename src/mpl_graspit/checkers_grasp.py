#!/usr/bin/env python2
"""
This fille allow to compute the grasp of a specific checker, given board position
"""
import rospy
from gazebo_checkers.srv import CellInfo #import custom services
from  mpl_graspit import grasps

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
        grasp_result=grasps.compute_grasp(
            joints_open_position,open_hand_time,
            close_hand_time,
            pre_grasp_aproach_direction, pre_grasp_desired_dis, pre_gras_min_dis,
            post_grasp_aproach_direction, post_grasp_desired_dis, post_gras_min_dis,
            world_file,
            package_name,
            package_folder,
            world_object_pose,
            aditional_rotation,
            aditional_translation,
            max_contact_force,
            allowed_touch_objects,
            reference_frame)
        return grasp_result

    else:
        rospy.loginfo("Cell "+cell_info.cell_name+" doesn't have a checker piece")


    