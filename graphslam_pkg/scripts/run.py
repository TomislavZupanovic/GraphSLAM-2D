#!/usr/bin/env python

import rospy
from graphslam_pkg.srv import simulationStartService, simulationStartServiceResponse
from graphslam.slam import SLAM2D
from graphslam.world import World

def handle_request_callback(req):
    world_size = 100
    measurement_range = 50.0
    measurement_noise = 2.0
    motion_noise = 2.0
    
    world = World(world_size, req.num_landmarks)
    slam = SLAM2D(req.steps, world_size, req.num_landmarks, measurement_noise, motion_noise)
    
    # Make data and run SLAM
    world.make_data(req.steps, measurement_range, motion_noise, measurement_noise)
    slam.run_slam(world.data)
    
    # Get estimated robot and landmark positions
    poses, landmarks = slam.get_poses_landmarks()
    
    # Print results
    slam.show_results()
    world.display_world(poses, landmarks)
    
    return simulationStartServiceResponse(True)
    
if __name__ == '__main__':
    rospy.init_node('graphslam_server_node')
    s = rospy.Service('graphslam_server', simulationStartService, handle_request_callback)
    print("Init sequence done.")
    rospy.spin()
