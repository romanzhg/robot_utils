#!/usr/bin/env python3
import rospy
import tf
import sys
import numpy
from nav_msgs.msg import OccupancyGrid
import pickle

if __name__ == '__main__':
    rospy.init_node('occupancy_grid_publisher')
    og_filename = "mcl.pkl"
    pkl_file = open(og_filename, 'rb')
    og = pickle.load(pkl_file)
    og.header.frame_id = "map" # Convert to tf2 frame format

    rate = rospy.Rate(1)
    og_pub = rospy.Publisher("/projected_map", OccupancyGrid, queue_size=1)
    while not rospy.is_shutdown():
        if og_pub.get_num_connections() > 0:
            og_pub.publish(og)
        rate.sleep()
        

