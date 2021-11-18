#!/usr/bin/env python
""" Simple occupancy-grid-based mapping without localization. 

Subscribed topics:
/scan
/odom

Published topics:
/map 
/map_metadata

"""
import rospy
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from nav_msgs.msg import OccupancyGrid, MapMetaData, Odometry
from geometry_msgs.msg import Pose, Point, Quaternion
from sensor_msgs.msg import LaserScan
import math
import numpy as np

class Map(object):
    """ 
    The Map class stores an occupancy grid as a two dimensional
    numpy array. 
    
    Public instance variables:

        width      --  Number of columns in the occupancy grid.
        height     --  Number of rows in the occupancy grid.
        resolution --  Width of each grid square in meters. 
        origin_x   --  Position of the grid cell (0,0) in 
        origin_y   --    in the map coordinate system.
        grid       --  numpy array with height rows and width columns.
        
    
    Note that x increases with increasing column number and y increases
    with increasing row number. 
    """

    def __init__(self, origin_x=-5.0, origin_y=-5.0, resolution=0.1, 
                 width=100, height=100):
        """ Construct an empty occupancy grid.
        
        Arguments: origin_x, 
                   origin_y  -- The position of grid cell (0,0) in the
                                map coordinate frame.
                   resolution-- width and height of the grid cells 
                                in meters.
                   width, 
                   height    -- The grid will have height rows and width
                                columns cells.  width is the size of
                                the x-dimension and height is the size
                                of the y-dimension.
                                
         The default arguments put (0,0) in the center of the grid. 
                                
        """
        self.origin_x = origin_x
        self.origin_y = origin_y
        self.resolution = resolution
        self.width = width 
        self.height = height 
        self.grid = np.zeros((height, width))

    def to_message(self):
        """ Return a nav_msgs/OccupancyGrid representation of this map. """
     
        grid_msg = OccupancyGrid()

        # Set up the header.
        grid_msg.header.stamp = rospy.Time.now()
        grid_msg.header.frame_id = "map"

        # .info is a nav_msgs/MapMetaData message. 
        grid_msg.info.resolution = self.resolution
        grid_msg.info.width = self.width
        grid_msg.info.height = self.height
        
        # Rotated maps are not supported... quaternion represents no
        # rotation. 
        grid_msg.info.origin = Pose(Point(self.origin_x, self.origin_y, 0),
                               Quaternion(0, 0, 0, 1))

        # Flatten the numpy array into a list of integers from 0-100.
        # This assumes that the grid entries are probalities in the
        # range 0-1. This code will need to be modified if the grid
        # entries are given a different interpretation (like
        # log-odds).
        

        flat_grid = self.grid.reshape((self.grid.size,)) * 100
        #print('-----------')
        
        #print (flat_grid)
        flat_grid=flat_grid.astype('int8')
      
        ##########


        grid_msg.data = list(np.round(flat_grid))
       

        return grid_msg

    def set_cell(self, x, y, val):
        """ Set the value of a cell in the grid. 

        Arguments: 
            x, y  - This is a point in the map coordinate frame.
            val   - This is the value that should be assigned to the
                    grid cell that contains (x,y).

        This would probably be a helpful method!  Feel free to throw out
        point that land outside of the grid. 
        """
        pass

class Mapper(object):
    """ 
    The Mapper class creates a map from laser scan data.
    """
    
    def __init__(self):
        """ Start the mapper. """

        rospy.init_node('mapper')
        self._map = Map()

        # Setting the queue_size to 1 will prevent the subscriber from
        # buffering scan messages.  This is important because the
        # callback is likely to be too slow to keep up with the scan
        # messages. If we buffer those messages we will fall behind
        # and end up processing really old scans.  Better to just drop
        # old scans and always work with the most recent available.
        rospy.Subscriber('scan',
                         LaserScan, self.scan_callback, queue_size=1)
        rospy.Subscriber('odom',
                         Odometry, self.get_my_odom, queue_size=1)

        # Latched publishers are used for slow changing topics like
        # maps.  Data will sit on the topic until someone reads it. 
        self._map_pub = rospy.Publisher('map', OccupancyGrid, latch=True, queue_size=1)
        self._map_data_pub = rospy.Publisher('map_metadata', 
                                             MapMetaData, latch=True, queue_size=1)
        
        rospy.spin()

    def get_my_odom(self, msg):
        """Gets the odometry of the robot (X, Y, and Theta)"""
        global pos, yaw
        pos =  msg.pose.pose.position
        orientation = msg.pose.pose.orientation
        quaternion = [orientation.x,orientation.y,orientation.z, orientation.w]
        (roll, pitch, yaw) = euler_from_quaternion(quaternion)


    def get_indix(self, x, y):
        x = x - self._map.origin_x
        y = y - self._map.origin_y
        i = int(round(x/self._map.resolution))
        j = int(round(y/self._map.resolution))

        return i, j

    def scan_callback(self, scan):
        """ Update the map on every scan callback. """
        global pos 
        ###############updat the mape based on scan reading########## 
        #you need to writr your code heer to update your map based on
        #sensor data
        print('pos.x =  ', pos.x)
        print('pos.y =  ', pos.y)
        print('yaw =  ', yaw)
        robot_radius = 5/100000 #the lower the robot_radius the better reading on the map, best case scenario - the robot is treated as a moving point
        

        for i in range(len(scan.ranges)):

            angular_offset = math.radians(i) #math.radians(i) is an angular offset
            distance_to_object = scan.ranges[i]

            if not np.isinf(scan.ranges[i]):
                if scan.ranges[i] > scan.range_min and scan.ranges[i] < scan.range_max:
                    x_pos = math.cos(angular_offset) * (robot_radius + distance_to_object) #finding the x position of the obstacle
                    y_pos = math.sin(angular_offset) * (robot_radius + distance_to_object) #finding the y position of the obstacle
               
                    x_prime = (x_pos * math.cos(yaw)) + (y_pos * -math.sin(yaw)) #rotating the x position of the obstacle into the global coordinates
                    y_prime = (x_pos * math.sin(yaw)) + (y_pos * math.cos(yaw)) #rotating the y position of the obstacle into the global coordinates

                    x_coord = x_prime + pos.x #translating the x' position of the obstacle into the global coordinates
                    y_coord = y_prime + pos.y #translating the y' position of the obstacle into the global coordinates
                
                    (k, l) = self.get_indix(x_coord, y_coord) #Inverting the map, otherwise  the occupancy grid reading is not accurate
                
                    self._map.grid[l, k] = 0.9
            else:
                (k, l) = self.get_indix(x_coord, y_coord)
                self._map.grid[l, k] = 0.1
         # Now that the map wass updated, so publish it!
        rospy.loginfo("Scan is processed, updating the map - look at rviz.")
        self.publish_map()


    def publish_map(self):
        """ Publish the map. """
        grid_msg = self._map.to_message()
        self._map_data_pub.publish(grid_msg.info)
        self._map_pub.publish(grid_msg)


if __name__ == '__main__':
    try:
        m = Mapper()
    except rospy.ROSInterruptException:
        pass

"""
https://w3.cs.jmu.edu/spragunr/CS354_S14/labs/mapping/mapper.py
https://www.youtube.com/watch?v=K1ZFkR4YsRQ
"""