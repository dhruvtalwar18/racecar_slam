#!/usr/bin/env python
'''
The Python program convert the voxblox_slice to occupancy grid map, using for ERL racecar
input topic: /voxblox_node/tsdf_slice (sensormsg:Pointcloud2)
output topic: /map (nav_msgs/OccupancyGrid)

Author: Shusen Lin, Zhexu Li 
Reference source: https://github.com/salihmarangoz/robot_laser_grid_mapping/blob/main/scripts/grid_mapping.py
'''
import rospy
import ros_numpy
import numpy as np
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import PointCloud2

class GridMapping:
    def __init__(self, map_center_x, map_center_y, map_size_x, map_size_y, map_resolution):
        self.map_center_x = map_center_x          #meter
        self.map_center_y = map_center_y          #meter
        self.map_size_x = map_size_x              #meter
        self.map_size_y = map_size_y              #meter
        self.map_resolution = map_resolution      #meter/cell

        map_rows = int(map_size_y / map_resolution)
        map_cols = int(map_size_x / map_resolution)

        self.gridmap = np.ones((map_rows, map_cols))*-1 #set the undiscoverd grid as -1
    
    
    
    def process_points(self, point): 
    	""" 
    	Process one point and update the gridmap using the point
    	
    	:param point: one point 
    	:type point: numpy array 
    	
    	:returns: None 
    	:type returns: None 
    
    	"""
    	
    	assert True 
    	#print("SHAPE", point.shape)
    	
    	i = round((point[1] - self.map_center_y) / self.map_resolution)
    	j = round((point[0] - self.map_center_x) / self.map_resolution)
    	#print("IJ", i, j)
    	
    	
    	self.gridmap[int(i), int(j)] = point[2]
    
    
    
    
    def update(self, points):
        '''
        Updates the gridmap
        
        :param points: points for updating 
        :type points: numpy array 
        
        :returns: updated gridmap 
        :type returns: numpy array 
        
        '''
        assert True
        
        
        np.apply_along_axis(self.process_points, axis = 1, arr = points)

        return self.gridmap

class GridMappingROS:
    def __init__(self):
        rospy.init_node('Vox_Mapping', anonymous=False)
        self.map_last_publish = rospy.Time()
        self.map_frame        = rospy.get_param('~map_frame', 'racecar01/map')
        self.map_resolution   = rospy.get_param('~map_resolution', 0.4)

        # Creata a OccupancyGrid message template
        self.map_msg = OccupancyGrid()
        self.map_msg.header.frame_id = self.map_frame
        self.map_msg.info.resolution = self.map_resolution
        self.map_msg.info.width  = 25
        self.map_msg.info.height = 25
        self.map_msg.info.origin.position.x = -1
        self.map_msg.info.origin.position.y = -1

        self.pc_sub = rospy.Subscriber('pc_input',PointCloud2,self.pc_callback,queue_size=2)
        self.map_pub = rospy.Publisher('mapout' , OccupancyGrid, queue_size=2)


    def fresh_gridmapping(self,points):

        x_min = min(points[:,0])
        x_max = max(points[:,0])
        y_min = min(points[:,1])
        y_max = max(points[:,1])

        self.map_size_x = int((np.abs(x_max-x_min))+5)
        self.map_size_y = int((np.abs(y_max-y_min))+5)
        self.map_center_x  = x_min-2
        self.map_center_y  = y_min-2

        self.gridmapping = GridMapping(self.map_center_x, self.map_center_y, self.map_size_x, self.map_size_y, self.map_resolution)
        self.is_gridmapping_initialized = False

        self.map_msg = OccupancyGrid()
        self.map_msg.header.frame_id = self.map_frame
        self.map_msg.info.resolution = self.map_resolution
        self.map_msg.info.width = int(self.map_size_x / self.map_resolution)
        self.map_msg.info.height = int(self.map_size_y / self.map_resolution)
        self.map_msg.info.origin.position.x = self.map_center_x
        self.map_msg.info.origin.position.y = self.map_center_y

        print('Voxblox_slice convert to Gridmap success!')
        # print('pc_xmin:'+str(x_min) + ' pc_xmax:'+str(x_max ))
        # print('pc_ymin:'+str(y_min) + ' pc_ymax:'+str(y_max) )
        # print('map_size_x: '+str(self.map_size_x)+ ' map_size_y: '+str(self.map_size_y))
        # print('map_center_x: '+str(self.map_center_x)+ ' map_center_y: '+str(self.map_center_y))
        # print('gridmap_width:'+ str(self.map_msg.info.width) + ' gridmap_height:'+ str(self.map_msg.info.height))
        # print('gridmap_origin_x:'+ str(self.map_msg.info.origin.position.x) + ' ridmap_origin_y:'+ str(self.map_msg.info.origin.position.y))
        # print(' ')
     
    def publish_occupancygrid(self, gridmap, stamp):
        # Convert gridmap to ROS supported data type : int8[]
        # http://docs.ros.org/en/melodic/api/nav_msgs/html/msg/OccupancyGrid.html
        # The map data, in row-major order, starting with (0,0).  Occupancy probabilities are in the range [0,100].  

        gridmap_int8 = gridmap.astype(dtype=np.int8)
    
        # Publish map
        self.map_msg.data = gridmap_int8
        self.map_msg.header.stamp = stamp
        self.map_pub.publish(self.map_msg)
        rospy.loginfo_once("Published map")

    def pc_callback(self,data):
        '''
        the main callback function to convert the pointcloud2 data
        '''
        try:
            pc = ros_numpy.numpify(data)
            points=np.zeros((pc.shape[0],3))

            points[:,0] = pc['x']
            points[:,1] = pc['y']
            points[:,2] = pc['intensity']
            points[:,2] = np.where(points[:,2]>0,0,100)

            self.fresh_gridmapping(points)
            
            self.map_last_publish = rospy.Time.now()
            gridmap = self.gridmapping.update(points).flatten()# update map
            self.publish_occupancygrid(gridmap, data.header.stamp)
        except ValueError as error:
            print('Receive empty pointcloud, still waiting...')
            pass

if __name__ == "__main__": 
    voxblox_map = GridMappingROS()
    while not rospy.is_shutdown():
    	
    	rospy.spin()

 
