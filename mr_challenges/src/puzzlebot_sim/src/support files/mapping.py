#!/usr/bin/env python  
""" Node that generates a map of the environment based on the laser scan data.
and the odometry data.

Author: kjartan@tec.mx (Kjartan Halvorsen) with help from github copilot

Notes.
1) The scan data give information about free space as well as obstacles. Each ray in the scan will cover a number
of pixels in the map. The map should be updated by setting the pixels covered by the ray to 0 (free) and the last pixel
to occupied (100). The map should be updated only if the ray range is less than the max_range of the scan.
2) You should determine the number of points in each scan ray by multiplying the range of the ray by the map resolution.
Then you convert these points (each corresponding to a pixel) from a robot frame to a map frame using the odometry data.
3) The map should be updated only if the robot has moved a certain distance since the last update. This is to
avoid updating the map too often, since it is a somewhat expensive operation.
4) It can be more efficient to use numpy arrays for the rigid transformations needed to convert scans
to map coordinates. To make this work, you need to convert the geometry_msgs/TransformStamped to a numpy array.
See https://answers.ros.org/question/332407/transformstamped-to-transformation-matrix-python/
With a transform matrix T, you can transform a number of points at once by collecting the points in a numpy array
and multiplying the array with T.
To use numpify, you need to install the package ros-meldic-ros-numpy.


"""
import sys
import numpy as np
import rospy
from nav_msgs.msg import OccupancyGrid, MapMetaData, Odometry
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Pose
from ros_numpy import numpify
import math

class Mapper:
    def __init__(self, map_width, map_height, map_resolution):
        """
        Arguments
        ---------
        map_width : float
            Width of map in pixels (x-axis)
        map_height : float
            Height of map in pixels (y-axis)
        map_resolution : float
            Resolution of map in meter per pixel
        """
        self.scan_listener = rospy.Subscriber('/scan', LaserScan,
                                              self.scan_callback)
        self.odom_listener = rospy.Subscriber('/odometry', Odometry,
                                              self.odom_callback)
        self.map_pub = rospy.Publisher('/map' , OccupancyGrid, queue_size=1 )
        self.rate = rospy.Rate(5.0)
        self.map = OccupancyGrid()
        self.map.info.map_load_time = rospy.Time.now()
        self.map.info.resolution = map_resolution
        self.map.info.width = map_width
        self.map.info.height = map_height
        self.map.info.origin.position.x = -(map_width*map_resolution)/2.0
        self.map.info.origin.position.y = (map_height*map_resolution)/2.0
        self.map.info.origin.position.z = 0.0
        self.map.info.origin.orientation.x = 1.0
        self.map.info.origin.orientation.y = 0.0
        self.map.info.origin.orientation.z = 0.0
        self.map.info.origin.orientation.w = 0.0

        self.map.data = np.zeros(map_width*map_height, dtype=np.int8)
        self.map.data[:] = -1 # Unknown
        self.map2d = np.zeros((map_width, map_height), dtype=np.int8) # For computation
        self.scan = None
        self.odom = None


    def scan_callback(self, msg):
        """Called when a new scan is available from the lidar. """
        self.scan = msg

    def odom_callback(self, msg):
        """Called when a new odometry message is available. """
        self.odom = msg

    def mapit(self):
        px = None
        py = None
        while not rospy.is_shutdown():

            if self.scan is not None and self.odom is not None:
                map_width = self.map.info.width
                map_height = self.map.info.height
                map_resolution = self.map.info.resolution
                map_origin = self.map.info.origin.position
                robot_pose = self.odom.pose.pose
                for i, ray in enumerate(self.scan.ranges):
                    if ray < self.scan.range_max:
                    # Transform ray from robot frame to map frame
                        angle = self.scan.angle_min + i*self.scan.angle_increment
                        x = ray*np.cos(angle)
                        y = ray*np.sin(angle)
                        px = int((x + map_origin.x) / map_resolution)
                        py = int((y + map_origin.y) / map_resolution)
                    if px >= 0 and py >= 0 and px < map_width and py < map_height:
                            # Set end point to occupied
                            self.map2d[py, px] = 100
                            # Set pixels along the ray to free
                            delta_x = int(x / map_resolution)
                            delta_y = int(y / map_resolution)
                            steps = max(abs(delta_x), abs(delta_y))
                            for step in range(1, steps):
                                cx = int(px + step * delta_x / steps)
                                cy = int(py + step * delta_y / steps)
                                if cx >= 0 and cy >= 0 and cx < map_width and cy < map_height:
                                    self.map2d[cy, cx] = 0

                orig_m, xy_m = scan_to_map_coordinates(self.scan, self.odom, self.map.info.origin)

                print(orig_m)
                for xy_ in xy_m:
                    #print(xy_)
                    ray_to_pixels(orig_m[0], orig_m[1], xy_[0], xy_[1], self.map.info.resolution, self.map2d)


                 # Publish the map
                np.copyto(self.map.data,  self.map2d.reshape(-1)) # Copy from map2d to 1d data, fastest way
                self.map.header.stamp = rospy.Time.now()
                self.map_pub.publish(self.map)
            self.rate.sleep()        
            

def ray_to_pixels(xr, yr, x, y, map_resolution, map):
    """ Set the pixels along the ray with origin (xr,yr) and with range ending at (x,y) to 0 (free) and the end point to 100 (occupied).
    Arguments
    ---------
    xr : float
        x-coordinate of the robot in the map frame
    yr : float
        y-coordinate of the robot in the map frame
    x : ndarray
        x-coordinates of the scan in the map frame
    y : ndarray
        y-coordinates of the scan in the map frame
    map_resolution : float
        Resolution of map in meter/pixel
    map : ndarray
        The map as a 2d numpy array

    Tests
    ------
    >>> mapmsg = test_map()
    >>> map = np.zeros((mapmsg.info.width, mapmsg.info.height), dtype=np.int8)
    >>> map[:] = -1
    >>> xr = 6.0
    >>> yr = 3.0
    >>> # Test 1 - ray from (6,3) to (7,3)
    >>> x = 7.0
    >>> y = 3.0
    >>> ray_to_pixels(xr, yr, x, y, mapmsg.info.resolution, map)
    >>> map[6, 12] == 0
    True
    >>> map[6, 13] == 0
    True
    >>> map[6, 14] == 100
    True
    >>> # Test 2 - ray from (6,3) to (6,2)
    >>> x = 6.0
    >>> y = 2.0
    >>> ray_to_pixels(xr, yr, x, y, mapmsg.info.resolution, map)
    >>> map[6, 12] == 0
    True
    >>> map[5, 12] == 0
    True
    >>> map[4, 12] == 100
    True
    >>> # Test 3 - ray from (6,3) to (5,3)
    >>> x = 5.0
    >>> y = 3.0
    >>> ray_to_pixels(xr, yr, x, y, mapmsg.info.resolution, map)
    >>> map[6, 12] == 0
    True
    >>> map[6, 11] == 0
    True
    >>> map[6, 10] == 100
    True
    >>> #map
    """

    v = np.array([x-xr, y-yr])
    n_pixels = np.linalg.norm(v)/map_resolution
    v = v/n_pixels
    xp = int(round(xr/map_resolution))
    yp = int(round(yr/map_resolution))
    # Set pixels along the ray to 0 (free)
    for i in range(int(n_pixels)):
        #-------------------------------------------
        # Your code here
        # Determine x_ and y_ from xp, yp, i and v
        x_ = xp + int(round(i*v[0]))
        y_ = yp + int(round(i*v[1]))
        map[y_, x_] = 0
    # Set last pixel of ray to 100
    map[int(round(y/map_resolution)), int(round(x/map_resolution))] = 100

def scan_to_map_coordinates(scan, odom, origin):
    """ Convert a scan from the robot frame to the map frame.
    Arguments
    ---------
    scan : LaserScan
        The scan to convert
    odom : Odometry
        The odometry message providing the robot pose
    origin : Pose
        The pose of the map in the odom frame
    Returns
    -------
    (xr, yr) : tuple of floats
        The position of the robot in the map frame
    xy : list-like
        list of tuples (x,y) with the coordinates of the scan end points in the map frame

    Tests
    -----
    >>> # Test 1 - With map origin at (0,0), no rotation of the map, and robot at (1,2) in odom frame
    >>> scan = test_laser_scan()
    >>> odom = test_odometry()
    >>> origin = Pose()
    >>> orig, xy = scan_to_map_coordinates(scan, odom, origin)
    >>> np.allclose(orig, (1.0, 2.0))
    True
    >>> np.allclose(xy,[(2.0, 2.0), (1.0, 3.0), (0.0, 2.0)])
    True
    >>> # Test 2 - With map origin at (-5, 5), map frame rotated pi about x, and robot at (1,2) in odom frame
    >>> map = test_map()
    >>> origin = map.info.origin
    >>> orig, xy = scan_to_map_coordinates(scan, odom, origin)
    >>> np.allclose(orig, (6.0, 3.0))
    True
    >>> np.allclose(xy,[(7.0, 3.0), (6.0, 2.0), (5.0, 3.0)])
    True
    >>>
    """

    T_ob = numpify(odom.pose.pose)  # Transform from odom to base_link
    T_om = numpify(origin)
    T_mo = np.linalg.inv(T_om)
    T_mb = np.dot(T_mo, T_ob)
    TT= np.eye(3)
    TT[:2, :2] = T_mb[:2, :2]
    TT[:2, 2] = T_mb[:2, 3]

    #------------------------------------------------------
    # Your code here
    # Transform all the scan end points to the map frame
    # scan_endpoints =
    #-----------------------------------------------------
    scan_endpoints = []
    for i, r in enumerate(scan.ranges):
        if r == float('inf') or r == 0.0:
            continue
        angle = scan.angle_min + i * scan.angle_increment
        x = r * np.cos(angle)
        y = r * np.sin(angle)
        p = np.array([x, y, 1.0])
        p = np.dot(TT, p)
        scan_endpoints.append((p[0], p[1]))
    return TT[:2, 2], scan_endpoints

def polar_to_cartesian(r, th):
    """ Convert a polar coordinate to a cartesian coordinate.
    Arguments
    ---------
    r : float
        The radius
    th : float
        The angle
    Returns
    -------
    (x, y) : tuple of floats
        The cartesian coordinates
    Tests
    -----
    >>> x, y = polar_to_cartesian(1.0, 0.0)
    >>> np.allclose(x, 1.0)
    True
    >>> np.allclose(y, 0.0)
    True
    >>> x, y = polar_to_cartesian(1.0, np.pi/2)
    >>> np.allclose(x, 0.0)
    True
    >>> np.allclose(y, 1.0)
    True
    >>> x, y = polar_to_cartesian(1.0, np.pi)
    >>> np.allclose(x, -1.0)
    True
    >>> np.allclose(y, 0.0)
    True
    """

    #------------------------------------------------------
    # Your code here
    # Convert the polar coordinate to a cartesian coordinate
    # x =
    # y =
    #-----------------------------------------------------
    x = r * np.cos(th)
    y = r * np.sin(th)
    #-----------------------------------------------------

    return (x, y)

def test_laser_scan():
    """ Create a simple test LaserScan message.
    There are 3 rays, to the left, straight ahead and to the right.
    Each ray has range 1.0. """
    scan = LaserScan()
    scan.header.frame_id = 'base_link'
    scan.angle_min = -np.pi/2
    scan.angle_max = np.pi/2
    scan.angle_increment = np.pi/2
    scan.range_min = 0.0
    scan.range_max = 10.0
    scan.ranges = [1.0, 1.0, 1.0]
    return scan
def test_odometry():
    """ Create a test Odometry message. """
    odom = Odometry()
    odom.header.frame_id = 'odom'
    odom.pose.pose.position.x = 1.0
    odom.pose.pose.position.y = 2.0
    odom.pose.pose.position.z = 0.0
    # Quaternion for 90 degree rotation around z-axis. So the robot is facing in the y-direction.
    odom.pose.pose.orientation.x = 0.0
    odom.pose.pose.orientation.y = 0.0
    odom.pose.pose.orientation.z = np.sin(np.pi/4)
    odom.pose.pose.orientation.w = np.cos(np.pi/4)
    return odom

def test_map():
    """ Create a test map. """
    map = OccupancyGrid()
    map.header.frame_id = 'map'
    map.info.resolution = 0.5
    map.info.width = 20
    map.info.height = 20
    # Position of the map origin in the odom frame.
    map.info.origin.position.x = -5.0
    map.info.origin.position.y = 5.0
    # Rotation of pi around x-axis. So the map is upside down.
    map.info.origin.orientation.w = 0.0
    map.info.origin.orientation.x = 1.0
    return map

if __name__ == '__main__':
    if len(sys.argv) > 1:
        if sys.argv[1] == "--test":
            import doctest
            doctest.testmod()
            sys.exit(0)

    rospy.init_node('Mapper')
    width = rospy.get_param("/mapper/width", 400)
    height = rospy.get_param("/mapper/height", 400)
    resolution = rospy.get_param("/mapper/resolution", 0.1) # meters per pixel

    Mapper(width, height, resolution).mapit()



