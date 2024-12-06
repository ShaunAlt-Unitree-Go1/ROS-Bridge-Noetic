#!/usr/bin/env python3

# =============================================================================
# Lidar ROS Topic Relay
# Created by: Shaun Altmann
# =============================================================================
'''
Lidar ROS Topic Relay
-
Creates a ROS node that gets the Slamware Mapper Lidar data and converts it
into standardized ROS topics for SLAM and NAV.
'''
# =============================================================================

# =============================================================================
# Imports
# =============================================================================

# used for ros
import rospy # type: ignore

# used for topic messages
from nav_msgs.msg import Odometry, OccupancyGrid # type: ignore
from sensor_msgs.msg import LaserScan # type: ignore


# =============================================================================
# Lidar Relay Node Definition
# =============================================================================
class LidarRelay(object):
    '''
    Lidar Relay Node
    -
    Contains the publishers and subscribers to relay lidar data through the
    relevant topics.

    Fields
    -
    - _namespace : `str`
    - _pub_map : `rospy.Publisher`
    - _pub_odom : `rospy.Publisher`
    - _pub_scan : `rospy.Publisher`

    Methods
    -
    - __init__() : `None`
    - _callback_map(msg) : `None`
    - _callback_odom(msg) : `None`
    - _callback_scan(msg) : `None`
    - spin() : `None`
    '''

    # ====================
    # Method - Constructor
    def __init__(self) -> None:
        '''
        Lidar Relay Node Constructor
        -
        Creates a new `LidarRelay` node.

        Parameters
        -
        None

        Returns
        -
        None
        '''

        # initialize node
        rospy.init_node('lidar_relay', anonymous = True)

        # set fields
        self._namespace = str(rospy.get_namespace()).strip('/')
        ''' Namespace to republish the data in. '''
        self._pub_map = rospy.Publisher(
            f'{self._namespace}/map',
            OccupancyGrid,
            queue_size = 10
        )
        ''' Publisher for the `"/map"` topic. '''
        self._pub_odom = rospy.Publisher(
            f'{self._namespace}/odom',
            Odometry,
            queue_size = 10
        )
        ''' Publisher for the `"/odom"` topic. '''
        self._pub_scan = rospy.Publisher(
            f'{self._namespace}/scan',
            LaserScan,
            queue_size = 10
        )
        ''' Publisher for the `"/scan"` topic. '''

        # subscribe to topics
        rospy.Subscriber(
            'slamware_ros_sdk_server_node/map',
            OccupancyGrid,
            self._callback_map
        )
        rospy.Subscriber(
            'slamware_ros_sdk_server_node/odom',
            Odometry,
            self._callback_odom
        )
        rospy.Subscriber(
            'slamware_ros_sdk_server_node/scan',
            LaserScan,
            self._callback_scan
        )

    # ========================================
    # Method - Subscriber Callback - Map Topic
    def _callback_map(self, msg: OccupancyGrid) -> None:
        '''
        Subscriber Callback - Map Topic
        -
        Callback method for the Slamware Lidar `"/map"` topic.

        Parameters
        -
        - msg : `OccupancyGrid`
            - Message data from the ros topic.

        Returns
        -
        None
        '''

        self._pub_map.publish(msg)

    # =========================================
    # Method - Subscriber Callback - Odom Topic
    def _callback_odom(self, msg: Odometry) -> None:
        '''
        Subscriber Callback - Odom Topic
        -
        Callback method for the Slamware Lidar `"/odom"` topic.

        Parameters
        -
        - msg : `Odometry`
            - Message data from the ros topic.

        Returns
        -
        None
        '''

        self._pub_odom.publish(msg)

    # =========================================
    # Method - Subscriber Callback - Scan Topic
    def _callback_scan(self, msg: LaserScan) -> None:
        '''
        Subscriber Callback - Scan Topic
        -
        Callback method for the Slamware Lidar `"/scan"` topic.

        Parameters
        -
        - msg : `LaserScan`
            - Message data from the ros topic.

        Returns
        -
        None
        '''

        self._pub_scan.publish(msg)

    # ==================
    # Method - Spin Node
    def spin(self) -> None:
        '''
        Spin Node
        -
        Starts the ROS node spinning.

        Parameters
        -
        None

        Returns
        -
        None
        '''

        rospy.spin()


# =============================================================================
# Main Loop
# =============================================================================
if __name__ == '__main__':
    try:
        node = LidarRelay()
        node.spin()
    except rospy.ROSInterruptException:
        pass


# =============================================================================
# End of File
# =============================================================================
