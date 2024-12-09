#!/usr/bin/env python3

# =============================================================================
# TF ROS Topic Re-Publisher
# Created by: Shaun Altmann
# =============================================================================
'''
TF ROS Topic Re-Publisher
-
Creates a ROS node that re-publishes /tf and /tf_static topic data.
'''
# =============================================================================

# =============================================================================
# Imports
# =============================================================================

# used for logging
import logging

# used for ros
import rospy # type: ignore

# used for topic messages
from tf2_msgs.msg import TFMessage # type: ignore


# =============================================================================
# TF Re-Publisher Node Definition
# =============================================================================
class TF_RePublisher(object):
    '''
    TF Re-Publisher Node
    -
    Contains the publishers and subscribers to re-publish tf data.

    Fields
    -
    - _namespace : `str`
    - _pub_tf : `rospy.Publisher`
    - _pub_tf_static : `rospy.Publisher`

    Methods
    -
    - __init__() : `None`
    - _callback_tf(msg) : `None`
    - _callback_tf_static(msg) : `None`
    - log(msg, lvl=logging.INFO) : `None` << static >>
    - spin() : `None`
    '''

    # ====================
    # Method - Constructor
    def __init__(self) -> None:
        '''
        TF Re-Publisher Node Constructor
        -
        Creates a new `TF_RePublisher` node.

        Parameters
        -
        None

        Returns
        -
        None
        '''

        # initialize node
        rospy.init_node('tf_republisher', anonymous = True)

        # set fields
        self._namespace = str(rospy.get_namespace()).strip('/')
        ''' Namespace to republish the data in. '''
        self._pub_tf = rospy.Publisher(
            f'{self._namespace}/tf',
            TFMessage,
            queue_size = 100
        )
        ''' Publisher for the `"/tf"` topic. '''
        self._pub_tf_static = rospy.Publisher(
            f'{self._namespace}/tf_static',
            TFMessage,
            queue_size = 100
        )
        ''' Publisher for the `"/tf_static"` topic. '''

        # subscribe to topics
        rospy.Subscriber(
            'tf',
            TFMessage,
            self._callback_tf
        )
        rospy.Subscriber(
            'slamware_ros_sdk_server_node/odom',
            TFMessage,
            self._callback_tf_static
        )

        # log node construction
        self.log('TF Re-Publisher Node Constructed')


    # =======================================
    # Method - Subscriber Callback - TF Topic
    def _callback_tf(self, msg: TFMessage) -> None:
        '''
        Subscriber Callback - TF Topic
        -
        Callback method for the Slamware Lidar `"/tf"` topic.

        Parameters
        -
        - msg : `TFMessage`
            - Message data from the ros topic.

        Returns
        -
        None
        '''

        self.log('Relaying TF Data', logging.DEBUG)

        # for some reason, some of the transforms are published at 1.6e9
        # instead of 1.7e9, so this fixes the timestamp of the old messages
        republish = False
        for tf in msg.transforms:
            if str(tf.header.stamp.secs).startswith('16'):
                republish = True
                msg.stamp = rospy.Time.now()
        if republish:
            self._pub_tf.publish(msg)

    # ==============================================
    # Method - Subscriber Callback - Static TF Topic
    def _callback_tf_static(self, msg: TFMessage) -> None:
        '''
        Subscriber Callback - Static TF Topic
        -
        Callback method for the Slamware Lidar `"/tf_static"` topic.

        Parameters
        -
        - msg : `TFMessage`
            - Message data from the ros topic.

        Returns
        -
        None
        '''

        self.log('Relaying Static TF Data', logging.DEBUG)

        # for some reason, some of the transforms are published at 1.6e9
        # instead of 1.7e9, so this fixes the timestamp of the old messages
        republish = False
        for tf in msg.transforms:
            if str(tf.header.stamp.secs).startswith('16'):
                republish = True
                msg.stamp = rospy.Time.now()
        if republish:
            self._pub_tf_static.publish(msg)

    # ====================
    # Method - Log Message
    @staticmethod
    def log(msg: str, lvl: int = logging.INFO) -> None:
        '''
        Log Message
        -
        Logs a message to the ROS console with a specified level.

        Parameters
        -
        - msg : `str`
            - Message to log.
        - lvl : `int`
            - Logging level to use. Defaults to `logging.INFO`.

        Returns
        -
        None
        '''

        # re-format message to include node name
        msg = f'{rospy.get_caller_id()}: {msg}'

        if lvl == logging.DEBUG: rospy.logdebug(msg)
        elif lvl == logging.INFO: rospy.loginfo(msg)
        elif lvl == logging.WARNING: rospy.logwarn(msg)
        elif lvl == logging.ERROR: rospy.logerr(msg)
        elif lvl == logging.FATAL: rospy.logfatal(msg)
        else:
            raise ValueError(
                f'TF_RePublisher.log(msg = {msg!r}, lvl = {lvl!r}) received ' \
                + f'invalid lvl value {lvl!r}'
            )

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
        node = TF_RePublisher()
        node.spin()
    except rospy.ROSInterruptException:
        pass


# =============================================================================
# End of File
# =============================================================================
