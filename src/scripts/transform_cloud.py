#!/usr/bin/env python3
import rospy
import tf2_ros
import tf2_sensor_msgs
from sensor_msgs.msg import PointCloud2

class CloudTransformer:
    def __init__(self):
        self.target_frame = rospy.get_param('~target_frame', 'body')
        self.source_frame = rospy.get_param('~source_frame', 'camera_init')
        self.input_topic = rospy.get_param('~input_topic', '/cloud_registered')
        self.output_topic = rospy.get_param('~output_topic', '/cloud_body')
        
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        
        self.pub = rospy.Publisher(self.output_topic, PointCloud2, queue_size=5)
        self.sub = rospy.Subscriber(self.input_topic, PointCloud2, self.callback)
        
    def callback(self, msg):
        try:
            # 等待变换可用
            if self.tf_buffer.can_transform(self.target_frame, msg.header.frame_id, msg.header.stamp, rospy.Duration(0.1)):
                transform = self.tf_buffer.lookup_transform(self.target_frame, msg.header.frame_id, msg.header.stamp)
                transformed = tf2_sensor_msgs.do_transform_cloud(msg, transform)
                self.pub.publish(transformed)
            else:
                rospy.logwarn_throttle(5.0, "Transform from %s to %s not available", msg.header.frame_id, self.target_frame)
        except Exception as e:
            rospy.logerr("Transform failed: %s", e)

if __name__ == '__main__':
    rospy.init_node('cloud_transformer')
    ct = CloudTransformer()
    rospy.spin()
