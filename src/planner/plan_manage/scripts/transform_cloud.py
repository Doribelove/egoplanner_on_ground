#!/usr/bin/env python3
import rospy
import tf2_ros
import tf2_sensor_msgs.tf2_sensor_msgs as tf2_geometry
from sensor_msgs.msg import PointCloud2

class CloudTransformer:
    def __init__(self):
        rospy.init_node('cloud_transformer')
        
        # 参数配置
        self.target_frame = rospy.get_param('~target_frame', 'body')
        
        # 初始化 TF2 监听器
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        
        # 发布与订阅
        self.pub = rospy.Publisher('output', PointCloud2, queue_size=2)
        self.sub = rospy.Subscriber('input', PointCloud2, self.callback, queue_size=2)
        rospy.loginfo(f"转换节点已启动: 目标坐标系 -> {self.target_frame}")

    def callback(self, msg):
        try:
            # 核心：根据点云的时间戳，查找当时的坐标变换矩阵
            # target_frame: body, source_frame: msg.header.frame_id (camera_init)
            trans = self.tf_buffer.lookup_transform(self.target_frame, 
                                                   msg.header.frame_id, 
                                                   msg.header.stamp, 
                                                   rospy.Duration(0.1))
            
            # 使用 tf2 执行点云的空间坐标变换（不仅仅是改 frame_id）
            transformed_cloud = tf2_geometry.do_transform_cloud(msg, trans)
            self.pub.publish(transformed_cloud)
            
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logwarn_throttle(5, f"等待坐标变换: {str(e)}")

if __name__ == '__main__':
    try:
        CloudTransformer()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
