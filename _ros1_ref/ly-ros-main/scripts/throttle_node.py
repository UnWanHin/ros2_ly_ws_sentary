#!/usr/bin/env python
from auto_aim_common.msg._AngleImage import AngleImage
import rospy
from sensor_msgs.msg import Image

class ThrottleNode:
    def __init__(self):
        rospy.init_node('image_throttle_node')
        
        # 创建1Hz的发布器
        self.pub = rospy.Publisher('/ly/ra/angle_image_throttled', Image, queue_size=10)
        
        # 订阅原始图像话题
        rospy.Subscriber('/ly/ra/angle_image', AngleImage, self.image_callback)
        
        # 设置1Hz的发布频率
        self.rate = rospy.Rate(2)
        self.latest_msg = None
        
        rospy.loginfo("图像转发节点已启动 (1Hz)")
        
    def image_callback(self, msg):
        # 只保存最新图像，不立即发布
        self.latest_msg = msg.image
        
    def run(self):
        while not rospy.is_shutdown():
            if self.latest_msg:
                # 添加时间戳标识
                self.latest_msg.header.stamp = rospy.Time.now()
                self.pub.publish(self.latest_msg)
                rospy.loginfo_once("已转发第一帧图像 (1Hz)")
            self.rate.sleep()

if __name__ == '__main__':
    try:
        node = ThrottleNode()
        node.run()
    except rospy.ROSInterruptException:
        pass