import numpy
import rosbag
import cv2
import os
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from cv_bridge import CvBridgeError
rgb_path = '/path/to/save/re-edit-bag/rgb/'# absolute path of extracted rgb images
depth_path = '/home/wty/3/body_depth/'# absolute path of extracted depth images
bridge = CvBridge()
with rosbag.Bag('3.bag', 'r') as bag:
    for topic,msg,t in bag.read_messages():
        if topic == "/camera/aligned_depth_to_color/image_raw": 
            cv_image = bridge.imgmsg_to_cv2(msg, '32FC1')
            cv_image = cv_image * 255  # 不知为何转化的深度图显示不出来。将其乘以 255 才能看到显示效果.

            timestr = "%.6f" %  msg.header.stamp.to_sec() # 时间戳命名
            image_name = timestr + '.png'# an extension is necessary
            # image_name = str(num) + '.png'# 编号命名
            cv2.imwrite(depth_path + image_name, cv_image)  
            # 实际应用可直接保存为 numpy array
            # np.save(depth_path + image_name, cv_image)  
            print(depth_path + image_name)
        if topic == "/rgb/image_raw": 
            cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")
            timestr = "%.6f" %  msg.header.stamp.to_sec()
            image_name = str(num) + '.png'
            cv2.imwrite(rgb_path + image_name, cv_image)
