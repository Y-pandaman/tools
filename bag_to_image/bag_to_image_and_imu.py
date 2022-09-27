# coding:utf-8
import roslib
import rosbag
import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from cv_bridge import CvBridgeError

path1 = '/home/bdca/桌面/corridor4/cam0/'  # 存放图片的位置
path2 = '/home/bdca/桌面/corridor4/cam1/'


class ImageCreator():

    def __init__(self):
        self.bridge = CvBridge()
        with rosbag.Bag('dataset-corridor4_1024_16.bag', 'r') as bag:  # 要读取的bag文件；
            for topic, msg, t in bag.read_messages():
                if topic == "/cam0/image_raw":  # 图像的topic；
                    try:
                        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
                    except CvBridgeError as e:
                        print e
                    timeq = msg.header.stamp.to_sec()
                    timestr = "{:.6f}".format(timeq)
                    # %.6f表示小数点后带有6位，可根据精确度需要修改；"%.6f" %  msg.header.stamp.to_sec() * pow(10, 9)
                    image_name = timestr + ".png"  # 图像命名：时间戳.png
                    cv2.imwrite(path1+image_name, cv_image)  # 保存；
                elif topic == "/cam1/image_raw":  # 图像的topic；
                    try:
                        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
                    except CvBridgeError as e:
                        print e
                    timeq = msg.header.stamp.to_sec()
                    timestr = "{:.6f}".format(timeq)
                    # %.6f表示小数点后带有6位，可根据精确度需要修改；"%.6f" %  msg.header.stamp.to_sec() * pow(10, 9)
                    image_name = timestr + ".png"  # 图像命名：时间戳.png
                    cv2.imwrite(path2+image_name, cv_image)  # 保存；
            imu = open("imu.txt", 'w')
            for topic, msg, t in bag.read_messages():
                if topic == "/imu0":  # imu topic；
                    acc_y = "%.6f" % msg.linear_acceleration.y
                    acc_x = "%.6f" % msg.linear_acceleration.x
                    acc_z = "%.6f" % msg.linear_acceleration.z
                    w_y = "%.6f" % msg.angular_velocity.y
                    w_x = "%.6f" % msg.angular_velocity.x
                    w_z = "%.6f" % msg.angular_velocity.z
                    timeimu = "%.6f" % msg.header.stamp.to_sec()
                    imudata = timeimu + " " + w_x + " " + w_y + " " + \
                        w_z + " " + acc_x + " " + acc_y + " " + acc_z
                    imu.write(imudata)
                    imu.write('\n')
            imu.close()


if __name__ == '__main__':

    try:
        image_creator = ImageCreator()
    except rospy.ROSInterruptException:
        pass
