#! /usr/bin/env python
import rospy
from std_msgs.msg import String,Int32,Int16MultiArray
import cv2
import numpy as np
import cv2  # OpenCV module
import time
from sensor_msgs.msg import Image

rospy.init_node('cam_read', anonymous=True)
image_raw_pub = rospy.Publisher("image_raw", Image, queue_size=1)
# 選擇第二隻攝影機
cap = cv2.VideoCapture(0)

while(True):
# 從攝影機擷取一張影像
	ret, frame = cap.read()
	data = cv2.resize(frame, (640, 480), interpolation=cv2.INTER_AREA)

	msg = Image()
	msg.header.stamp = rospy.Time.now()
	msg.width        = 640
	msg.height       = 480
	channel = 3
	msg.encoding = "bgr8"
	msg.is_bigendian = 1
	msg.step = 640 * channel
	msg.data = data.tostring()
	image_raw_pub.publish(msg)
	rospy.loginfo("image_raw_pub")


# 釋放攝影機
cap.release()

# 關閉所有 OpenCV 視窗
cv2.destroyAllWindows()
