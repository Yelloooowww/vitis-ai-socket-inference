#! /usr/bin/env python3
import cv2
import socket
import rospy
from std_msgs.msg import String,Int32,Int16MultiArray
import numpy as np
import cv2  # OpenCV module
import time
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
class EfficientDetSocketClient():
	def __init__(self):


		self.bridge = CvBridge()

		self.image_sub = rospy.Subscriber("image_raw", Image, self.img_cb)
		self.image_pub = rospy.Publisher("EfficientDet_result", Image, queue_size=1)

		self.input_image = None
		self.timer = rospy.Timer(rospy.Duration(0.1), self.inference)


	def inference(self,event):
		if self.input_image==None: return

		try:
			cv_image = self.bridge.imgmsg_to_cv2(self.input_image, "bgr8")
		except CvBridgeError as e:
			print(e)
			return

		# socket re-connect
		self.my_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
		try:
			self.my_socket.connect(("192.168.0.100", 5678))
			connect_success = True
		except Exception as e:
			rospy.loginfo(e)
			return

		# send
		resize_image = cv2.resize(cv_image, (640, 480), interpolation=cv2.INTER_AREA)
		for i in range(0,480): #split pkg
			byte_data = np.array(resize_image[i], dtype='<u1').tobytes()
			self.my_socket.send(byte_data)
		print("send")

		# receive
		bufferData = b''
		imgSize = 3 * 640 * 480
		bytes = 0;
		i = 0;
		while i < imgSize:
			newbuf = self.my_socket.recv(imgSize - i)
			bufferData += newbuf
			i += len(newbuf)
		print("recv")
		self.my_socket.close()
		self.input_image = None

		recv_img = []
		for i in range(0,480):
			tmp = []
			for j in range(0,640):
				tmp.append([bufferData[i*640*3 + j*3 + 0],\
							bufferData[i*640*3 + j*3 + 1],\
							bufferData[i*640*3 + j*3 + 2]])

			recv_img.append(tmp)
		recv_img = np.array(recv_img,dtype=np.uint8)
		self.image_pub.publish(self.bridge.cv2_to_imgmsg(recv_img,"bgr8"))



	def img_cb(self, msg):
		self.input_image = msg


	def onShutdown(self):
		self.my_socket.close()
		rospy.loginfo("Shutdown.")


if __name__ == '__main__':
	rospy.init_node('EfficientDetSocketClient')
	foo = EfficientDetSocketClient()
	rospy.on_shutdown(foo.onShutdown)
	rospy.spin()
