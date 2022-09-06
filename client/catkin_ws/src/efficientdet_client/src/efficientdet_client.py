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

		connect_success = False
		while not connect_success:
			# socket re-connect
			self.my_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
			try:
				self.my_socket.connect(("192.168.0.100", 5678))
				connect_success = True
			except Exception as e:
				rospy.loginfo(e)

		self.timer = rospy.Timer(rospy.Duration(0.1), self.inference)


	def inference(self,event):
		if self.input_image==None: return # no image

		# cv_bridge
		try:
			cv_image = self.bridge.imgmsg_to_cv2(self.input_image, "bgr8")
		except CvBridgeError as e:
			print(e)
			return

		# send
		resize_image = cv2.resize(cv_image, (640, 480), interpolation=cv2.INTER_AREA)
		for i in range(0,480): #split pkg
			byte_data = np.array(resize_image[i], dtype='<u1').tobytes()
			self.my_socket.send(byte_data)

		# receive
		header_and_bytes = self.my_socket.recv(5)
		datasize = (header_and_bytes[3]* 2**8) + header_and_bytes[4]
		# print("datasize=",datasize)
		bufferData = b''
		bytes = 0;
		i = 0;
		while i < datasize:
			newbuf = self.my_socket.recv(datasize - i)
			bufferData += newbuf
			i += len(newbuf)
		checksum = self.my_socket.recv(1)

		for i in range( int(datasize/12) ):
			print(i,end=' ')
			for j,item in enumerate(['xmin','ymin','xmax','ymax','label','confidence']):
				value = bufferData[i*12+j*2]* 2**8 + bufferData[i*12+j*2+1]
				print(item,"=",value,end=' ')
			print('')

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
