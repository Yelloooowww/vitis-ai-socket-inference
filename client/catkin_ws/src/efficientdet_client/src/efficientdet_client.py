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
		self.resize_image = np.zeros((640, 480))
		self.new_data_coming = False

		# socket connect
		connect_success = False
		while not connect_success:
			self.my_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
			try:
				self.my_socket.connect(("192.168.0.100", 5678))
				connect_success = True
			except Exception as e:
				rospy.loginfo(e)

		self.timer = rospy.Timer(rospy.Duration(0.01), self.inference)
		self.timer1 = rospy.Timer(rospy.Duration(0.01), self.process_result)

	def process_result(self,event):
		if not self.new_data_coming: return

		image_to_pub = self.resize_image
		for i in range( int(len(self.RecvBufferData)/12) ):
			xmin = self.RecvBufferData[i*12+0*2]* 2**8 + self.RecvBufferData[i*12+0*2+1]
			ymin = self.RecvBufferData[i*12+1*2]* 2**8 + self.RecvBufferData[i*12+1*2+1]
			xmax = self.RecvBufferData[i*12+2*2]* 2**8 + self.RecvBufferData[i*12+2*2+1]
			ymax = self.RecvBufferData[i*12+3*2]* 2**8 + self.RecvBufferData[i*12+3*2+1]
			label = self.RecvBufferData[i*12+4*2]* 2**8 + self.RecvBufferData[i*12+4*2+1]
			confidence = self.RecvBufferData[i*12+5*2]* 2**8 + self.RecvBufferData[i*12+5*2+1]
			# print(i+1," xmin=",xmin," ymin=",ymin," xmax=",xmax," ymax=",ymax," label=",label, " confidence=",confidence)

			# draw image for results
			text = str(label) + ": {:.2f}".format(confidence/1000)
			x1, y1, x2, y2 = (xmin, ymin, xmax, ymax)
			cv2.rectangle(image_to_pub, (x1,y1), (x2,y2), (0,255,0), 6)
			fontFace = cv2.FONT_HERSHEY_COMPLEX
			fontScale = 0.5
			thickness = 1
			labelSize = cv2.getTextSize(text, fontFace, fontScale, thickness)
			_x1 = x1 # bottomleft x of text
			_y1 = y1 # bottomleft y of text
			_x2 = x1+labelSize[0][0] # topright x of text
			_y2 = y1-labelSize[0][1] # topright y of text
			cv2.rectangle(image_to_pub, (_x1,_y1), (_x2,_y2), (0,255,0), cv2.FILLED) # text background
			cv2.putText(image_to_pub, text, (x1,y1), fontFace, fontScale, (0,0,0), thickness)

		self.image_pub.publish(self.bridge.cv2_to_imgmsg(image_to_pub,"bgr8"))
		self.new_data_coming = False



	def inference(self,event):
		if self.input_image==None: return # no image

		# cv_bridge
		try:
			cv_image = self.bridge.imgmsg_to_cv2(self.input_image, "bgr8")
		except CvBridgeError as e:
			print(e)
			return

		# send
		self.resize_image = cv2.resize(cv_image, (640, 480), interpolation=cv2.INTER_AREA)
		for i in range(0,480): #split pkg
			byte_data = np.array(self.resize_image[i], dtype='<u1').tobytes()
			self.my_socket.send(byte_data)

		# receive
		header_and_bytes = self.my_socket.recv(5)
		datasize = (header_and_bytes[3]* 2**8) + header_and_bytes[4]
		self.RecvBufferData = b''
		bytes = 0;
		i = 0;
		while i < datasize:
			newbuf = self.my_socket.recv(datasize - i)
			self.RecvBufferData += newbuf
			i += len(newbuf)
		checksum = self.my_socket.recv(1)

		#clear
		self.input_image = None
		self.new_data_coming = True

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
