#!/usr/bin/env python
#------------------------------------------------------------------------------
# 	File:	aruco_overlay.py
#	Desc:	ROS node which reads in an image message stream and overlays a 
#			real-time coordinate frame on top of an Aruco marker (if present).
#	Date:	Initiated Dec. 2019
#	Auth:	Scott Nortman, scott@mainstreamrobotics.com
#	Note:
#			Use aruco_detect, http://wiki.ros.org/aruco_detect 
#			Markers can be generated from http://chev.me/arucogen/
#
#			Prerequisite:  
#				The camera used to stream the input image must
#				also have a valid intrinsic camera calibration, see
#				http://wiki.ros.org/camera_calibration
#				
#
#			Subscribes:
#				/camera => sensor_msgs/Image, contains the raw image with a 
#					possible Aruco
#				/camera_info =>  sensor_msgs/CameraInfo, intrinsic calibration 
#					parameters of camera used to produce the above image data
#				/fiducial_verticies => fiducial_msgs/Fiducials, verticies of 
#					detected fiducials
#
#			Publishes:
#				/annotated_image => sensor_msgs/Image, contains a copy of the
#					input image with added annotations
#
#			Example usage:
#
#				$ roslaunch realsense2_camera rs_rgbd.launch \
#					infra_width:=1280 infra_height:=720
#				$ roslaunch aruco_detect aruco_detect.launch camera:="camera/infra1" \
#						image:="image_rect_raw" fiducial_len:="0.1" dictionary:="0"
#				$ roslaunch aruco_overlay launch.launch
#				$ rosrun image_view image_view image:=/camera/image
#
#------------------------------------------------------------------------------

from __future__ import print_function
import sys
import argparse
import Queue

import rospy
import cv2
import message_filters
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, CameraInfo
from fiducial_msgs.msg import FiducialArray







class ArucoOverlayNode( object ):

	def __init__( self, name='aruco_overlay', image_in='/camera', intrinsics='/camera_info', 
		fiducials='/fiducial_vertices', rate=10.0 ):
		
		self.name = name
		self.image_in = image_in
		self.intrinsics = intrinsics
		self.fiducials = fiducials
		self.rate = rate


		rospy.init_node( self.name, anonymous=False, )
		rospy.loginfo( '\n'\
			'aruco_overlay node: [{}] started with parameters: \n' \
			'image_in:\t\t[\'{}\']\n' \
			'intrinsics:\t\t[\'{}\']\n' \
			'fiducials:\t\t[\'{}\']\n' \
			'image_out:\t\t[\'{}\']\n' 
			'rate:\t\t\t[{}]' \
			.format( self.name, self.image_in, self.intrinsics, self.fiducials, \
				'/annotated_image_raw', self.rate ) )


		# Filter subscriptions and time synchronize
		image_in_sub = message_filters.Subscriber( self.image_in, Image )
		intrinsics_sub = message_filters.Subscriber( self.intrinsics, CameraInfo )
		fiducials_sub = message_filters.Subscriber( self.fiducials, FiducialArray )
		time_sync = message_filters.TimeSynchronizer( \
			[image_in_sub, intrinsics_sub, fiducials_sub], 10 )

		time_sync.registerCallback( self.msgs_callback )


		# Create a publisher
		self.publisher = rospy.Publisher( '/'+self.name+'/annotated_image_raw', Image, queue_size=10 )


		rospy.spin()

		#print( "ArucoOverlay {}".format(1) )
	def msgs_callback( self, image, intrinsics, fiducialArray ):

		rospy.loginfo('msgs_callback_test')

		if len( fiducialArray.fiducials ) > 0 :

			for ff in range( 0,len(fiducialArray.fiducials) ):

				fi = fiducialArray.fiducials[ff].fiducial_id
				dd = fiducialArray.fiducials[ff].direction
				x0 = fiducialArray.fiducials[ff].x0
				y0 = fiducialArray.fiducials[ff].y0
				x1 = fiducialArray.fiducials[ff].x1
				y1 = fiducialArray.fiducials[ff].y1
				x2 = fiducialArray.fiducials[ff].x2
				y2 = fiducialArray.fiducials[ff].y2
				x3 = fiducialArray.fiducials[ff].x3
				y3 = fiducialArray.fiducials[ff].y3

				rospy.logwarn( '[{},{}], [{},{}], [{},{}], [{},{}]'.format(x0,y0,x1,y1,x2,y2,x3,y3) )

				fids = [[x0,y0],[x1,y1],[x2,y2],[x3,y3]]

		
			#annotated_image = image
			#self.publisher.publish( annotated_image )

			self.publisher.publish( self.draw_aruco_3d( image, fids ) )

		# do some stuff

	def draw_aruco_3d( self, image_in, fids ):

		# Conver image to cv2 for drawing
		bridge = CvBridge()
		image_in_cv2 = bridge.imgmsg_to_cv2( image_in, 'bgr8' )
		image_in_cv2 = cv2.line( image_in_cv2, \
			( int(fids[0][0]), int(fids[0][1])), (int(fids[1][0]), int(fids[1][1])), (0,0,255), 2 )

		return  bridge.cv2_to_imgmsg( image_in_cv2, 'bgr8' )









def main():
	aruco_overlay_node = ArucoOverlayNode( image_in='/camera/infra1/image_rect_raw', intrinsics='/camera/infra1/camera_info')

if __name__ == '__main__':
	try:
		main()
	except rospy.ROSInterruptException:
		pass


