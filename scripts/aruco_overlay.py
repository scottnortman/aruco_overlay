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
import numpy as np

import rospy
import tf
import cv2
import message_filters
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, CameraInfo
from fiducial_msgs.msg import FiducialArray, FiducialTransformArray
from geometry_msgs.msg import Vector3, Transform, Quaternion



# TODO: change to Transform stamped?



class ArucoOverlayNode( object ):

	def __init__( self, name='aruco_overlay', image_in='/camera', cam_info='/camera_info', 
		fid_trans='/fiducial_transforms', fid_verts='/fiducial_vertices', rate=10.0 ):
		
		self.name = name
		self.image_in = image_in
		self.cam_info = cam_info
		self.fid_trans = fid_trans
		self.fid_verts = fid_verts
		self.rate = rate


		rospy.init_node( self.name, anonymous=False )
		rospy.loginfo( '\n'\
			'aruco_overlay node: [{}] started with parameters: \n' \
			'image_in:\t\t[\'{}\']\n' \
			'cam_info:\t\t[\'{}\']\n' \
			'fid_trans:\t\t[\'{}\']\n'\
			'fid_verts:\t\t[\'{}\']\n' \
			'image_out:\t\t[\'{}\']\n' 
			'rate:\t\t\t[{}]' \
			.format( self.name, self.image_in, self.cam_info, self.fid_trans, self.fid_verts, \
				'/annotated_image_raw', self.rate ) )


		# Filter subscriptions and time synchronize
		image_in_sub = message_filters.Subscriber( self.image_in, Image )
		cam_info_sub = message_filters.Subscriber( self.cam_info, CameraInfo )
		fid_verts_sub = message_filters.Subscriber( self.fid_verts, FiducialArray )
		fid_trans_sub = message_filters.Subscriber( self.fid_trans, FiducialTransformArray )
		time_sync = message_filters.TimeSynchronizer( \
			[image_in_sub, cam_info_sub, fid_verts_sub, fid_trans_sub], 10 )

		time_sync.registerCallback( self.msgs_callback )


		# Create a publisher
		self.publisher = rospy.Publisher( '/'+self.name+'/annotated_image_raw', Image, queue_size=10 )


		rospy.spin()

		#print( "ArucoOverlay {}".format(1) )
	def msgs_callback( self, image, cam_info, fiducialArray, fiducialTransformArray ):

		if len( fiducialArray.fiducials ) > 0 :

			for ff in range( 0,len(fiducialArray.fiducials) ):

				tx = fiducialTransformArray.transforms[ff].transform
				#tx = Transform( translation=Vector3(1,2,3),rotation=Quaternion( 1,0,0,0))


				#rospy.loginfo( '[{},{},{},{},{},{},{}]'.format( \
				#	tx.translation.x, tx.translation.y, tx.translation.z, \
				#	tx.rotation.x, tx.rotation.y, tx.rotation.z, tx.rotation.w ) )

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

				fids = [[x0,y0],[x1,y1],[x2,y2],[x3,y3]]

		
			#annotated_image = image
			#self.publisher.publish( annotated_image )

			#print( cam_info.K )

			self.publisher.publish( self.draw_aruco_3d( image, fids, tx, cam_info.K, aruco_id=fi ) )

		else :

			# No markers found, publish original image
			self.publisher.publish( image )


	def draw_aruco_3d( self, image_in, fids, tx, K, aruco_id=None ):

		# Convert image to cv2 for drawing
		bridge = CvBridge()
		image_in_cv2 = bridge.imgmsg_to_cv2( image_in, 'bgr8' )

		# Add bounding box around aruco marker
		image_in_cv2 = cv2.line( image_in_cv2, \
			( int(fids[0][0]), int(fids[0][1])), (int(fids[1][0]), int(fids[1][1])), (255,255,255), 2 )
		
		image_in_cv2 = cv2.line( image_in_cv2, \
			( int(fids[1][0]), int(fids[1][1])), (int(fids[2][0]), int(fids[2][1])), (255,255,255), 2 )
		
		image_in_cv2 = cv2.line( image_in_cv2, \
			( int(fids[2][0]), int(fids[2][1])), (int(fids[3][0]), int(fids[3][1])), (255,255,255), 2 )
		
		image_in_cv2 = cv2.line( image_in_cv2, \
			( int(fids[3][0]), int(fids[3][1])), (int(fids[0][0]), int(fids[0][1])), (255,255,255), 2 )


		# Given ROS Transform as tx.translation = Vector3 and tx.rotation = Quaternion, we
		#	need to convert into a 3x3 rotation matrix and a 3x1 translation vector; get this
		#	by first converting into a 4x4 homogeneous transform, then extracting needed parts
		tr = tf.TransformerROS()
		hgt = tr.fromTranslationRotation( ( tx.translation.x, tx.translation.y, tx.translation.z ), \
			( tx.rotation.x, tx.rotation.y, tx.rotation.z, tx.rotation.w ) )

		K = np.array(K).reshape(3,3) 

		image_in_cv2 = self.draw_axis( image_in_cv2, hgt[0:3,0:3], hgt[0:3,3], K, scale=0.1, aruco_id=aruco_id )

		return  bridge.cv2_to_imgmsg( image_in_cv2, 'bgr8' )


		# Draws a coordinate frame axis on an image; taken from
		# https://stackoverflow.com/questions/30207467/how-to-draw-3d-coordinate-axes-with-opencv-for-face-pose-estimation


	def draw_axis( self, img, R, t, K, scale, aruco_id=None ) :
		# img = cv2 image
		# R = 3x3 rotation matrix (DCM)
		# t = 3x1 translation vector
		# K = 3x3 camera intrinsics matrix 
		# rotV is a 3x1
		rotV, _ = cv2.Rodrigues(R)
		points = np.float32([[scale, 0, 0], [0, scale, 0], [0, 0, scale], [0, 0, 0]]).reshape(-1, 3)
		axisPoints, _ = cv2.projectPoints(points, rotV, t, K, (0, 0, 0, 0))
		img = cv2.line(img, tuple(axisPoints[3].ravel()), tuple(axisPoints[0].ravel()), (0,0,255), 3) #color order is B,G,R
		img = cv2.line(img, tuple(axisPoints[3].ravel()), tuple(axisPoints[1].ravel()), (0,255,0), 3)
		img = cv2.line(img, tuple(axisPoints[3].ravel()), tuple(axisPoints[2].ravel()), (255,0,0), 3)

		if aruco_id != None:
			font = cv2.FONT_HERSHEY_SIMPLEX
			loc = tuple(axisPoints[3].ravel())
			fscale = 0.5
			fc = (255,255,255)
			thk = 2
			img = cv2.putText( img, 'ID:{}'.format( aruco_id ), loc, font, fscale, fc, thk, cv2.LINE_AA )

		return img







def main( ):

	# Set up argument parser 
	parser = argparse.ArgumentParser( description="ROS node to overlay image annotations on found Aruco markers" )
	parser.add_argument( "-n", "--node_name", help="Name of this ROS node instance", type=str )
	parser.add_argument( "-i", "--image_name", help="Name of Image subscription", type=str )
	parser.add_argument( "-c", "--cam_info", help="Name of CameraInfo subscription", type=str )
	parser.add_argument( "-t", "--transforms", help="Name of FiducialTransforms subscription", type=str )
	parser.add_argument( "-v", "--vertices", help="Name of FiducialArray", type=str )


	# Note arg[0] is calling filename
	args = parser.parse_args( rospy.myargv()[1:] )

	node = ArucoOverlayNode( name=args.node_name, image_in=args.image_name, cam_info=args.cam_info,  \
		fid_trans=args.transforms, fid_verts=args.vertices ) 


	#args = rospy.myargv( argv=sys.argv )
	# Note arg[0] is calling filename
	# aruco_overlay_node = ArucoOverlayNode( name=args[1], image_in=args[2], intrinsics=args[3], \
	# 	extrinsics=args[4], fiducials=args[5] )


	# aruco_overlay_node = ArucoOverlayNode( \
	# 	image_in='/camera/infra1/image_rect_raw', \
	# 	intrinsics='/camera/infra1/camera_info')

if __name__ == '__main__':
	try:
		main()
	except rospy.ROSInterruptException, err:
		pass


