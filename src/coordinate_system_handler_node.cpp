//============================================================================
// Name        : coordinate_system_handler.cpp
// Author      : Fredrik Macintosh
// Version     : 1.0
// Copyright   : OpenSource, but plz mention me Senpai. No commercial use without written permission from author.
// Info        : A program that handles transformation of coordinate systems, from robot to world. It also inserts markers in the world so
// that it is possible to get a better view of what is going on "behind the scenes."
//============================================================================

#include "ros/ros.h"
#include "tf/transform_listener.h"
#include <iostream>
#include "visualization_msgs/Marker.h"
#include "geometry_msgs/Pose.h"
#include <eigen3/Eigen/Dense>
#include <math.h>

#define PI 3.14159265


tf::StampedTransform transform;
visualization_msgs::Marker tagPoseMarker;
geometry_msgs::Pose tagPose;
geometry_msgs::Pose robPose;
visualization_msgs::Marker box;

// function that handles the position of the robot (kinect being the centerpoint)
Eigen::Matrix4f getHomTrans(const tf::StampedTransform transform)
{
	Eigen::Matrix4f mTrans(4,4); 	// create the translation matrix
	Eigen::Matrix4f mRotZ(4,4); 	// create the z-rotation matrix
	Eigen::Matrix4f mRotX(4,4);		// create the x-rotation matrix
	Eigen::Matrix4f homTrans(4,4);	// create the homogeneous transformation matrix, the one that translates points from robot coordinate to map.
	double rotZ = transform.getRotation().z(); // this will simplify things further down, trust me.
	double rotX = -90*PI/180;

	// fill the translation matrix with the robots position
	mTrans(0, 3) = transform.getOrigin().x();
	mTrans(1, 3) = transform.getOrigin().y();
	mTrans(2, 3) = transform.getOrigin().x();
	// end of filling robot position

	// fill the diagonal with ones
	mTrans(0, 0) = 1;
	mTrans(1, 1) = 1;
	mTrans(2, 2) = 1;
	mTrans(3, 3) = 1;
	// end of filling diagonal

	// fill the z-rotation matrix with the robots Z-rotation transform.getRotation().y(); //
	mRotZ(0, 0) = cos(rotZ*PI/180); // told you it would simplify...
	mRotZ(0, 1) = -sin(rotZ*PI/180);
	mRotZ(1, 0) = sin(rotZ*PI/180);
	mRotZ(1, 1) = cos(rotZ*PI/180);
	// end of filling matrix

	// fill the rest of the diagonal with ones
	mRotZ(2, 2) = 1;
	mRotZ(3, 3) = 1;
	// end of filling with ones

	// fill the x-rotation matrix with 90 deg rotation
	mRotX(2, 2) = cos(rotX);
	mRotX(1, 1) = cos(rotX);
	mRotX(2, 1) = sin(rotX);
	mRotX(1, 2) = -sin(rotX);
	// end of filling with 90 deg

	// fill the rest of the diagonal with ones
	mRotX(0, 0) = 1;
	mRotX(3, 3) = 1;
	// end of filling diagonal

	// create the homogenous transformation matrix
	homTrans = mTrans * mRotZ * mRotX;

	return homTrans;


} // end of getHomTrans();


// creates the Marker from the tag pose
void tagPoseToMarkerCallback(geometry_msgs::Pose tagPose)
{

	box.header.frame_id = "/map";
	box.header.stamp = ros::Time();
	box.ns = "box";
	box.id = 0;
	box.type = visualization_msgs::Marker::CUBE;
	box.action = visualization_msgs::Marker::ADD;
	box.scale.x = 0.5;
	box.scale.y = 0.5;
	box.scale.z = 0.5;
	box.pose.position.x = tagPose.position.x;
	box.pose.position.y = tagPose.position.y;
	box.pose.position.z = tagPose.position.z;
	box.pose.orientation.x = tagPose.orientation.x;
	box.pose.orientation.y = tagPose.orientation.y;
	box.pose.orientation.z = tagPose.orientation.z;
	box.pose.orientation.w = tagPose.orientation.w;
	box.color.a = 1.0;
	box.color.r = 255; // mangenta 4 days
	box.color.g = 20;
	box.color.b = 147;




} // end of tagPoseListenerCallback()




int main(int argc, char **argv)
{
	ros::init(argc, argv,"tagPoseListener");  // initializing ROS
	ros::NodeHandle tagPoseSubNodehandle;  // creating the nodehandle for the sub
	ros::NodeHandle markerPubNodehandle;

	tf::TransformListener mapToKinectListener; // creating the transform to put data from the transform listener
	ros::Subscriber tagSub = tagPoseSubNodehandle.subscribe("tag_pose", 1000, tagPoseToMarkerCallback); // creating
	// subscriber for the tag_pose (later an actual box. hopefully)
	ros::Publisher boxPublisher = markerPubNodehandle.advertise<visualization_msgs::Marker>("tag_marker", 1000);

	ros::Rate rate(10.0); // loop rate for callback in Hz.

	while (ros::ok())  // while ROS is running
	{
		try
		{
			mapToKinectListener.lookupTransform("/map", "/camera_link", ros::Time(0), transform); // get data from transform
			// between /map and /camera_link frames
		}
		catch (tf::TransformException ex){ // no clue, some exception thingy that I didn't write.
			ROS_ERROR("%s",ex.what());
			ros::Duration(1.0).sleep();
		}

		Eigen::Matrix4f homTrans = getHomTrans(transform); 	// send the transform to my awesome function that creates the homogenous transformation matrix

		boxPublisher.publish(box);

		// THIS IS WHERE PLENTY ASESOME CODE NEEDS TO BE WRITTEN, GET TO IT.

		ros::spinOnce();  // pump ONE callback
		rate.sleep();  // make sure the rate of 10Hz is maintained (muy importante)
	}// end of while(ros::ok())



	return 0;
}// end of main()
