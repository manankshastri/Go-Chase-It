#include "ros/ros.h"
#include "ball_chaser/DriveToTarget.h"
#include <sensor_msgs/Image.h>
#include <string>

// Define a global client that can request services
ros::ServiceClient client;

// This function calls the command_robot service to drive the robot in the specified direction
void drive_robot(float lin_x, float ang_z) {
	
  // Request a service and pass the velocities to it to drive the robot
	ROS_INFO_STREAM("ATTEMPTING TO MOVE THE ROBOT...");
	
	ball_chaser::DriveToTarget srv;
	srv.request.linear_x = lin_x;
	srv.request.angular_z = ang_z;
	
	// Call the /ball_chaser/command_robot service and pass the velocities
	if (!client.call(srv))
        ROS_ERROR("FAILED TO CALL THE SERVICE /ball_chaser/command_robot......");
}

// This callback function continuously executes and reads the image data	
void process_image_callback(const sensor_msgs::Image img) {

    int whitePixel = 255;
    
    // TODO: Loop through each pixel in the image and check if there's a bright white one
    // Then, identify if this pixel falls in the left, mid, or right side of the image
    // Depending on the white ball position, call the drive_bot function and pass velocities to it
    // Request a stop when there's no white ball seen by the camera
	
	
	/* 
		Counting the number of white pixels: if they are in the left/middle/right side of the image
		Computing maximum white pixel count; moving the robot in the direction related to this max count. 
		An additional string variable for status info 
		
		Reference: 
		https://knowledge.udacity.com/questions/368996
	*/
	
	int leftSide = int(img.width / 3);
	int rightSide = 2 * leftSide;
	
	int leftCount = 0, middleCount = 0, rightCount = 0;
	
	for (int i = 0; i < img.height * img.step; i += 3) {	
	
		if (img.data[i] == whitePixel && img.data[i+1] == whitePixel && img.data[i+2] == whitePixel) {

			auto checkPixel = (i % img.step) / 3;
			
			// added seperate if else-if for each side, instead of putting middle side count to else. 
			if (checkPixel < leftSide)
				leftCount += 1;
			
			else if (checkPixel >= leftSide && checkPixel <= rightSide)
				middleCount += 1 ;
			
			else if (checkPixel > rightSide)
				rightCount += 1;
		}
	}
	
	std::string robotMovingSide;
	int maxCount = std::max(std::max(leftCount, middleCount), rightCount);
	
	if (maxCount == 0) {
		robotMovingSide = "Robot Not Moving...";
		drive_robot(0.0, 0.0);
	}
	
	else if (maxCount == leftCount) {
		robotMovingSide = "Robot Moving Left...";
		drive_robot(0.0, 0.5);
	}
	
	else if (maxCount == middleCount) {
		robotMovingSide = "Robot Moving Forward...";
		drive_robot(0.5, 0.0);
	}
	
	else if (maxCount == rightCount) {
		robotMovingSide = "Robot Moving Right...";
		drive_robot(0.0, -0.5);
	}
	
	ROS_INFO("Status: %s", robotMovingSide.c_str());

}

int main(int argc, char** argv) {
	// Initialize the process_image node and create a handle to it
	ros::init(argc, argv, "process_image");
	ros::NodeHandle n;

	// Define a client service capable of requesting services from command_robot
	client = n.serviceClient<ball_chaser::DriveToTarget>("/ball_chaser/command_robot");

	// Subscribe to /camera/rgb/image_raw topic to read the image data inside the process_image_callback function
	ros::Subscriber sub1 = n.subscribe("/camera/rgb/image_raw", 10, process_image_callback);

	// Handle ROS communication events
	ros::spin();

	return 0;
}

