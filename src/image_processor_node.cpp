#include "cyberpod_sim_ros/image_processor_node.hpp" 

using namespace Eigen;

ros::NodeHandle *nh_;
ros::NodeHandle *nhParams_;
ros::Subscriber sub_camera_feed_;
ros::Subscriber sub_cmd_;
ros::Publisher pub_input_;

cyberpod_sim_ros::input input_;
cyberpod_sim_ros::cmd cmdCurrent_;
sensor_msgs::Image imageCurrent_;

// Callback Functions

void imageCallback(const sensor_msgs::Image::ConstPtr msg)
{

}

void cmdCallback(const cyberpod_sim_ros::cmd::ConstPtr msg)
{
	cmdCurrent_ = *msg;
}

// Global variables


// Controller gains


int main (int argc, char *argv[])
{
	// Init ros
	ros::init(argc,argv,"image_processor");

	// Instanciate NodeHandles
	nhParams_ = new ros::NodeHandle("~");
	nh_ = new ros::NodeHandle();

	// Initialize variables

	// Init pubs, subs and srvs
	sub_camera_feed_ = nh_->subscribe<sensor_msgs::Image>("/camera1/camera_feed", 1, imageCallback);
	sub_cmd_ = nh_->subscribe<cyberpod_sim_ros::cmd>("cmd", 1, cmdCallback);
	pub_input_ = nh_->advertise<cyberpod_sim_ros::input>("inputDes", 1);

	// Retreive params
	/*
	nhParams_->param<double>("offset_angle",offset_angle_,0.);
	nhParams_->param<std::vector<double>>("gains",gains_,gains_);
	if(gains_.size()!=STATE_LENGTH)
	{
		gains_ = std::vector<double>(STATE_LENGTH,0.0);
		ROS_WARN("gains must be of length %u",STATE_LENGTH);
	}
	for(uint32_t i=0; i<STATE_LENGTH; i++)
		gainsVec_(i) = gains_[i];
	*/

	// Display node info
	ROS_INFO("Image Processor node successfuly");
	/* ROS_INFO("___offset_angle=%f",offset_angle_);
	ROS_INFO("___gains=");
	for(uint8_t i = 0; i<STATE_LENGTH; i++)
	{
		ROS_INFO("      %.3f",gains_[i]);
	}
	*/

	// Take it for a spin
	while(ros::ok())
		ros::spinOnce(); // this looks like there's something wrong... where's the code block? 

	return 0;
}