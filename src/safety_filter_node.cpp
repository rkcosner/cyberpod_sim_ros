#include "cyberpod_sim_ros/safety_filter_node.hpp"

ros::NodeHandle *nh_;
ros::NodeHandle *nhParams_;

ros::Subscriber sub_state_;
ros::Subscriber sub_inputDes_;

ros::Publisher pub_inputAct_;
ros::Publisher pub_info_;
ros::Publisher pub_backupTraj_;

cyberpod_sim_ros::input inputDes_;
cyberpod_sim_ros::input inputAct_;
cyberpod_sim_ros::state stateCurrent_;
cyberpod_sim_ros::filterInfo filter_info_;
nav_msgs::Path backTrajMsg_;

int32_t passThrough_;
uint32_t iterInput_;
uint32_t iterState_;
uint32_t iter_;
double integration_dt_;
double backup_Tmax_;

// Controller gains


void filterInput(void)
{
	iter_++;
	inputAct_.header.seq = iter_;
	inputAct_.header.stamp = ros::Time::now();
	inputAct_.header.frame_id = std::string("stateIter=") + std::to_string(iterState_) + std::string(", inputIter=") + std::to_string(iterInput_);
	

	if(passThrough_==1)
	{
		std::copy(inputDes_.inputVec.begin(),inputDes_.inputVec.end(),inputAct_.inputVec.begin());
	}
	else
	{
		
	}

	inputAct_.status = static_cast<uint8_t>(STATUS::RUNNING);
}

void inputCallback(const cyberpod_sim_ros::input::ConstPtr msg)
{
	inputDes_ = *msg;
	iterInput_ = inputAct_.header.seq;
}

void stateCallback(const cyberpod_sim_ros::state::ConstPtr msg)
{

	stateCurrent_ = *msg;
	iterState_ = stateCurrent_.header.seq;

	filterInput();
	pub_inputAct_.publish(inputAct_);
	pub_info_.publish(filter_info_);
	pub_backupTraj_.publish(backTrajMsg_);
}


int main(int argc, char *argv[])
{

	// Init ros
	ros::init(argc,argv,"safety_filter");

	// Instanciate NodeHandles
	nhParams_ = new ros::NodeHandle("~");
	nh_ = new ros::NodeHandle();

	// Init pubs, subs and srvs
	sub_state_ = nh_->subscribe<cyberpod_sim_ros::state>("state", 1, stateCallback);
	sub_inputDes_ = nh_->subscribe<cyberpod_sim_ros::input>("inputDes", 1, inputCallback);

	pub_inputAct_ = nh_->advertise<cyberpod_sim_ros::input>("input", 1);
	pub_info_ = nh_->advertise<cyberpod_sim_ros::filterInfo>("safety_filter_info", 1);
	pub_backupTraj_ = nh_->advertise<nav_msgs::Path>("backup_traj", 1);

	// Retreive params
	nhParams_->param<int32_t>("pass_through",passThrough_,0);
	nhParams_->param<double>("integration_dt",integration_dt_,0.01);
	nhParams_->param<double>("backup_Tmax",backup_Tmax_,1.0);
	double integration_steps = round(backup_Tmax_/integration_dt_);

	if(passThrough_!=0 && passThrough_!=1)
	{
		passThrough_ = 1;
		ROS_WARN("passTrough must be 0 or 1. Will be set to %i",passThrough_);
	}
	// Initialize variables
	iter_ = 0;
	iterInput_ = 0;
	iterState_ = 0;
	backTrajMsg_.header.frame_id = "/world";
	backTrajMsg_.poses.reserve(integration_steps+1);

	// Display node info
	ROS_INFO("Safety Filter node successfuly started with:");
	ROS_INFO("___pass_through=%u",passThrough_);
	ROS_INFO("___integration_dt=%.3f",integration_dt_);
	ROS_INFO("___backup_Tmax=%.3f",backup_Tmax_);

	// Take it for a spin
	ros::spin();

	return 0;
}

