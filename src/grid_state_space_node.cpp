#include "cyberpod_sim_ros/grid_state_space_node.hpp"


using namespace Eigen;

ros::NodeHandle *nh_;
ros::NodeHandle *nhParams_;
ros::ServiceServer srv_ui_;
ros::Subscriber sub_input_;
ros::Publisher pub_state_true_;

double t_;
double dt_;
double umax_;
uint32_t iter_;
STATUS status_;
cyberpod_sim_ros::state stateCurrent_;
cyberpod_sim_ros::input inputCurrent_;
double input_delay_ms_ = 0.;
std::vector<cyberpod_sim_ros::input> inputBuffer_;


void sendTransformCurrent(void)
{
	static tf::TransformBroadcaster odom_broadcaster;
	geometry_msgs::TransformStamped odom_trans;
	odom_trans.header.seq = iter_;
	odom_trans.header.stamp = ros::Time::now();
	odom_trans.header.frame_id = "world";
	odom_trans.child_frame_id = "cyberpod/base_link";

	odom_trans.transform.translation.x = stateCurrent_.x;
	odom_trans.transform.translation.y = stateCurrent_.y;
	odom_trans.transform.translation.z = 0.195;

	Quaterniond cyberpod_q;
	Vector3d cyberpod_eul(0.0,stateCurrent_.psi,stateCurrent_.theta);
	eul2quatZYX(cyberpod_eul,cyberpod_q);

	geometry_msgs::Quaternion odom_quat;
	odom_quat.w = cyberpod_q.w();
	odom_quat.x = cyberpod_q.x();
	odom_quat.y = cyberpod_q.y();
	odom_quat.z = cyberpod_q.z();
	odom_trans.transform.rotation = odom_quat;

	pub_state_true_.publish(stateCurrent_);
	odom_broadcaster.sendTransform(odom_trans);
}

bool interpretRequestCmd(const uint8_t &cmdRaw,
                               CMD &cmd)
{
	if(cmdRaw<5)
	{
		cmd = static_cast<CMD>(cmdRaw);
		return true;
	}
	else
		return false;
}

bool uiCallback(cyberpod_sim_ros::ui::Request &req,
                cyberpod_sim_ros::ui::Response &res)
{
	CMD cmd;
	if(!interpretRequestCmd(req.cmd,cmd))
	{
		ROS_WARN("Unkown request command: %i",req.cmd);
		res.result = false;
		return false;
	}

	switch(cmd)
	{
		case CMD::STOP:
		{
			t_ = 0.0;
			iter_ = 0;
			status_ = STATUS::STOPPED;
			ROS_INFO("Stopping simulation");
			break;
		}
		case CMD::START:
		{
			status_ = STATUS::RUNNING;
			stateCurrent_.status=static_cast<uint8_t>(status_); 
			ROS_INFO("Starting simulation");
			break;
		}
	}

	sendTransformCurrent();

	res.result = true;
	return true;
}


int main (int argc, char *argv[])
{
	// Init ros
	ros::init(argc,argv,"gridder");

	// Create useless broadcaster so that transforms get published
	tf::TransformBroadcaster useless_broadcaster;

	// Instanciate NodeHandles
	nhParams_ = new ros::NodeHandle("~");
	nh_ = new ros::NodeHandle();

	// Init pubs, subs and srvs
	pub_state_true_ = nh_->advertise<cyberpod_sim_ros::state>("state_true", 1);
	srv_ui_ = nh_->advertiseService("ui", uiCallback);


	// Initialize variables
	stateCurrent_.x = 0;
	stateCurrent_.y = 0; 
	stateCurrent_.psi = 0; 
	stateCurrent_.theta = 0;  
	ros::Rate rate(10);

	// Take it for a spin
	while(status_!=STATUS::RUNNING){
		ros::spinOnce(); 
		rate.sleep(); 
	}

	for (double psi = 1; psi>=-1; psi-=0.1){
		for(double x = 1; x>=-1; x-=0.1){
			//Get latest input
			ros::spinOnce();

			//Integrate dynamics
			iter_++;
			t_+=dt_;
			stateCurrent_.x = x; stateCurrent_.stateVec[0] = x; 
			stateCurrent_.psi = psi; stateCurrent_.stateVec[5] = psi; 

			//Publish tranform
			sendTransformCurrent();


			//Wait for tick
			rate.sleep();
		}
	}

	return 0;
}
