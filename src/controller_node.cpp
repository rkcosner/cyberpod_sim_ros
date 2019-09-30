#include "cyberpod_sim_ros/controller_node.hpp" 

using namespace Eigen;

ros::NodeHandle *nh_;
ros::NodeHandle *nhParams_;
ros::Subscriber sub_state_;
ros::Subscriber sub_cmd_;
ros::Publisher pub_input_;

cyberpod_sim_ros::input input_;
cyberpod_sim_ros::state stateCurrent_;
cyberpod_sim_ros::cmd cmdCurrent_;

// Global variables
uint32_t iter_;
double maxInclination_;

// Controller gains
double KiVz_;
double KpVz_;
double KpVxy_;
double KpAttitude_;
double KdAttitude_;
double KpOmegaz_;
double KiOmegaz_;

// Controller persistent variables
double hoverThrust_;
double uzInt_;
double uomegazInt_;
double tNm1_;

void computeControlAction(void)
{
	// Prepare Header
	iter_ = stateCurrent_.header.seq;
	input_.header.seq = iter_;
	input_.header.stamp = ros::Time::now();
	input_.header.frame_id = std::to_string(cmdCurrent_.header.seq);
	input_.status = static_cast<uint8_t>(STATUS::RUNNING);

	//Compute orientations from quaternion
	Quaterniond q(stateCurrent_.qw,stateCurrent_.qx,stateCurrent_.qy,stateCurrent_.qz);

	Matrix3d rotBody2World;
	quat2rotm(q,rotBody2World);

	Vector3d eul;
	quat2eulZYX(q,eul);

	// Compute body frame velocity
	Vector3d zBodyInWorld = rotBody2World*(Vector3d() << 0, 0, 1).finished();
	Vector3d vBody(stateCurrent_.vbx,stateCurrent_.vby,stateCurrent_.vbz);
	Vector3d vWorldNoYaw = rotz(-eul(2))*rotBody2World*vBody;

	// Compute error
	Vector3d vDes(cmdCurrent_.vDes[0],cmdCurrent_.vDes[1],cmdCurrent_.vDes[2]);
	const double yawDotDes = cmdCurrent_.vDes[3];

	const double vxError = saturate(KpVxy_*(vDes(0) - vWorldNoYaw(0)),-deg2rad(maxInclination_),deg2rad(maxInclination_));
	const double vyError = saturate(KpVxy_*(vDes(1) - vWorldNoYaw(1)),-deg2rad(maxInclination_),deg2rad(maxInclination_));
	const double vzError = vDes(2) - vWorldNoYaw(2);

	double rollError = -vyError-eul(0);
	double pitchError = vxError-eul(1);
	double rollErrorDot = -stateCurrent_.omegax;
	double pitchErrorDot = -stateCurrent_.omegay;
	double yawErrorDot = yawDotDes - stateCurrent_.omegaz;

	if(iter_==0)
	{
		uzInt_ = hoverThrust_;
		uomegazInt_ = 0.0;
	}
	else
	{
		if(abs(vDes(2))<0.01)
		{
			uzInt_ += (stateCurrent_.time - tNm1_)*KiVz_*vzError;
			saturateInPlace(uzInt_,hoverThrust_-0.1,hoverThrust_+0.1);
		}
		if(abs(yawDotDes)<0.01)
		{
			uomegazInt_ += (stateCurrent_.time - tNm1_)*KiOmegaz_*yawErrorDot;
			saturateInPlace(uomegazInt_,-0.1,0.1);
		}
	}
	tNm1_ = stateCurrent_.time;

	// Compute vz PI output
	double uz = KpVz_*vzError + uzInt_/zBodyInWorld(2);
	double uroll = KpAttitude_*rollError + KdAttitude_*rollErrorDot;
	double upitch = KpAttitude_*pitchError + KdAttitude_*pitchErrorDot;
	double uomegaz = KpOmegaz_*yawErrorDot + uomegazInt_;

	saturateInPlace(uz,hoverThrust_-0.3,hoverThrust_+0.3);
	saturateInPlace(uomegaz,-0.2,0.2);
	saturateInPlace(uroll,-0.5,0.5);
	saturateInPlace(upitch,-0.5,0.5);

	// Fill up message 
	input_.input[0] = uz - uroll - upitch - uomegaz;
	input_.input[1] = uz - uroll + upitch + uomegaz;
	input_.input[2] = uz + uroll + upitch - uomegaz;
	input_.input[3] = uz + uroll - upitch + uomegaz;

	// Clamp inputs
	for(uint32_t i = 0; i<INPUT_LENGTH; i++)
	{
		saturateInPlace(input_.input[i],0.0,1.0);
	}	
}

void controlCallback(const cyberpod_sim_ros::state::ConstPtr msg)
{
	stateCurrent_ = *msg;

	computeControlAction();

	pub_input_.publish(input_);
}

void cmdCallback(const cyberpod_sim_ros::cmd::ConstPtr msg)
{
	cmdCurrent_ = *msg;
}

int main (int argc, char *argv[])
{
	// Init ros
	ros::init(argc,argv,"controller");

	// Instanciate NodeHandles
	nhParams_ = new ros::NodeHandle("~");
	nh_ = new ros::NodeHandle();

	// Initialize variables
	iter_ = 0;
	cmdCurrent_.vDes.fill(0.0);

	// Init pubs, subs and srvs
	sub_state_ = nh_->subscribe<cyberpod_sim_ros::state>("uav_state", 1, controlCallback);
	sub_cmd_ = nh_->subscribe<cyberpod_sim_ros::cmd>("uav_cmd", 1, cmdCallback);
	pub_input_ = nh_->advertise<cyberpod_sim_ros::input>("uav_input", 1);

	// Retreive params
	nhParams_->param<double>("hoverThrust",hoverThrust_,0.5);
	nhParams_->param<double>("KiVz",KiVz_,1.0);
	nhParams_->param<double>("KpVz",KpVz_,1.0);
	nhParams_->param<double>("KpVxy",KpVxy_,1.0);
	nhParams_->param<double>("KpAttitude",KpAttitude_,1.0);
	nhParams_->param<double>("KdAttitude",KdAttitude_,1.0);
	nhParams_->param<double>("KpOmegaz",KpOmegaz_,1.0);
	nhParams_->param<double>("KiOmegaz",KiOmegaz_,1.0);
	nhParams_->param<double>("maxInclination",maxInclination_,30.0);

	// Display node infor
	ROS_INFO("Controller node successfuly started with:");
	ROS_INFO("___hoverThrust=%f",hoverThrust_);
	ROS_INFO("___KiVz=%f",KiVz_);
	ROS_INFO("___KpVz=%f",KpVz_);
	ROS_INFO("___KpVxy=%f",KpVxy_);
	ROS_INFO("___KpAttitude=%f",KpAttitude_);
	ROS_INFO("___KdAttitude=%f",KdAttitude_);
	ROS_INFO("___KpOmegaz=%f",KpOmegaz_);
	ROS_INFO("___KiOmegaz=%f",KiOmegaz_);
	ROS_INFO("___maxInclination=%f",maxInclination_);

	// Take it for a spin
	ros::spin();

	return 0;
}