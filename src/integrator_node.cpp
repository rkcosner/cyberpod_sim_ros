#include "cyberpod_sim_ros/integrator_node.hpp" 


ros::NodeHandle *nh_;
ros::NodeHandle *nhParams_;
ros::ServiceServer srv_ui_;
ros::Subscriber sub_input_;
ros::Publisher pub_state_;
ros::Publisher pub_stateRaw_;

double t_;
double dt_;
uint32_t iter_;
STATUS status_;
stepper_t odeStepper_;
state_t odeState_;
cyberpod_sim_ros::state stateCurrent_;
cyberpod_sim_ros::input inputCurrent_;
state_t initialConditions_{0.0,0.0,1.0,
                           1.0,0.0,0.0,0.0,
                           0.0,0.0,0.0,
                           0.0,0.0,0.0,
                           0.0,0.0,0.0,0.0};

void resetOdeState(void)
{
	odeState_ = initialConditions_;
	inputCurrent_.input[0] = 0.0;
	inputCurrent_.input[1] = 0.0;
	inputCurrent_.input[2] = 0.0;
	inputCurrent_.input[3] = 0.0;
}

void dynamicsCL(const state_t &x, state_t &xDot, const double t)
{
	xDot = x;
	double inputSaturated[INPUT_LENGTH];
	for(uint32_t i = 0; i<INPUT_LENGTH; i++)
	{
		if(inputCurrent_.input[i]>1.0)
			inputSaturated[i] = 1.0;
		else if(inputCurrent_.input[i]<0.0)
			inputSaturated[i] = 0.0;
		else
			inputSaturated[i] = inputCurrent_.input[i];
	}
   uavDynamics(x.data(),inputSaturated,xDot.data(),t);
}

void inputCallback(const cyberpod_sim_ros::input::ConstPtr msg)
{
	inputCurrent_ = *msg;
}

void updateStateCurrent(void)
{
	//normalize quaternion
	Quaterniond q(odeState_[3],odeState_[4],odeState_[5],odeState_[6]);
	q.normalize();

	stateCurrent_.status = static_cast<uint8_t>(status_);
	stateCurrent_.time = t_;
	stateCurrent_.x = odeState_[0];
	stateCurrent_.y = odeState_[1];
	stateCurrent_.z = odeState_[2];
	stateCurrent_.qw = q.w();
	stateCurrent_.qx = q.x();
	stateCurrent_.qy = q.y();
	stateCurrent_.qz = q.z();
	stateCurrent_.vx = odeState_[7];
	stateCurrent_.vy = odeState_[8];
	stateCurrent_.vz = odeState_[9];
	stateCurrent_.omegax = odeState_[10];
	stateCurrent_.omegay = odeState_[11];
	stateCurrent_.omegaz = odeState_[12];
	stateCurrent_.omega1 = odeState_[13];
	stateCurrent_.omega2 = odeState_[14];
	stateCurrent_.omega3 = odeState_[15];
	stateCurrent_.omega4 = odeState_[16];

	Matrix3d rotBody2World;
	quat2rotm(q,rotBody2World);

	Vector3d v(odeState_[7],odeState_[8],odeState_[9]);
	Vector3d vb = rotBody2World.transpose()*v;

	stateCurrent_.vbx = vb(0);
	stateCurrent_.vby = vb(1);
	stateCurrent_.vbz = vb(2);
}

void sendStateCurrent(void)
{
	stateCurrent_.header.seq = iter_;
	stateCurrent_.header.stamp = ros::Time::now();
	stateCurrent_.header.frame_id = std::to_string(inputCurrent_.header.seq);
	
	pub_state_.publish(stateCurrent_);

	std_msgs::Float64MultiArray stateRaw;
	stateRaw.data = odeState_;
	pub_stateRaw_.publish(stateRaw);
}

void sendTransformCurrent(void)
{
	tf::TransformBroadcaster uav_odom_broadcaster;
	geometry_msgs::TransformStamped uav_odom_trans;
	uav_odom_trans.header.seq = iter_;
	uav_odom_trans.header.stamp = ros::Time::now();
	uav_odom_trans.header.frame_id = "world";
    uav_odom_trans.child_frame_id = "uav/base_link";

    uav_odom_trans.transform.translation.x = stateCurrent_.x;
    uav_odom_trans.transform.translation.y = stateCurrent_.y;
    uav_odom_trans.transform.translation.z = stateCurrent_.z;
	
	geometry_msgs::Quaternion uav_odom_quat;
	uav_odom_quat.w = stateCurrent_.qw;
	uav_odom_quat.x = stateCurrent_.qx;
	uav_odom_quat.y = stateCurrent_.qy;
	uav_odom_quat.z = stateCurrent_.qz;
	uav_odom_trans.transform.rotation = uav_odom_quat;

	uav_odom_broadcaster.sendTransform(uav_odom_trans);
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
			resetOdeState();
			status_ = STATUS::STOPPED;
			ROS_INFO("Stopping simulation");
			break;
		}
		case CMD::START:
		{
			status_ = STATUS::RUNNING;
			ROS_INFO("Starting simulation");
			break;
		}
		case CMD::PAUSE:
		{
			status_ = STATUS::STOPPED;
			ROS_INFO("Pausing simulation");
			break;
		}
		case CMD::REPOSITION:
		{
			if(req.data.size()==STATE_LENGTH)
			{
				ROS_INFO("State reseted to custom value");
				odeState_ = state_t(req.data.begin(),req.data.end());		
			}
			else
			{
				ROS_INFO("State reseted to Initial Conditions");
				resetOdeState();
			}
			break;
		}
		case CMD::RESET:
		{
			t_ = 0.0;
			iter_ = 0;
			resetOdeState();
			ROS_INFO("Resetting simulation");
			break;
		}
	}
	updateStateCurrent();
	sendStateCurrent();
	sendTransformCurrent();

	res.result = true;
	return true;
}

int main (int argc, char *argv[])
{
	// Init ros
	ros::init(argc,argv,"integrator");

	// Create useless broadcaster so that transforms get published
	tf::TransformBroadcaster useless_broadcaster;

	// Instanciate NodeHandles
	nhParams_ = new ros::NodeHandle("~");
	nh_ = new ros::NodeHandle();

	// Init pubs, subs and srvs
	sub_input_ = nh_->subscribe<cyberpod_sim_ros::input>("uav_input", 1,inputCallback);
	pub_state_ = nh_->advertise<cyberpod_sim_ros::state>("uav_state", 1);
	pub_stateRaw_ = nh_->advertise<std_msgs::Float64MultiArray>("uav_state_raw", 10);
	srv_ui_ = nh_->advertiseService("integrator/ui", uiCallback);

	// Retreive params
	nhParams_->param<double>("dt",dt_,0.001);
	nhParams_->param<state_t>("IC",initialConditions_,initialConditions_);

	if(dt_<=0.0)
	{
		dt_ = 0.001;
		ROS_WARN("dt must be strictly positive");
	}
	if(initialConditions_.size()!=STATE_LENGTH)
	{
		initialConditions_ = state_t{0.0,0.0,1.0,
		                             1.0,0.0,0.0,0.0,
		                             0.0,0.0,0.0,
		                             0.0,0.0,0.0,
		                             0.0,0.0,0.0,0.0};
		ROS_WARN("Initial conditions must be of length 17");
	}
	// Initialize variables
	t_ = 0.0;
	iter_ = 0;
	status_ = STATUS::STOPPED;
	resetOdeState();

	ros::Rate rate(1/dt_);

	// Display node info
	ROS_INFO("Integrator node successfuly started with:");
	ROS_INFO("___dt=%.4fs",dt_);
	ROS_INFO("___IC=");
	for(uint8_t i = 0; i<STATE_LENGTH; i++)
	{
		ROS_INFO("      %.3f",initialConditions_[i]);
	}

	// Take it for a spin
	while(ros::ok())
	{

		//Get latest input
		ros::spinOnce();

		//Integrate dynamics
		if(status_==STATUS::RUNNING)
		{
			odeStepper_.do_step(dynamicsCL,odeState_,t_,dt_);
			updateStateCurrent();
			iter_++;
			t_+=dt_;			

			//Publish state
			sendStateCurrent();

			//Publish tranform
			sendTransformCurrent();
		}

		//Wait for tick
		rate.sleep();
	}

	return 0;
}