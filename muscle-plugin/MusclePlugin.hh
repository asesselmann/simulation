#ifndef _GAZEBO_MUSCLE_HH_
#define _GAZEBO_MUSCLE_HH_

#include "gazebo/common/Plugin.hh"
#include "gazebo/common/PID.hh"
#include "gazebo/util/system.hh"
#include <sstream>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <ignition/math/Vector3.hh>
#include <gazebo/common/common.hh>
#include <boost/bind.hpp>
#include <math.h>
#include <map>
#include <stdio.h>
#include <algorithm>
#include <boost/numeric/odeint.hpp>

#include "ros/ros.h"
#include "ros/callback_queue.h"
#include "ros/subscribe_options.h"
#include "std_msgs/Float32.h"
#include "thread"

#include "StateMachine.hh"
#include "Definitions.hh"
#include "ITendon.hh"
#include "IActuator.hh"

namespace gazebo
{

	using namespace std;
	using namespace boost::numeric::odeint;

	class MusclePlugin : public ModelPlugin
	{

	    public: MusclePlugin();
	    public: ~MusclePlugin();

	    public: void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);
	    public: void Init();

	    private: void OnUpdate();
	    private: double VelocityController();

	    public: IActuator::state_type x;
        public: IActuator actuator;
        public: ITendon tendon;

	    private: void updateObstacleOrigins();
	    private: void ComputeStepTime();
	    private: void ApplyForce();

	    /// \brief Handle an incoming message from ROS
		/// \param[in] _msg A double value that is used to set the velocity
		/// of the Velodyne.
		public: void OnRosMsg(const std_msgs::Float32ConstPtr &_msg);

		/// \brief ROS helper function that processes messages
		private: void QueueThread();

		/// \brief Set the velocity of the Velodyne
		/// \param[in] _vel New target velocity
		public: void SetVelocity(const double &_vel);

	    private: event::ConnectionPtr connection;
	    private: common::Time prevUpdateTime;
	    private: common::Time controllerPrevUpdateTime;
	    private: common::Time currTime;
	    private: common::Time stepTime;
	    private: physics::ModelPtr model;
	    private: std::vector<physics::LinkPtr> links;

	    /// \brief Position PID controllers.
      	private:  common::PID posPID;//std::map<std::string, common::PID> posPids;

      	/// \brief Velocity PID controllers.
      	private:  common::PID velPID;

	    /// \brief A node use for ROS transport
		private: ros::NodeHandle *rosNode;//unique_ptr<ros::NodeHandle> *rosNode;

		/// \brief A ROS subscriber
		private: ros::Subscriber rosSub;

		/// \brief A ROS callbackqueue that helps process messages
		private: ros::CallbackQueue rosQueue;

		/// \brief A thread the keeps running the rosQueue
		private: std::thread rosQueueThread;

  };
}

#endif
