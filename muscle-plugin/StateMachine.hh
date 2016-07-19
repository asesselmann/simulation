#ifndef _GAZEBO_STATE_MACHINE_HH_
#define _GAZEBO_STATE_MACHINE_HH_

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

#include "Definitions.hh"


namespace gazebo
{

	using namespace std;
	using namespace boost::numeric::odeint;

    class StateMachine{

        public: bool firstUpdate;
        public: int state;
        public: math::Vector3 normal;
        public: int lambda;
        public: double projection;

        public: StateMachine();

        ////////////////////////////////////////
        /// \brief Decides if the muscle is positive, negative or not wrapping
        /// \param[in] normal the durrent normal of j1 and j2
        /// \param[in] height the distance between the center point and the line between p1 and p2
        /// \param[in] radius the radius of the wrapping surface
        /// \param[in] j1 the vector from the center point to p1
        /// \param[in] j2 the vector from the center point to p2
        public: void UpdateState(math::Vector3 normal, double height, double radius, math::Vector3 j1, math::Vector3 j2);

        ////////////////////////////////////////
        /// \brief updates the RevCounter
        /// \param[in] projection the current projection from vector ta->tb onto the vector ta->p1
        public: void UpdateRevCounter(double projection);
    };
}

#endif
