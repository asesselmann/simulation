#ifndef _GAZEBO_IACTUATOR_HH_
#define _GAZEBO_IACTUATOR_HH_

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

    class IActuator
    {

        public: IActuator();

	  	// state vector for differential model
		public: typedef std::vector< double > state_type;

	    // stepper for integration
	    public: boost::numeric::odeint::runge_kutta4< state_type > stepper;

	    ////////////////////////////////////////
	    /// \brief Approximates gear's velocity according to the direction of the rotation of the gear
	    /// i.e. eta or 1/eta
	    /// \return Approximated value for gear efficiency
	    public: static double EfficiencyApproximation();

	    ////////////////////////////////////////
	    /// \brief Describes the differential model for the simulations
	    /// of dynamics of a DC motor, a spindle, and a gear box
	    public: static void DiffModel( const state_type &x , state_type &dxdt , const double /* t */ );
    };
}

#endif
