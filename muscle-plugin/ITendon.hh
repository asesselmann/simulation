#ifndef _GAZEBO_ITENDON_HH_
#define _GAZEBO_ITENDON_HH_

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
#include "StateMachine.hh"

namespace gazebo
{

	using namespace std;
	using namespace boost::numeric::odeint;

	  class ITendon
	 	{
        public: ITendon();

	    ////////////////////////////////////////
	    /// \brief Calculate the dot product between two vectors
	    /// \param[in] _v1 vector 1 coordinates
	    /// \param[in] _v2 vector 2 coordinates
	    /// \return Dot product
	    private: double DotProduct(const math::Vector3 &_v1, const math::Vector3 &_v2);

	    ////////////////////////////////////////
	    /// \brief Calculate the angle between two vectors
	    /// \param[in] _v1 vector 1 coordinates
	    /// \param[in] _v2 vector 2 coordinates
	    /// \return Angle between two vectors in radians
	    private: double Angle(const math::Vector3 &_v1, const math::Vector3 &_v2);

	    ////////////////////////////////////////
	    /// \brief Calculate the crossproduct between two vectors
	    /// \param[in] _v1 vector 1 coordinates
	    /// \param[in] _v2 vector 2 coordinates
	    /// \return Cross Product
	    private: math::Vector3 CrossProduct(const math::Vector3 &_v1, const math::Vector3 &_v2);

	    ////////////////////////////////////////
	    /// \brief Calculate elastic force of the series elastic element
	    /// \param[in] see the series elastic element
	    /// \param[in] _length Current length of the spring
	    public: void ElasticElementModel(SEE &see, const double _length);

        ////////////////////////////////////////
	    /// \brief Calculate muscle length and the muscles lines of action using STRAIGHT LINE muscles
	    public: void muscleLengthLine();

        ////////////////////////////////////////
	    /// \brief Calculate muscle length and the muscles lines of action using SPHERICAL WRAPPING surfaces
	    public: void muscleLengthSphere();

	    ////////////////////////////////////////
	    /// \brief Calculate muscle length and the muscles lines of action using CYLINDRICAL WRAPPING surfaces
	    public: void muscleLengthCylinder();


        //has to be provided by sdf
        private: int muscleKinematics;
        public: int getMuscleKinematics();
        public: void setMuscleKinematics(const int kinematics);

        public: bool firstUpdate;

        //from sdf
        public: math::Vector3 insertion;
        public: math::Vector3 fixation;

        //calculated by muscleLength functions
        public: double muscleLength;
        public: double tendonLength;
        public: double initialTendonLength;
        private: math::Vector3 insertionForceVector;
        private: math::Vector3 fixationForceVector;
        private: math::Vector3 iTangentForceVector;
        private: math::Vector3 fTangentForceVector;
        private: math::Vector3 iTangent;
        private: math::Vector3 fTangent;

        public: math::Vector3 getInsertionForceVector();
        public: math::Vector3 getFixationForceVector();
        public: math::Vector3 getITangentForceVector();
        public: math::Vector3 getFTangentForceVector();
        public: math::Vector3 getITangentPoint();
        public: math::Vector3 getFTangentPoint();

        public: StateMachine stateMachine;

        public: Sphere sphere;
        public: Cylinder cylinder;
    };
}

#endif
