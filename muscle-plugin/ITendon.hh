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
	    /// \brief Calculate elastic force of the series elastic element
	    /// \param[in] see the series elastic element
	    /// \param[in] _length Current length of the spring
	    public: void ElasticElementModel(SEE &see, const double _length);

        ////////////////////////////////////////
	    /// \brief Calculate muscle length and the muscles lines of action using STRAIGHT LINE muscles
	    public: void muscleLengthLine();

	    ////////////////////////////////////////
	    /// \brief compute the four tangent points of the circle and decide which two to use
	    /// \param[in] insertionP point where the tendon starts
	    /// \param[in] fixationP point where the tendon ends
	    /// \param[in] center center of the circle
	    /// \param[in] radius radius of the circle
	    private: void ComputeTangentPoints(math::Vector3 insertionP, math::Vector3 fixationP, math::Vector3 center, double radius);

	    ////////////////////////////////////////
	    /// \brief compute the four force vectors
	    private: void ComputeForceVectors();

	    ////////////////////////////////////////
	    /// \brief calculates the wrapping angle
	    /// \param[in] radius the radius of the wrapping surface
	    /// \return the wrapping angle
	    private: double CalculateWrappingAngle(double radius);

        ////////////////////////////////////////
	    /// \brief Calculate muscle length and the muscles lines of action using SPHERICAL WRAPPING surfaces
	    public: void muscleLengthSphere();

	    ////////////////////////////////////////
	    /// \brief Calculate the projection of a point onto the cylinder plane
	    /// \param[in] original the original point that should be projected
	    private: math::Vector3 cylinderPlaneProjection(math::Vector3 &original);

	    ////////////////////////////////////////
	    /// \brief projects points from the cylinder plane back into R3
	    /// \param[in] tangent the tangent point that should be projected
	    /// \param[in] localDistance distance between the tangents corresponding via-point and the via-points projection
	    /// \param[in] remoteDistance distance between the other via-point and the other via-points projection
	    /// \param[in] localLength length in the cylinder plane between the projected tanged point and the projected corresponding via-point
        /// \param[in] arcLength length of the arc in the cylinder plane
	    /// \param[in] remoteLength length in the cylinder plane between the other projected tanged point and the other projected via-point
	    private: double R3Projection(math::Vector3 &tangent, double localDistance, double remoteDistance,
                                     double localLength, double arcLength, double remoteLength);

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
