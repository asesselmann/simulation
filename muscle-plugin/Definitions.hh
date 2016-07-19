#ifndef _GAZEBO_DEFINITIONS_HH_
#define _GAZEBO_DEFINITIONS_HH_

namespace gazebo
{
    enum MuscleKinematics
    {
        STRAIGHT_LINE = 0,
        VIA_POINTS = 1,
        SPHERICAL_WRAPPING = 2,
        CYLINDRICAL_WRAPPING = 3,
        MESH_WRAPPING = 4
    };

    enum State
    {
        NOTWRAPPING = 0,
        POSITIVE = 1,
        NEGATIVE = 2
    };


    struct Motor
    {
      double current;
      double torqueConst;
      double resistance;
      double inductance;
      double voltage;
      double BEMFConst; // back electromagnetic force constant
      double inertiaMoment;
    };

    struct Gear
    {
      double inertiaMoment;
      double ratio;
      double efficiency; // gear efficciency
      double appEfficiency; // approximated efficiency
      double position;
    };

    struct Spindle
    {
      double angVel; // angular velocity of the spindle
      double radius;
      double desVel; // desired velocity
    };

		struct SEE
    {
	  double stiffness;
	  double length0;
	  double expansion;
	  double force;
    };

    struct Sphere
    {
        math::Vector3 center;
        double radius;
        int lambda;
    };

    struct Cylinder
    {
        math::Vector3 center;
        math::Vector3 rotation;
        double radius;
        int lambda;
    };
}

#endif
