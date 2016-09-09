#include "IActuator.hh"

using namespace gazebo;

extern Motor _motor;
extern Gear _gear;
extern Spindle _spindle;
extern SEE _see;

IActuator::IActuator()
{
};

double IActuator::EfficiencyApproximation()
{
	double param1 = 0.1; // defines steepness of the approxiamtion
	double param2 = 0; // defines zero crossing of the approxiamtion
	return _gear.efficiency + (1/_gear.efficiency - _gear.efficiency)*
		(0.5*(tanh(-param1 * _spindle.angVel * _motor.current - param2) +1));
}


void IActuator::DiffModel( const state_type &x , state_type &dxdt , const double /* t */ )
{
    //x[0] - motor electric current
    //x[1] - spindle angular velocity
	double totalIM = _motor.inertiaMoment + _gear.inertiaMoment; // total moment of inertia
    dxdt[0] = 1/_motor.inductance * (-_motor.resistance * x[0] -
    	_motor.BEMFConst * _gear.ratio * x[1] + _motor.voltage);
    dxdt[1] = _motor.torqueConst * x[0] / (_gear.ratio * totalIM) -
    	_spindle.radius * _see.force / (_gear.ratio * _gear.ratio * totalIM *
    		_gear.appEfficiency);
}
