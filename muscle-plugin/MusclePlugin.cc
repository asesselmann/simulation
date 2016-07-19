#include <sstream>
#include <gazebo/gazebo.hh>
#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>
#include <ignition/math/Vector3.hh>
#include <boost/bind.hpp>
#include <math.h>
#include <stdio.h>
#include <algorithm>
#include <boost/numeric/odeint.hpp>
#include <vector>
#include <iostream>
#include <fstream>
#include <chrono>

#include "MusclePlugin.hh"
#include "StateMachine.hh"
#include "Definitions.hh"
#include "ITendon.hh"
#include "IActuator.hh"

using namespace gazebo;

//Register plugin with this simulator
GZ_REGISTER_MODEL_PLUGIN(MusclePlugin);

Motor _motor;
Gear _gear;
Spindle _spindle;
SEE _see;

int counter = 0;
int update = 10000;

MusclePlugin::MusclePlugin()
{
}

MusclePlugin::~MusclePlugin()
{
	this->rosQueue.clear();
	this->rosQueue.disable();
	this->rosNode->shutdown();
	this->rosQueueThread.join();

	delete this->rosNode;
}

void MusclePlugin::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
{
	//Store pointer to the model
	this->model = _parent;

	//Get the parameters from SDF
	gzmsg << "Reading values from SDF" << std::endl;

	if (!_sdf->HasElement("muscle_kinematics"))
	{
        gzwarn << "Invalid SDF: Missing required element muscle kinematics";
    }

    this->tendon.setMuscleKinematics(_sdf->Get<int>("muscle_kinematics"));
    switch(this->tendon.getMuscleKinematics())
    {
        case 0:
            gzdbg << "muscleKinematics STRAIGHT_LINE\n";
            break;
        case 1:
            gzdbg << "muscleKinematics VIA_POINTS\n";
            break;
        case 2:
            gzdbg << "muscleKinematics SPHERICAL_WRAPPING\n";
            break;
        case 3:
            gzdbg << "muscleKinematics CYLINDRICAL_WRAPPING\n";
            break;
        case 4:
            gzdbg << "muscleKinematics MESH_WRAPPING\n";
            break;
    }


	if (_sdf->HasElement("motor"))
	{
		sdf::ElementPtr motorElement = _sdf->GetElement("motor");
		gzdbg << "motor:\n";

		if (!motorElement->HasElement("torque_constant") ||
			!motorElement->HasElement("bemf_constant") ||
			!motorElement->HasElement("inductance") ||
			!motorElement->HasElement("resistance") ||
			!motorElement->HasElement("inertiaMoment"))

		{
			gzwarn << "Invalid SDF: Missing required elements for motor model";
		}

		if (motorElement->HasElement("torque_constant"))
		{
			_motor.torqueConst = motorElement->Get<double>("torque_constant");
			gzdbg << "torque_constant " <<_motor.torqueConst<< "\n";
		}

		if (motorElement->HasElement("bemf_constant"))
		{
			_motor.BEMFConst = motorElement->Get<double>("bemf_constant");
			gzdbg << "bemf_constant " <<_motor.BEMFConst<< "\n";
		}

		if (motorElement->HasElement("inductance"))
		{
			_motor.inductance = motorElement->Get<double>("inductance");
			gzdbg << "inductance " <<_motor.inductance<< "\n";
		}

		if (motorElement->HasElement("resistance"))
		{
			_motor.resistance = motorElement->Get<double>("resistance");
			gzdbg << "resistance " <<_motor.resistance<< "\n";
		}

		if (motorElement->HasElement("inertiaMoment"))
		{
			_motor.inertiaMoment = motorElement->Get<double>("inertiaMoment");
			gzdbg << "inertia " <<_motor.inertiaMoment<< "\n";
		}
	}

	if (_sdf->HasElement("gear"))
	{
		sdf::ElementPtr gearElement = _sdf->GetElement("gear");
		gzdbg << "gear:\n";

		if (!gearElement->HasElement("ratio") ||
			!gearElement->HasElement("efficiency") ||
			!gearElement->HasElement("inertiaMoment"))

		{
			gzwarn << "Invalid SDF: Missing required elements for gear model";
		}

		if (gearElement->HasElement("ratio"))
		{
			_gear.ratio = gearElement->Get<double>("ratio");
			gzdbg << "ratio " <<_gear.ratio<< "\n";
		}

		if (gearElement->HasElement("efficiency"))
		{
			_gear.efficiency = gearElement->Get<double>("efficiency");
			gzdbg << "efficiency " <<_gear.efficiency<< "\n";
		}

		if (gearElement->HasElement("inertiaMoment"))
		{
			_gear.inertiaMoment = gearElement->Get<double>("inertiaMoment");
			gzdbg << "inertia " <<_gear.inertiaMoment<< "\n";
		}
	}


	if (_sdf->HasElement("spindle"))
	{
		sdf::ElementPtr spindleElement = _sdf->GetElement("spindle");
		gzdbg << "spindle:\n";

		if (!spindleElement->HasElement("radius"))

		{
			gzwarn << "Invalid SDF: Missing required elements for spindle model";
		}

		if (spindleElement->HasElement("radius"))
		{
			_spindle.radius = spindleElement->Get<double>("radius");
			gzdbg << "radius " <<_spindle.radius<< "\n";
		}
	}

	if (_sdf->HasElement("SEE"))
	{

		sdf::ElementPtr elasticElement = _sdf->GetElement("SEE");
		gzdbg << "SEE:\n";


		if (!elasticElement->HasElement("stiffness") ||
            !elasticElement->HasElement("length0") /*|| !elasticElement->HasElement("damping")*/)
		{
			gzwarn << "Invalid SDF: Missing required elements for series elastic element";
		}

		if (elasticElement->HasElement("stiffness"))
		{
			_see.stiffness = elasticElement->Get<double>("stiffness");
			gzdbg << "stiffness " <<_see.stiffness<< "\n";
		}

		if (elasticElement->HasElement("length0"))
		{
			_see.stiffness = elasticElement->Get<double>("length0");
			gzdbg << "length0 " <<_see.length0<< "\n";
		}
	}

	std::vector<std::string> linkNames = {"upper_arm", "hand"};
	for(auto linkName: linkNames)
	{
		physics::LinkPtr link = this->model->GetLink(linkName);
    	if (!link)
    	{
	        gzwarn << "Invalid SDF: model link " << linkName << " does not "
	               << "exist!" << std::endl;
	        continue;
    	}
    	this->links.push_back(link);
	}

	if(this->tendon.getMuscleKinematics() == SPHERICAL_WRAPPING)
	{
        std::vector<std::string> linkNames = {"sphere"};
        for(auto linkName: linkNames)
        {
            physics::LinkPtr link = this->model->GetLink(linkName);
            if (!link)
            {
                gzwarn << "Invalid SDF: model link " << linkName << " does not "
                        << "exist!" << std::endl;
                continue;
            }

            this->links.push_back(link);

            //that's not save!
            this->tendon.sphere.center = link->GetWorldCoGPose().pos;
            this->tendon.sphere.radius = link->GetSDF()->GetElement("collision")->GetElement("geometry")->GetElement("sphere")->Get<double>("radius");
            this->tendon.stateMachine.state = link->GetSDF()->GetElement("projector")->GetElement("plugin")->Get<int>("state_init");
            this->tendon.stateMachine.lambda = link->GetSDF()->GetElement("projector")->GetElement("plugin")->Get<int>("wcounter_init");
            gzdbg << linkName << " center " << this->tendon.sphere.center.x << " " << this->tendon.sphere.center.y << " " << this->tendon.sphere.center.z << "\n";
            gzdbg << linkName << " radius " << this->tendon.sphere.radius << "\n";
            gzdbg << linkName << " state " << this->tendon.stateMachine.state << "\n";
            gzdbg << linkName << " lambda " << this->tendon.stateMachine.lambda << "\n";

        }
	}

	// Initialize ros, if it has not already bee initialized.
	if (!ros::isInitialized())
	{

	  int argc = 0;
	  char **argv = NULL;
	  ros::init(argc, argv, "gazebo_client",
	      ros::init_options::NoSigintHandler);
	}

	gzdbg << "initialized ros\n";


	// Establish connection
	this->connection = event::Events::ConnectWorldUpdateBegin(
         			 boost::bind(&MusclePlugin::OnUpdate, this));
    gzdbg << "established connection\n";


	// Create the ROS node. This acts in a similar manner to
	// the Gazebo node
	this->rosNode = new ros::NodeHandle("gazebo_client");
	gzdbg << "create ros node\n";

	// Create a named topic, and subscribe to it.
	ros::SubscribeOptions so =
	  ros::SubscribeOptions::create<std_msgs::Float32>(
	      "/roboy/trajectory_motor0/vel",
	      1000,
	      boost::bind(&MusclePlugin::OnRosMsg, this, _1),
	      ros::VoidPtr(), &this->rosQueue);
	this->rosSub = this->rosNode->subscribe(so);
    gzdbg << "subscribe to topic\n";

	// Spin up the queue helper thread.
	this->rosQueueThread =
	  std::thread(std::bind(&MusclePlugin::QueueThread, this));
    gzdbg << "helper thread\n";

}

void MusclePlugin::Init()
{
    gzdbg << "muscle plugin init\n";
	//state initialization
    x = {0,0};

    gzdbg << "initialized state_type x\n";
    // _motor.voltage = 0.0;
    _spindle.angVel = 0;
    _spindle.desVel = 0;

    _see.expansion = 0;
	_see.force = 0;

	_gear.position = 0;

	gzdbg << "init spindle, see, gear\n";
	switch (this->tendon.stateMachine.state)
	{
        case NOTWRAPPING:
            this->tendon.stateMachine.projection = 0.0;
            break;
        default:
            if (this->tendon.stateMachine.lambda % 2 == 0)
            {
                this->tendon.stateMachine.projection = -1.0;
            }
            else
            {
                this->tendon.stateMachine.projection = 1.0;
            }
            break;
	}
	gzdbg << "init tendon projection\n";

    // get bounding box of the link
    gzmsg << "CoG 1 : " << links[0]->GetWorldCoGPose() << "\n";
    gzmsg << "CoG 2 : " << links[1]->GetWorldCoGPose() << "\n";

	// initialize PID controller
	// BUG in gazebo PID controller - min/max value cannot be 0
	this->velPID.Init(1500, 0.0, 0.0, 1.0, 0.0, 220.0, 0.000001);

}

double MusclePlugin::VelocityController()
{
	common::Time _currTime = this->model->GetWorld()->GetSimTime();
	common::Time _stepTime = _currTime - this->controllerPrevUpdateTime;

	if ( _currTime.Double()==0 || fmod(_currTime.Double(),0.01) == 0 )
    {
		gzdbg << "target velocity:"
			  << _spindle.desVel
			  << "\n";
		gzdbg << "current velocity:"
			  << _spindle.angVel
			  << "\n";
	}

	return this->velPID.Update( _spindle.angVel - _spindle.desVel, _stepTime);
}

void MusclePlugin::OnRosMsg(const std_msgs::Float32ConstPtr &_msg)
{
  this->SetVelocity(static_cast<double>(_msg->data));
  gzdbg << "set desired velocity to " << _msg->data << "\n";
}

void MusclePlugin::QueueThread()
{
  static const double timeout = 0.001;
  while (this->rosNode->ok())
  {
    this->rosQueue.callAvailable(ros::WallDuration(timeout));
  }
}

void MusclePlugin::SetVelocity(const double &_vel)
{
  // Set the spindle's target velocity.
  _spindle.desVel = _vel;
}

void MusclePlugin::updateObstacleOrigins()
{
    const math::Pose armPose = links[0]->GetWorldCoGPose();
	const math::Pose handPose = links[1]->GetWorldCoGPose();
	this->tendon.insertion = armPose.pos;
	this->tendon.fixation = handPose.pos;
}

void MusclePlugin::OnUpdate()
{

	//compute the step time
	common::Time currTime = this->model->GetWorld()->GetSimTime();
    common::Time stepTime = currTime - this->prevUpdateTime;
 	this->prevUpdateTime = currTime;

	// set the voltage
	if (_spindle.desVel != _spindle.angVel)
	{
		_motor.voltage = VelocityController();
		if (_motor.voltage == 0.000001)
		{
            _motor.voltage = 0;
		}
	    this->controllerPrevUpdateTime = currTime;
	}

	if ( currTime.Double()==0 || fmod(currTime.Double(),0.01) == 0 )
    {
		 gzdbg << "voltage: "
		  << _motor.voltage
		  << "\n";
	}



	// get the positions of the links
	updateObstacleOrigins();

	//if(counter%update == 0)
	//{
    //    gzdbg << "insertion " << this->tendon.insertion << "\n";
    //    gzdbg << "fixation " << this->tendon.fixation << "\n";
	//}



    //compute force points and muscle length

    switch (this->tendon.getMuscleKinematics())
	{
        case STRAIGHT_LINE:
            this->tendon.muscleLengthLine();
            break;
        case VIA_POINTS:
            break;
        case SPHERICAL_WRAPPING:
            this->tendon.muscleLengthSphere();
            break;
        case CYLINDRICAL_WRAPPING:
            break;
        case MESH_WRAPPING:
            break;
	}

    if (this->tendon.firstUpdate)
    {
        this->tendon.tendonLength = this->tendon.initialTendonLength;
    }
    this->tendon.firstUpdate = false;

    if (counter%update == 0)
    {
        gzdbg << "tendonLength: " << this->tendon.tendonLength << "\n";
        gzdbg << "muscleLength: " << this->tendon.muscleLength << "\n";
	}

    //calculate elastic force
    double length = this->tendon.muscleLength - this->tendon.tendonLength;
    this->tendon.ElasticElementModel( _see, length);


    //apply Force
    math::Vector3 insertionForce = this->tendon.getInsertionForceVector() * _see.force;
    math::Vector3 fixationForce = this->tendon.getFixationForceVector() * _see.force;
    this->links[0]->AddForceAtWorldPosition(insertionForce, this->tendon.insertion);
	this->links[1]->AddForceAtWorldPosition(fixationForce, this->tendon.fixation);
	if (this->tendon.getMuscleKinematics() == SPHERICAL_WRAPPING && this->tendon.stateMachine.state != NOTWRAPPING)
	{
        math::Vector3 iTangentForce = this->tendon.getITangentForceVector() * _see.force;
        math::Vector3 fTangentForce = this->tendon.getFTangentForceVector() * _see.force;
        this->links[2]->AddForceAtWorldPosition(iTangentForce, this->tendon.getITangentPoint());
        this->links[2]->AddForceAtWorldPosition(fTangentForce, this->tendon.getFTangentPoint());
    }



	// calculate the approximation of gear's efficiency
	_gear.appEfficiency = actuator.EfficiencyApproximation();

	// do 1 step of integration of DiffModel() at current time
	actuator.stepper.do_step(IActuator::DiffModel, x, currTime.Double(), stepTime.Double());
	 // gzdbg << "electric current: "
		// 	  << x[0]
		// 	  << "\t"
		// 	  << "speed: "
		// 	  << x[1]
		// 	  << "\n";
	_motor.current = x[0];
 	_spindle.angVel = x[1];
 	//if(counter%update == 0 && _spindle.angVel < 0)
 	//{
    //    gzdbg << "angVel: " << _spindle.angVel << "\n";
 	//}

    _gear.position += _spindle.angVel*stepTime.Double();
 	this->tendon.tendonLength = this->tendon.initialTendonLength - _spindle.radius*_gear.position;
 	if (counter%update == 0)
 	{
        gzdbg << "gear position: " << _gear.position << "\n";
 	}


    // if ((int) currTime.Double() % 2 == 0)
    // {
	    // gzdbg << "electric current: "
			  // << _motor.current
			  // << "\n"
			  // << "speed: "
			  // << _spindle.angVel
			  // << "\n"
			  // << "voltage"
			  // << _motor.voltage
			  // << "\n";
			  // << " and motorForce: "
			  // << actuatorForce
			  // << "\n"
			  // << " and length: "
			  // << _see.length
			  // << "\n"
			  // << "current time: "
			  // << currTime.Double()
			  // << "\n";
    // }

    counter++;

}
