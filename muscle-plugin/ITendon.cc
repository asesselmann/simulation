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

#include "ITendon.hh"
#include "StateMachine.hh"
#include "Definitions.hh"

using namespace gazebo;

ITendon::ITendon(): muscleLength(0), tendonLength(0), initialTendonLength(0.257593), firstUpdate(true)
{
};

double ITendon::DotProduct(const math::Vector3 &_v1, const math::Vector3 &_v2)
{
	return _v1.x*_v2.x + _v1.y*_v2.y + _v1.z*_v2.z;
}


double ITendon::Angle(const math::Vector3 &_v1, const math::Vector3 &_v2)
{
	return acos(_v1.Dot(_v2)/_v1.GetLength()*_v2.GetLength());
}


math::Vector3 ITendon::CrossProduct(const math::Vector3 &_v1, const math::Vector3 &_v2)
{
    math::Vector3 _v3;
    _v3.x = _v1.y*_v2.z - _v1.z*_v2.y;
    _v3.y = _v1.z*_v2.x - _v1.x*_v2.z;
    _v3.z = _v1.x*_v2.y - _v1.y*_v2.x;
    return _v3;
}


void ITendon::ElasticElementModel(SEE &see, const double _length)
{
    see.expansion = _length - see.length0;

    if (see.expansion>=0)
    {
        see.force=see.expansion*see.stiffness;
    }
    else
    {
        see.force=0;
    }
}


void ITendon::muscleLengthLine()
{
    math::Vector3 diff = insertion-fixation;
    muscleLength = diff.GetLength();
    fixationForceVector = diff/muscleLength;
    insertionForceVector = -fixationForceVector;
};


void ITendon::muscleLengthSphere()
{
    //compute unit vectors and according length
    double l_j1 = (insertion-sphere.center).GetLength();
    math::Vector3 j1 = (insertion-sphere.center)/l_j1;
    double l_j2 = (fixation-sphere.center).GetLength();
    math::Vector3 j2 = (fixation-sphere.center)/l_j2;

    //compute normal,
    math::Vector3 normal = j1.Cross(j2);

    //calculate height = distance between straight line from insertion to fixation and spherecenter
    math::Vector3 diff = insertion-fixation;
    double height = l_j1 * sin(acos((j1).Dot(diff/diff.GetLength())));

    //if(counter%update == 0)
    //{
    //    gzdbg << "height: " << height << "\n";
    //}

    stateMachine.UpdateState(normal, height, sphere.radius, j1, j2);
    math::Vector3 iTangent, fTangent;

    //if muscle is not wrapping, use straight line calculation
    if (stateMachine.state == NOTWRAPPING)
    {
        return muscleLengthLine();
    }

    //compute k1, k2
    math::Vector3 k1 = j1.Cross(normal);
    math::Vector3 k2 = normal.Cross(j2);

    //compute length of a1, a2, b1, b2
    double a1 = sphere.radius*sphere.radius/l_j1;
    double a2 = sphere.radius*sphere.radius/l_j2;
    double b1 = sqrt(sphere.radius*sphere.radius - a1*a1);
    double b2 = sqrt(sphere.radius*sphere.radius - a2*a2);

    if (stateMachine.state == POSITIVE)
    {
        iTangent = sphere.center + a1*j1 - b1*k1;
        fTangent = sphere.center + a2*j2 - b2*k2;
    }
    else if (stateMachine.state == NEGATIVE)
    {
        iTangent = sphere.center + a1*j1 + b1*k1;
        fTangent = sphere.center + a2*j2 + b2*k2;
    }

    //update revolution counter
    double projection = (insertion-iTangent).Dot(fTangent-iTangent);
    stateMachine.UpdateRevCounter(projection);

    //calculate the wrapping angle
    double angle = acos(1 - (iTangent-fTangent).GetLength()/2*sphere.radius*sphere.radius);
    double alpha = 2*(boost::math::constants::pi<double>())*ceil(stateMachine.lambda/2);
    alpha += (stateMachine.lambda % 2 == 0)?(angle):(-angle);

    //calculate the lines of action and the muscle's length
    insertionForceVector = (iTangent-insertion)/(iTangent-insertion).GetLength();
    iTangentForceVector = -insertionForceVector;
    fixationForceVector = (fTangent-fixation)/(fTangent-fixation).GetLength();
    fTangentForceVector = -fixationForceVector;
    muscleLength = (insertion-iTangent).GetLength() + alpha*sphere.radius + (fTangent-fixation).GetLength();

}


void ITendon::muscleLengthCylinder()
{

}


math::Vector3 ITendon::getInsertionForceVector()
{
    return this->insertionForceVector;
}


math::Vector3 ITendon::getFixationForceVector()
{
    return this->fixationForceVector;
}

math::Vector3 ITendon::getITangentForceVector()
{
    return this->iTangentForceVector;
}

math::Vector3 ITendon::getFTangentForceVector()
{
    return this->fTangentForceVector;
}

math::Vector3 ITendon::getITangentPoint()
{
    return this->iTangent;
}

math::Vector3 ITendon::getFTangentPoint()
{
    return this->fTangent;
}

int ITendon::getMuscleKinematics()
{
    return this->muscleKinematics;
}

void ITendon::setMuscleKinematics(int kinematics)
{
    this->muscleKinematics = (kinematics >= 0 && kinematics <= 4)?(kinematics):0;
}

