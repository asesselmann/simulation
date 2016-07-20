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

void ITendon::ComputeTangentPoints(math::Vector3 insertionP, math::Vector3 fixationP, math::Vector3 center, double radius)
{
    //compute unit vectors and according length
    double l_j1 = (insertionP-center).GetLength();
    math::Vector3 j1 = (insertionP-center)/l_j1;
    double l_j2 = (fixationP-center).GetLength();
    math::Vector3 j2 = (fixationP-center)/l_j2;

    //compute normal,
    math::Vector3 normal = j1.Cross(j2);

    //compute k1, k2
    math::Vector3 k1 = j1.Cross(normal);
    math::Vector3 k2 = normal.Cross(j2);

    //compute length of a1, a2, b1, b2
    double a1 = radius*radius/l_j1;
    double a2 = radius*radius/l_j2;
    double b1 = sqrt(radius*radius - a1*a1);
    double b2 = sqrt(radius*radius - a2*a2);

    if (stateMachine.state == POSITIVE)
    {
        iTangent = center + a1*j1 - b1*k1;
        fTangent = center + a2*j2 - b2*k2;
    }
    else if (stateMachine.state == NEGATIVE)
    {
        iTangent = center + a1*j1 + b1*k1;
        fTangent = center + a2*j2 + b2*k2;
    }
}

void ITendon::ComputeForceVectors()
{
    insertionForceVector = (iTangent-insertion)/(iTangent-insertion).GetLength();
    iTangentForceVector = -insertionForceVector;
    fixationForceVector = (fTangent-fixation)/(fTangent-fixation).GetLength();
    fTangentForceVector = -fixationForceVector;
}

double ITendon::CalculateWrappingAngle(double radius)
{
    //calculate the wrapping angle
    double angle = acos(1 - (iTangent-fTangent).GetLength()/2*radius*radius);
    double alpha = 2*(boost::math::constants::pi<double>())*ceil(stateMachine.lambda/2);
    return (stateMachine.lambda % 2 == 0)?(alpha + angle):(alpha - angle);
}

void ITendon::muscleLengthSphere()
{
    stateMachine.UpdateState(insertion, fixation, sphere.center, sphere.radius);

    //if muscle is not wrapping, use straight line calculation
    if (stateMachine.state == NOTWRAPPING)
    {
        return muscleLengthLine();
    }

    //compute tangent points
    ComputeTangentPoints(insertion, fixation, sphere.center, sphere.radius);

    //compute force vectors
    ComputeForceVectors();

    //update revolution counter
    double projection = (insertion-iTangent).Dot(fTangent-iTangent);
    stateMachine.UpdateRevCounter(projection);

    //calculate wrapping angle
    double alpha = CalculateWrappingAngle(sphere.radius);

    //calculate the lines of action and the muscle's length
    muscleLength = (insertion-iTangent).GetLength() + alpha*sphere.radius + (fTangent-fixation).GetLength();

}

math::Vector3 ITendon::cylinderPlaneProjection(math::Vector3 &original)
{
    //calculate normal onto plane
    math::Vector3 unit_normal = cylinder.rotation.operator*(math::Vector3 (0,0,1));
    //calculate distance to plane
    double distance = (original-cylinder.center).Dot(unit_normal);
    //return projection of the original onto the plane
    return original - distance*unit_normal;
}

double ITendon::R3Projection(math::Vector3 &tangent, double localDistance, double remoteDistance, double localLength, double arcLength, double remoteLength)
{
    //calculate normal onto plane
    math::Vector3 unit_normal = cylinder.rotation.operator*(math::Vector3 (0,0,1));
    //calculate tangent point distance to the plane
    double tangentDistance = localDistance + (localLength)*(remoteDistance - localDistance)/(localLength + arcLength + remoteLength);
    //project tangent point into R3
    tangent += tangentDistance*unit_normal;
    return tangentDistance;
}

void ITendon::muscleLengthCylinder()
{
    //project insertion and fixation point onto xy plane of the cylinder
    math::Vector3 p_insertion = cylinderPlaneProjection(insertion);
    math::Vector3 p_fixation = cylinderPlaneProjection(fixation);

    //update wrapping state
    stateMachine.UpdateState(p_insertion, p_fixation, cylinder.center, cylinder.radius);

    //if muscle is not wrapping, use straight line calculation
    if (stateMachine.state == NOTWRAPPING)
    {
        return muscleLengthLine();
    }

    //compute tangent points in cylinder plane
    ComputeTangentPoints(p_insertion, p_fixation, cylinder.center, cylinder.radius);

    //update revolution counter
    double projection = (p_insertion-iTangent).Dot(fTangent-iTangent);
    stateMachine.UpdateRevCounter(projection);

    //calculate wrapping angle
    double alpha = CalculateWrappingAngle(cylinder.radius);

    //compute lengths in cylinder plane
    double l_insertion = (p_insertion - iTangent).GetLength();
    double l_fixation = (fTangent - p_fixation).GetLength();
    double l_arc = alpha*cylinder.radius;
    double insertionDistance = (insertion - p_insertion).GetLength();
    double fixationDistance = (fixation - p_fixation).GetLength();

    // project tangent points back into R3
    double iTDistance = R3Projection(iTangent, insertionDistance, fixationDistance, l_insertion, l_arc, l_fixation);
    double fTDistance = R3Projection(fTangent, fixationDistance, insertionDistance, l_fixation, l_arc, l_insertion);

    //compute force vectors
    ComputeForceVectors();

    //calculate the lines of action and the muscle's length
    double distance = iTDistance - fTDistance;
    muscleLength = (insertion-iTangent).GetLength() + sqrt(distance*distance + l_arc*l_arc) + (fTangent-fixation).GetLength();
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

