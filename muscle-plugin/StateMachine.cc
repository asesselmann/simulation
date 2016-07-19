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

#include "StateMachine.hh"

using namespace gazebo;

StateMachine::StateMachine(): firstUpdate(true), state(0), normal(math::Vector3(0,0,0)), lambda(0), projection(0)
{

};

void StateMachine::UpdateState(math::Vector3 normal, double height, double radius, math::Vector3 j1, math::Vector3 j2)
{
    if(firstUpdate)
    {
        this->normal = normal;
    }

    //state machine decides how muscle length is calculated
    switch(state)
    {
        case NOTWRAPPING:
            if((height < radius) && (j1.Dot(j2) < 0))
            {
                state = POSITIVE;
                gzdbg << "state POSITIVE\n";
            }
            break;
        case POSITIVE:
            if ((height >= radius) && (lambda == 0))
            {
                state = NOTWRAPPING;
                gzdbg << "state NOTWRAPPING\n";
            }
            else if (normal.Dot(this->normal) < 0)
            {
                state = NEGATIVE;
                gzdbg << "state NEGATIVE!!!!!!!!!!!!!!!!!!!!!!\n";
            }
            break;
        case NEGATIVE:
            if (normal.Dot(this->normal) < 0)
            {
                state = POSITIVE;
                gzdbg << "state POSITIVE\n";
            }
            break;
    }

    this->normal = normal;
    firstUpdate = false;
};

void StateMachine::UpdateRevCounter(double proj)
{
    if(state == POSITIVE)
    {
        if(projection<0 && proj>0)
        {
            lambda--;
        }
        else if (projection>0 && proj<0)
        {
            lambda++;
        }
    }
    else if (state == NEGATIVE)
    {
        if(projection<0 && proj>0)
        {
            lambda++;
        }
        else if (projection>0 && proj<0)
        {
            lambda--;
        }
    }

    projection = proj;
};
