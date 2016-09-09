#include "StateMachine.hh"



using namespace gazebo;

StateMachine::StateMachine(): firstUpdateState(true), firstUpdateLambda(true), state(0), normal(math::Vector3(0,0,0)), lambda(0), projection(0)
{

};

void StateMachine::UpdateState(math::Vector3& insertionP, math::Vector3& fixationP, math::Vector3& center,double radius)
{
    //compute unit vectors and according length
    double l_j1 = (insertionP-center).GetLength();
    math::Vector3 j1 = (insertionP-center)/l_j1;
    double l_j2 = (fixationP-center).GetLength();
    math::Vector3 j2 = (fixationP-center)/l_j2;

    //compute normal
    math::Vector3 normal = j1.Cross(j2);

    //calculate height = distance between straight line from insertion to fixation and spherecenter
    math::Vector3 diff = insertionP-fixationP;
    double height = l_j1 * sin(acos((j1).Dot(diff/diff.GetLength())));

    //if(counter%update == 0)
    //{
    //    gzdbg << "height: " << height << "\n";
    //}

    if(firstUpdateState)
    {
        this->normal = normal;
        switch (state)
        {
            case NOTWRAPPING:
                projection = 0.0;
                break;
            default:
                if (lambda % 2 == 0)
                {
                    projection = -1.0;
                }
                else
                {
                    projection = 1.0;
                }
                break;
        }
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
    firstUpdateState = false;
};

void StateMachine::UpdateRevCounter(double proj)
{
    /*
    if(firstUpdateLambda)
    {
        projection = proj;
    }
    */

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
    //firstUpdateLambda = false;
};
