#ifndef _GAZEBO_STATE_MACHINE_HH_
#define _GAZEBO_STATE_MACHINE_HH_

#include "Definitions.hh"

#include <gazebo/physics/physics.hh>
#include <ignition/math/Vector3.hh>
#include <math.h>

namespace gazebo
{
    class StateMachine{

        private: bool firstUpdateState;
        private: bool firstUpdateLambda;
        public: int state;
        private: math::Vector3 normal;
        public: int lambda;
        private: double projection;

        public: StateMachine();

        ////////////////////////////////////////
        /// \brief Decides if the muscle is positive, negative or not wrapping
        /// \param[in] insertion the insertion point
        /// \param[in] fixation the fixation point
        /// \param[in] center the center of the wrapping surface
        /// \param[in] radius the radius of the wrapping surface
        public: void UpdateState(math::Vector3& insertion, math::Vector3& fixation, math::Vector3& center, double radius);

        ////////////////////////////////////////
        /// \brief updates the RevCounter
        /// \param[in] projection the current projection from vector ta->tb onto the vector ta->p1
        public: void UpdateRevCounter(double projection);
    };
}

#endif
