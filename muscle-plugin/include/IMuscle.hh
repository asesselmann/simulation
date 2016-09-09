#ifndef _GAZEBO_IMUSCLE_HH_
#define _GAZEBO_IMUSCLE_HH_

#include "Definitions.hh"
#include "StateMachine.hh"

#include <boost/numeric/odeint.hpp>

namespace gazebo
{

    struct Sphere
	{
        math::Vector3 center;
        double radius;
    };

    struct Cylinder
    {
        math::Vector3 center;
        math::Matrix3 rotation;
        double radius;
    };

	class ITendon
    {

        public: StateMachine stateMachine;

        public: Sphere sphere;
        public: Cylinder cylinder;

        //from sdf
        public: math::Vector3 insertion;
        public: math::Vector3 fixation;

        //calculated by kinematics functions
        public: double muscleLength;
        public: math::Vector3 insertionForceVector;
        public: math::Vector3 fixationForceVector;

        //calculated by the wrapping functions
        public: math::Vector3 iTangent;
        public: math::Vector3 fTangent;
        public: math::Vector3 iTangentForceVector;
        public: math::Vector3 fTangentForceVector;



        public: double tendonLength;
        public: double initialTendonLength;

        public: ITendon();

	    ////////////////////////////////////////
	    /// \brief Calculate elastic force of the series elastic element
	    /// \param[in] see the series elastic element
	    /// \param[in] _length Current length of the spring
	    public: void ElasticElementModel(SEE &see, const double &length);

        ////////////////////////////////////////
	    /// \brief Calculate muscle length and the muscles lines of action using STRAIGHT LINE muscles
	    public: void StraightLineKinematics();

	    ////////////////////////////////////////
	    /// \brief compute the four tangent points of the circle and decide which two to use
	    /// \param[in] insertionP point where the tendon starts
	    /// \param[in] fixationP point where the tendon ends
	    /// \param[in] center center of the circle
	    /// \param[in] radius radius of the circle
	    private: void TangentPoints(math::Vector3& insertionP, math::Vector3& fixationP, math::Vector3& center, double radius);

	    ////////////////////////////////////////
	    /// \brief compute the four force vectors
	    private: void ForceVectors();

	    ////////////////////////////////////////
	    /// \brief calculates the wrapping angle
	    /// \param[in] radius the radius of the wrapping surface
	    /// \return the wrapping angle
	    private: double WrappingAngle(double radius);

        ////////////////////////////////////////
	    /// \brief Calculate muscle length and the muscles lines of action using SPHERICAL WRAPPING surfaces
	    public: void SphericalWrapping();

	    ////////////////////////////////////////
	    /// \brief Calculate the projection of a point onto the cylinder plane
	    /// \param[in] original the original point that should be projected
	    /// \return the projection of the point onto the cylinder plane
	    private: math::Vector3 cylinderPlaneProjection(math::Vector3& original);

	    ////////////////////////////////////////
	    /// \brief projects points from the cylinder plane back into R3
	    /// \param[in] tangent the tangent point that should be projected
	    /// \param[in] localDistance distance between the tangents corresponding via-point and the via-points projection
	    /// \param[in] remoteDistance distance between the other via-point and the other via-points projection
	    /// \param[in] localLength length in the cylinder plane between the projected tanged point and the projected corresponding via-point
        /// \param[in] arcLength length of the arc in the cylinder plane
	    /// \param[in] remoteLength length in the cylinder plane between the other projected tanged point and the other projected via-point
	    /// \return the distance of the point to the plane
	    private: double R3Projection(math::Vector3& tangent, double localDistance, double remoteDistance,
                                     double localLength, double arcLength, double remoteLength);

	    ////////////////////////////////////////
	    /// \brief Calculate muscle length and the muscles lines of action using CYLINDRICAL WRAPPING surfaces
	    public: void CylindricalWrapping();

	    struct Mesh
        {
            math::Vector3 wrappingDir;
            std::map<int, math::Vector3> vertices;
            std::set<math::Vector3> facets;
        };

        public: math::Vector3 x_axis;
        public: math::Vector3 y_axis;
        public: math::Vector3 z_axis;
        private: std::map<int, math::Vector3> convexHullVertices;
        private: std::map<int, math::Vector3> convexHullFacets;
        private: std::map<int, math::Vector3> facetNormals;
        private: double alphaMax;

        private: void CoordinateFrame();

        private: std::map<int, math::Vector3> HalfSpace();

        private: void ConvexEnvelope();

        private: std::map<int, math::Vector3> GeodesicPath();

        public: void MeshWrapping();


        //has to be provided by sdf
        private: int muscleKinematics;
        public: int getMuscleKinematics();
        public: void setMuscleKinematics(const int kinematics);

        public: bool firstUpdate;




    };
}

#endif
