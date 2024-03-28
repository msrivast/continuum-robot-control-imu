#ifndef shape_space_attitude_h
#define shape_space_attitude_h
//Shape space attitude parameterization assumes that the quaternion has been generated without yaw(torsion) using only roll and pitch.
//(Quaternions obscure information about how a particular rotation was generated in general.) 
class Shape_space_attitude{
    public:
        double theta=0.0;
        double phi=0.0;
};

#endif