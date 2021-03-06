//
// Created by xuhao on 2016/4/14.
//

#ifndef ETERNITY_FC_RIGIDBODY_H
#define ETERNITY_FC_RIGIDBODY_H
#include <eigen3/Eigen/Eigen>
#include <boost/array.hpp>

class RigidBody
{
private:
    /* sim */
    double ticks;    // dt * ticks = total life time

    Eigen::Matrix3d inertia;

    Eigen::Vector3d position;
    Eigen::Vector3d velocity;
    //Eigen::Vector3d acceleration;

    Eigen::Quaterniond attitude;
    Eigen::Vector3d angularVelocity;

    /* force on the system */
    Eigen::Vector3d force;
    Eigen::Vector3d torque;

    void updateInternalState();

public:
    double mass;
    // For internal use, but needs to be public for odeint
    typedef boost::array<double, 13> InternalState;
    InternalState internalState;
    void operator()(const RigidBody::InternalState &x, RigidBody::InternalState &dxdt, const double /* t */);

    RigidBody();
    void sim_step(double dt);
    void set_force(Eigen::Vector3d _force);
    void set_torque(Eigen::Vector3d _torque);

    Eigen::Quaterniond get_attitude();
    void set_attitude(Eigen::Quaterniond _attitude);
    Eigen::Vector3d get_position();
    void external_set_position(Eigen::Vector3d setting_vec);
    void external_set_attitude(Eigen::Quaterniond _attitude);
    Eigen::Vector3d get_velocity();
    double get_mass();
    Eigen::Vector3d get_angularVelocity() const;
    Eigen::Matrix3d get_inertia() const;

};
#endif //ETERNITY_FC_RIGIDBODY_H
