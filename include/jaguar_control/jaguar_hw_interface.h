//
// Created by ryan on 7/16/17.
//

#ifndef JAGUAR_CONTROL_JAGUAR_HW_INTERFACE_H
#define JAGUAR_CONTROL_JAGUAR_HW_INTERFACE_H

#define CLOSED_LOOP

#include <atomic>
#include <cstddef>
#include <memory>
#include <control_toolbox/pid.h>

#ifdef CLOSED_LOOP
#include <control_toolbox/pid.h>
#include <control_toolbox/pid_gains_setter.h>
#endif

#include <jaguar/jaguar.h>
#include <jaguar/jaguar_bridge.h>
#include <jaguar/jaguar_broadcaster.h>

#include <ros/ros.h>
#include <ros_control_boilerplate/generic_hw_interface.h>

namespace jaguar_control {

    class JaguarHWInterface : public ros_control_boilerplate::GenericHWInterface {
    public:
        enum Side {
            kLeftJaguar,
            kRightJaguar
        };

        JaguarHWInterface(ros::NodeHandle& nh, urdf::Model* urdf_model = NULL);

        virtual void init();

        virtual void read(ros::Duration &elapsed_time);

        virtual void write(ros::Duration &elasped_time);

        virtual void enforceLimits(ros::Duration &period);
    private:

        void block(can::TokenPtr l, can::TokenPtr r);

        void diagCallback(Side side, jaguar::LimitStatus::Enum limits, jaguar::Fault::Enum faults, double voltage, double temp);
        void odomCallback(Side side, double pos, double vel);

        ros::NodeHandle nh;

        std::size_t leftJointIndex;
        std::size_t rightJointIndex;

        std::unique_ptr<can::JaguarBridge> bridge;
        std::unique_ptr<jaguar::JaguarBroadcaster> broadcaster;
        std::unique_ptr<jaguar::Jaguar> leftJag;
        std::unique_ptr<jaguar::Jaguar> rightJag;

        std::atomic<double> leftPos;
        std::atomic<double> rightPos;

        std::atomic<double> leftVel;
        std::atomic<double> rightVel;

        control_toolbox::Pid leftPid;
        control_toolbox::Pid rightPid;
        control_toolbox::PidGainsSetter leftPidSetter;
        control_toolbox::PidGainsSetter rightPidSetter;

        double leftFF;
        double rightFF;

        ros::Publisher leftPidErrorPub;
        ros::Publisher rightPidErrorPub;
    };
}

#endif //JAGUAR_CONTROL_JAGUAR_HW_INTERFACE_H
