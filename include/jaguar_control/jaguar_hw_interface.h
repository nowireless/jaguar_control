//
// Created by ryan on 7/16/17.
//

#ifndef JAGUAR_CONTROL_JAGUAR_HW_INTERFACE_H
#define JAGUAR_CONTROL_JAGUAR_HW_INTERFACE_H

//#define CLOSED_LOOP

#include <atomic>
#include <cstddef>
#include <memory>
#include <control_toolbox/pid.h>

#ifndef DISABLE_PID
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

        ros::NodeHandle nh;

        std::unique_ptr<can::JaguarBridge> bridge;
        std::unique_ptr<jaguar::JaguarBroadcaster> broadcaster;
        std::unique_ptr<jaguar::Jaguar> leftJag;
        std::unique_ptr<jaguar::Jaguar> rightJag;

        std::atomic<double> leftPos;
        std::atomic<double> rightPos;

        std::atomic<double> leftVel;
        std::atomic<double> rightVel;

        void odomCallback(Side side, double pos, double vel);
        void diagCallback(Side side, jaguar::LimitStatus::Enum limits, jaguar::Fault::Enum faults, double voltage, double temp);

        std::size_t leftJointIndex;
        std::size_t rightJointIndex;
    };
}

#endif //JAGUAR_CONTROL_JAGUAR_HW_INTERFACE_H
