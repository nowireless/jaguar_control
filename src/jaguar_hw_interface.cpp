//
// Created by ryan on 7/16/17.
//

#include <boost/signals2.hpp>
#include <jaguar_control/jaguar_hw_interface.h>
#include <cmath>

using namespace ros_control_boilerplate;
using namespace jaguar;
using namespace can;

namespace jaguar_control {
    JaguarHWInterface::JaguarHWInterface(ros::NodeHandle &nh, urdf::Model* urdf_model)
            : GenericHWInterface(nh, urdf_model), nh(nh, "jaguar_hw_interface") {
        ROS_INFO_NAMED("jaguar_control", "JaguarHWInterface Ready");
    }

    void JaguarHWInterface::init() {
        //Use parent function to initialize joints from urdf
        GenericHWInterface::init();

        std::string leftJointName = "front_left_wheel";
        if(!nh.getParam("left_wheel_joint", leftJointName)) {
            ROS_WARN("Using default left wheel joint name");
        }

        std::string rightJointName = "front_right_wheel";
        if(!nh.getParam("right_wheel_joint", rightJointName )) {
            ROS_WARN("Using default right wheel joint name");
        }

        //Find indices of left and right joints
        for (std::size_t i = 0; i < joint_names_.size(); i++) {
            //TODO: Figure out the real joint names
            std::string name = joint_names_[i];
            if(name.compare(leftJointName) != 0) {
                this->leftJointIndex = i;
                ROS_INFO_STREAM("Found left joint: " << name << " at index " << i);
            } else if(name.compare(rightJointName) != 0) {
                this->rightJointIndex = i;
                ROS_INFO_STREAM("Found right joint: " << name << " at index " << i);
            }
        }

        // Initialize Jaguars and stuff
        bridge = std::unique_ptr<JaguarBridge>(new JaguarBridge("/dev/ttyUSB0"));
        broadcaster = std::unique_ptr<JaguarBroadcaster>(new JaguarBroadcaster((*bridge)));

        broadcaster->heartbeat();

        //TODO Get from ROS Param
        int leftId = 4;
        int rightId = 2;

        leftJag = std::unique_ptr<Jaguar>(new Jaguar(*bridge, leftId));
        rightJag = std::unique_ptr<Jaguar>(new Jaguar(*bridge, rightId));

        ROS_INFO("Resetting");
        broadcaster->system_reset();
        sleep(1);
        ROS_INFO("Hopefully reset");

        // Configure Brake mode
        bool enableBrake = true;
        if(enableBrake) {
            ROS_INFO("Configuring break override");
            this->block(leftJag->config_brake_set(BrakeCoastSetting::kOverrideBrake),
                        rightJag->config_brake_set(BrakeCoastSetting::kOverrideBrake));
        } else {
            ROS_INFO("Configuring coast override");
            this->block(leftJag->config_brake_set(BrakeCoastSetting::kOverrideCoast),
                        rightJag->config_brake_set(BrakeCoastSetting::kOverrideCoast));
        }

        // Configure Encoder
        block(leftJag->speed_set_reference(jaguar::SpeedReference::kQuadratureEncoder),
              rightJag->speed_set_reference(jaguar::SpeedReference::kQuadratureEncoder));
        block(leftJag->position_set_reference(jaguar::PositionReference::kQuadratureEncoder),
              rightJag->position_set_reference(jaguar::PositionReference::kQuadratureEncoder));

        int encoderTicks = 360;
        block(leftJag->config_encoders_set(360),
              rightJag->config_encoders_set(360));

        // Configure odom callback
        block(leftJag->periodic_config_odom(0, boost::bind(&jaguar_control::JaguarHWInterface::odomCallback, this, kLeftJaguar, _1, _2)),
              rightJag->periodic_config_odom(0, boost::bind(&jaguar_control::JaguarHWInterface::odomCallback, this, kRightJaguar, _1, _2)));
        uint16_t odomUpdateRateMS = 20;
        block(leftJag->periodic_enable(0, odomUpdateRateMS),
              rightJag->periodic_enable(0, odomUpdateRateMS));

        // Configure diag callback
        block(leftJag->periodic_config_diag(1, boost::bind(&jaguar_control::JaguarHWInterface::diagCallback, this, kLeftJaguar, _1, _2, _3, _4)),
              rightJag->periodic_config_diag(1, boost::bind(&jaguar_control::JaguarHWInterface::diagCallback, this, kRightJaguar, _1, _2, _3, _4)));
        uint16_t diagUpdateRateMS = 500;
        block(leftJag->periodic_enable(1, diagUpdateRateMS),
              rightJag->periodic_enable(1, diagUpdateRateMS));

        ROS_INFO("Enabling");
        this->block(leftJag->voltage_enable(), rightJag->voltage_enable());
        broadcaster->system_resume();


        // Initialize PID controllers
#ifndef DISABLE_PID
        if(!leftPid.init(ros::NodeHandle(nh, "left_pid"))) {
            ROS_ERROR("Could not construct left pid");
        }


        if(!rightPid.init(ros::NodeHandle(nh, "right_pid"))) {
            ROS_ERROR("Could not construct right pid");
        }

        pidGainsSetter.add(&leftPid);
        pidGainsSetter.add(&rightPid);
        pidGainsSetter.advertise(nh);
#endif
    }

    void JaguarHWInterface::read(ros::Duration &elapsed_time) {
        //In theory we only need to set the joint position as diff drive controller only uses the positon
        //joints. There are used only to calculate wheel odometry.

        //We also need to calcluate the left and right wheel velocity for the PID loop

        //Read from encoders here
        double leftPos = this->leftPos; // RAD
        double rightPos = this->rightPos; // RAD
        double leftVel = this->leftVel; // RAD/S
        double rightVel = this->rightVel; // RAD/S


        joint_position_[leftJointIndex] = leftPos;
        joint_position_[rightJointIndex] = rightPos;

        joint_velocity_[leftJointIndex] = leftVel;
        joint_velocity_[rightJointIndex] = rightVel;
    }

    void JaguarHWInterface::write(ros::Duration &elapsed_time) {
        //Safety, does nothing right now
        enforceLimits(elapsed_time);

        double leftCmdVel = joint_velocity_command_[leftJointIndex];
        double rightCmdVel = joint_velocity_command_[rightJointIndex];

        ROS_DEBUG("Cmd Vel L:%f,R:%f", leftCmdVel, rightCmdVel);

        double leftOutput = 0;
        double rightOutput = 0;
#ifndef DISABLE_PID
        // Some PID Stuff
#endif

        const double kV = 0.1; // Feedforward term
        leftOutput = leftCmdVel * kV;
        if(1.0 <= fabs(leftOutput)) leftOutput = std::copysign(1.0, leftOutput);
        rightOutput = rightCmdVel * kV;
        if(1.0 <= fabs(rightOutput)) rightOutput = std::copysign(1.0, rightOutput);

        // This is really important, or the jaguars will stutter
        broadcaster->heartbeat();

        ROS_DEBUG("Out L:%f, R:%f", leftOutput, rightOutput);
        this->block(leftJag->voltage_set(leftOutput), rightJag->voltage_set(rightOutput));

    }


    void JaguarHWInterface::enforceLimits(ros::Duration &period) {
        //Adapted from:
        //https://github.com/davetcoleman/ros_control_boilerplate/blob/kinetic-devel/rrbot_control/src/rrbot_hw_interface.cpp

        //TODO:Pick One
        // ----------------------------------------------------
        // ----------------------------------------------------
        // ----------------------------------------------------
        //
        // CHOOSE THE TYPE OF JOINT LIMITS INTERFACE YOU WANT TO USE
        // YOU SHOULD ONLY NEED TO USE ONE SATURATION INTERFACE,
        // DEPENDING ON YOUR CONTROL METHOD
        //
        // EXAMPLES:
        //
        // Saturation Limits ---------------------------
        //
        // Enforces position and velocity
        //pos_jnt_sat_interface_.enforceLimits(period);
        //
        // Enforces velocity and acceleration limits
        // vel_jnt_sat_interface_.enforceLimits(period);
        //
        // Enforces position, velocity, and effort
        // eff_jnt_sat_interface_.enforceLimits(period);

        // Soft limits ---------------------------------
        //
        // pos_jnt_soft_limits_.enforceLimits(period);
        // vel_jnt_soft_limits_.enforceLimits(period);
        // eff_jnt_soft_limits_.enforceLimits(period);
        //
        // ----------------------------------------------------
        // ----------------------------------------------------
        // ----------------------------------------------------

    }

    void JaguarHWInterface::block(can::TokenPtr l, can::TokenPtr r) {
        if(!l->timed_block(boost::posix_time::seconds(2))) {
            ROS_INFO("Could not send left token");
        }

        if(!r->timed_block(boost::posix_time::seconds(2))) {
            ROS_INFO("Could not send right token");
        }
    }

    void JaguarHWInterface::odomCallback(Side side, double pos, double vel) {
        if(kLeftJaguar == side) {
            leftPos = pos * M_2_PI; // 1 Revolution * (2PI RAD/1 Rev)
            leftVel = vel * M_2_PI / 60.0; // (1 Rev / Min) * (60 Seconds / 1 Min) * (2PI RAD/1 Rev) => Rad/s
        } else {
            rightPos = pos * M_2_PI; // 1 Revolution * (2PI RAD/1 Rev)
            rightVel = vel * M_2_PI / 60.0; // (1 Rev / Min) * (1 Min/ 60 Seconds) * (2PI RAD/1 Rev) => Rad/s
        }
    }

    void JaguarHWInterface::diagCallback(Side side, jaguar::LimitStatus::Enum limits, jaguar::Fault::Enum faults, double voltage, double temp) {
        if(kLeftJaguar == side) {
            ROS_INFO("Left Voltage: %f, Temp: %f", voltage, temp);
        } else {
            ROS_INFO("Right Voltage: %f, Temp: %f", voltage, temp);
        }
    }

}

