#ifndef ODRIVE_CAN_ROS_ODRIVE_H
#define ODRIVE_CAN_ROS_ODRIVE_H
// #define LOG_DEBUG

#include <string>

#include <socketcan_interface/socketcan.h>
#include <socketcan_interface/threading.h>

#include <odrive_can_ros/can_simple.hpp>

namespace odrive_can_ros
{

class ODriveCAN
{
    public:
        ODriveCAN(const std::string can_device, const std::vector<unsigned int>& axes);

    private:
        can::ThreadedSocketCANInterfaceSharedPtr driver_;
        void frameCallback(const can::Frame &f);
        void stateCallback(const can::State &s);
        
        CANSimple odrive_can_;
        std::vector<ODriveAxis> axes_;


};  // class ODriveCAN


};  // namespace odrive_can_ros

#endif  // ODRIVE_CAN_ROS_ODRIVE_H
