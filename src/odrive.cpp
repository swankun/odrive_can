#include <odrive_can_ros/odrive.h>

#include <string>

namespace odrive_can_ros
{

ODriveCAN::ODriveCAN(const std::string can_device, const std::vector<unsigned int>& axes) 
{
    can::ThreadedSocketCANInterfaceSharedPtr driver_ = std::make_shared<can::ThreadedSocketCANInterface> ();
    // initialize device at can_device, 0 for no loopback.
    if (!driver_->init(can_device, 0, can::NoSettings::create()))
    {
        fprintf(stderr, "Failed to initialize can_device at %s\n", can_device.c_str());
        return;
    }
    // register handler for frames and state changes.
    driver_->createMsgListenerM(this, &ODriveCAN::frameCallback);
    driver_->createStateListenerM(this, &ODriveCAN::stateCallback);

    for (auto axis : axes)
    {
        axes_.push_back( ODriveAxis(axis) );
    } 
    
    can::DriverInterfaceSharedPtr driver_handle = driver_;
    odrive_can_.init(driver_handle);
}


void ODriveCAN::frameCallback(const can::Frame &f)
{
    if (!f.isValid())
    {
#ifdef LOG_DEBUG
        fprintf(stderr, "Invalid frame from SocketCAN: id: %#04x, length: %u, is_extended: %d, is_error: %d, is_rtr: %d\n",
                    f.id, f.dlc, f.is_extended, f.is_error, f.is_rtr);
#endif
        return;
    }
    else if (f.is_error)
    {
#ifdef LOG_DEBUG
        fprintf(stderr, "Received frame is error: %s\n", can::tostring(f, true).c_str());
#endif
        return;
    }

    can_Message_t rxmsg;
    rxmsg.id = f.id;
    rxmsg.isExt = f.is_extended;
    rxmsg.len = f.dlc;
    rxmsg.rtr = f.is_rtr;
    std::copy(std::begin(f.data), std::end(f.data), std::begin(rxmsg.buf));
    odrive_can_.handle_can_message(rxmsg, axes_);
}


void ODriveCAN::stateCallback(const can::State &s)
{
    std::string err;
    driver_->translateError(s.internal_error, err);
    if (s.internal_error)
    {
#ifdef LOG_DEBUG
        fprintf(stderr, "Error: %s, asio: %s\n", err.c_str(), s.error_code.message().c_str());
#endif
        return;
    }
}


};  // namespace odrive_can_ros
