///////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2013, PAL Robotics S.L.
//
//////////////////////////////////////////////////////////////////////////////

#ifndef REEM_HARDWARE_REEM_HARDWARE_H
#define REEM_HARDWARE_REEM_HARDWARE_H

// C++ standard
#include <vector>
#include <string>

// Boost
#include <boost/scoped_ptr.hpp>

// Orocos RTT
#include <rtt/TaskContext.hpp>
#include <rtt/os/TimeService.hpp>

// ros
#include <ros/callback_queue.h>
#include <ros/service.h>
#include <std_srvs/Empty.h>

// pal_ros_control
#include <pal_ros_control/hardware_accessors.h>
#include <pal_ros_control/dumb_spinner.h>

// realtime_tools
#include <realtime_tools/realtime_clock.h>

namespace controller_manager
{
  class ControllerManager;
}

namespace pal_ros_control
{
  class RosControlRobot;
}

namespace reem_hardware
{

class ReemHardware : public RTT::TaskContext
{
public:
  ReemHardware(const std::string& name);

protected:
  bool configureHook();
  bool startHook();
  void updateHook();
  void errorHook();
  void stopHook();
  void cleanupHook();

private:
  // Actuators manager interface
  pal_ros_control::ActuatorAccesor actuators_;
  double dummy_caster_data_; // Dummy raw caster data

  // Inclinometer interface. We use the IMu interface, but only measure orientation in one axis
  pal_ros_control::ImuSensorAccesor  base_orientation_;

  // Emergency stop
  pal_ros_control::EmergencyStopAccesor e_stop_;

  // Time management
  RTT::os::TimeService::ticks last_ticks_;
  realtime_tools::RealtimeClock realtime_clock_;

  // ros_control - robot hardware
  boost::scoped_ptr<pal_ros_control::RosControlRobot> robot_hw_;

  // ros_control - controller manager
  boost::scoped_ptr<controller_manager::ControllerManager> controller_manager_;

  // ROS callback processing
  typedef pal_ros_control::DumbSpinner DumbSpinner;
  ros::CallbackQueue cm_queue_;                       // Services controller manager callbacks
  ros::CallbackQueue start_stop_queue_;               // Services component start/stop requests
  boost::scoped_ptr<DumbSpinner> cm_spinner_;         // Spinner for controller manager callback queue
  boost::scoped_ptr<DumbSpinner> start_stop_spinner_; // Spinner for component start/stop callback queue

  // ROS services for starting/stopping
  ros::ServiceServer start_service_;
  ros::ServiceServer stop_service_;
  bool do_stop_; /// Flag used to signal a pending stop

  bool startService(std_srvs::Empty::Request&  req,
                    std_srvs::Empty::Response& resp)
  {
    if (!isRunning()) {start();}
    return true;
  }

  bool stopService(std_srvs::Empty::Request&  req,
                   std_srvs::Empty::Response& resp)
  {
    if (isRunning()) {do_stop_ = true;}
    return true;
  }

  /**
   * \brief Add dummy caster joints to robot hardware abstraction.
   * Caster joints don't have sensors, so we set them to zero and make them show up in joint_states mostly for cosmetic
   * reasons (otherwise things like tf and the RobotModel Rviz plugin can't update the frame locations).
   */
  bool addDummyCasters();

  /**
   * \brief Method called at every control step for getting up-to-date time variables.
   * \param[in] ticks Current monotonic time in ticks.
   * \param[out] time Current system time.
   * \param [out] period Current control period.
   */
  void getTimeData(const RTT::os::TimeService::ticks& ticks,
                   ros::Time& time,
                   ros::Duration& period)
  {
    // Current time
    using namespace RTT::os;
    ros::Time realtime_time; realtime_time.fromNSec(TimeService::ticks2nsecs(ticks));
    time = realtime_clock_.getSystemTime(realtime_time);

    // Duration since last update
    period.fromNSec(TimeService::ticks2nsecs(TimeService::Instance()->ticksSince(last_ticks_)));

    // Cache current time
    last_ticks_ = ticks;
  }
};

} // namespace

#endif // header guard
