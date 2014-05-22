///////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2013, PAL Robotics S.L.
//
//////////////////////////////////////////////////////////////////////////////

// C++ standard
#include <algorithm>
#include <sstream>
#include <stdexcept>

// Orocos RTT
#include <rtt/Component.hpp>

// ros
#include <ros/ros.h>

// ros_control
#include <controller_manager/controller_manager.h>

// pal_ros_control
#include <pal_ros_control/ros_control_robot.h>

// Project
#include <reem_hardware/reem_hardware.h>

using std::vector;
using std::string;

namespace
{
const string ACT_POS_CUR_PORT        = "act_position";
const string ACT_VEL_CUR_PORT        = ""; //"act_velocity";
const string ACT_POS_REF_PORT        = "ref_position";
const string ACT_VEL_REF_PORT        = "ref_velocity";

const string BASE_ORIENTATION_NAME   = "base_inclinometer";      // TODO: Fetch from URDF?
const string BASE_ORIENTATION_FRAME  = "base_inclinometer_link"; // TODO: Fetch from URDF?
const string BASE_ORIENTATION_PORT   = "orientation_base";

const string EMERGENCY_STOP_PORT     = "emergency_stop_state";
}

namespace reem_hardware
{

ReemHardware::ReemHardware(const string &name)
  : RTT::TaskContext(name, PreOperational),
    actuators_(ACT_POS_CUR_PORT,
               ACT_VEL_CUR_PORT,
               ACT_POS_REF_PORT,
               ACT_VEL_REF_PORT,
               "", //No max current command interface
               this),
    dummy_caster_data_(0.0),
    base_orientation_(BASE_ORIENTATION_NAME,
                      BASE_ORIENTATION_FRAME,
                      BASE_ORIENTATION_PORT,
                      this),
    e_stop_(EMERGENCY_STOP_PORT,
            this),
    do_stop_(false)
{}

bool ReemHardware::configureHook()
{
  // Precondition
  if (isConfigured())
  {
    ROS_INFO_STREAM("Component is already configured. Cleaning up first.");
    cleanupHook();
  }

  // Setup actuators
  if (!actuators_.configure()) {return false;}

  // Setup inclinometer
  if (!base_orientation_.configure()) {return false;}
  pal_ros_control::RawImuDataList imu_data(1, base_orientation_.getData());

  // Setup callback queue and spinner thread
  ros::NodeHandle cm_nh, start_stop_nh;
  cm_nh.setCallbackQueue(&cm_queue_);
  start_stop_nh.setCallbackQueue(&start_stop_queue_);
  cm_spinner_.reset(new DumbSpinner(&cm_queue_));
  start_stop_spinner_.reset(new DumbSpinner(&start_stop_queue_));

  // Reset robot hardware abstraction
  try
  {
    robot_hw_.reset(new pal_ros_control::RosControlRobot(actuators_.getData(),
                                                         pal_ros_control::RawForceTorqueDataList(), // empty
                                                         imu_data,
                                                         cm_nh));
  }
  catch(const std::runtime_error& ex)
  {
    ROS_ERROR_STREAM(ex.what());
    return false;
  }
  catch(...)
  {
    ROS_ERROR_STREAM("Unexpected exception caught while constructing robot hardware abstraction.");
    return false;
  }

  // Add dummy caster joints to robot hardware abstraction
  if (!addDummyCasters()) {return false;}

  // Reset controller manager
  controller_manager_.reset(new controller_manager::ControllerManager(robot_hw_.get(), cm_nh));

  // Start/stop services
  start_service_ = start_stop_nh.advertiseService(getName() + "/start", &ReemHardware::startService, this);
  stop_service_  = start_stop_nh.advertiseService(getName() + "/stop",  &ReemHardware::stopService,  this);

  // Start servicing ros callbacks
  cm_spinner_->start();
  start_stop_spinner_->start();

  return true;
}

bool ReemHardware::startHook()
{
  // Precondition: Configured component
  if (!isConfigured())
  {
    ROS_ERROR("Can't start component. It has not yet been configured.");
    return false;
  }

  // Precondition: Connected ports
  if (!actuators_.start() ||
      !base_orientation_.start())
      //!e_stop_.start())// TODO: Add estop check?
  {
    return false;
  }

  // Check e-stop
  if (!e_stop_.read())
  {
    // TODO: Enable!
//     ROS_ERROR("Can't start component: Couldn't read emergency stop state");
//     return false;
  }

  // Hold current position
  // TODO: Can we get rid of the eror message at start of one lost cycle?
  if (!actuators_.read(ros::Duration(0.005))) // TODO: Remove magic number!
  {
    ROS_ERROR("Can't start component: Couldn't read actuators state");
    return false;
  }
  robot_hw_->holdPosition();

  // Initialize previous control cycle timestamp
  last_ticks_ = RTT::os::TimeService::Instance()->getTicks();

  return true;
}

void ReemHardware::updateHook()
{
  // Stop if requested
  if (do_stop_)
  {
    do_stop_ = false;
    stop();
    return;
  }

  // Time management // TODO: Get time from actuators manager?
  using namespace RTT::os;
  ros::Time time;
  ros::Duration period;
  TimeService::ticks ticks = TimeService::Instance()->getTicks();
  getTimeData(ticks, time, period);

  // Emergency stop management
  e_stop_.read();
  if(e_stop_.getValue())
  {
    this->error();
    return;
  }

  // Read current robot state
  if (!actuators_.read(period))
  {
    this->error();
    return;
  }
  robot_hw_->read(time, period);

  // Compute the controller commands
  controller_manager_->update(time, period);

  // Write commands
  robot_hw_->write(time, period);
  actuators_.write();

  // Update cycle time sanity check
  // TODO: Implement statistics publisher and don't log!
  if (getPeriod() > 0.0)
  {
    const double update_duration = TimeService::Instance()->secondsSince(ticks);
    if (update_duration > getPeriod())
    {
      ROS_ERROR_STREAM("Update cycle took " << update_duration <<
                       "s, which is greater than the control period of " << getPeriod() << "s.");
    }
  }
}

void ReemHardware::errorHook()
{
  // Stop if requested
  if (do_stop_)
  {
    do_stop_ = false;
    stop();
    return;
  }

  // Time management // TODO: Get time from actuators manager?
  using namespace RTT::os;
  ros::Time time;
  ros::Duration period;
  TimeService::ticks ticks = TimeService::Instance()->getTicks();
  getTimeData(ticks, time, period);

  // Emergency stop management
  e_stop_.read();
  const bool estop_released = !e_stop_.getValue();
  const bool read_act_ok = actuators_.read(period);

  if (estop_released && read_act_ok)
  {
    robot_hw_->holdPosition();
    robot_hw_->reset();
    actuators_.resetVelocityEstimator();
    controller_manager_->update(time, period, true); // restart controllers
    robot_hw_->write(time, period);
    actuators_.write();
    this->recover();
  }
}

void ReemHardware::stopHook()
{
  // Reset raw data and forward null commands to actuator manager
  actuators_.stop();
  actuators_.write(); // NOTE: Not really needed but a redundant safety measure
  robot_hw_->reset(); // NOTE: Joint limit interface handles need to be reset
}

void ReemHardware::cleanupHook()
{
  // Stop servicing ros callbacks
  cm_spinner_->stop();
  start_stop_spinner_->stop();

  // NOTE: Without this statement, unloading the component gives a:
  // "pure virtual method called, terminate called without an active exception. Aborted". Nasty.
  controller_manager_.reset();

  // Shutdown start/stop services
  start_service_.shutdown();
  stop_service_.shutdown();
}

bool ReemHardware::addDummyCasters()
{
  // Preconditions
  if (!robot_hw_)
  {
    ROS_ERROR("Robot hardware abstraction not yet initialized. Not adding dummy casters.");
    return false;
  }
  hardware_interface::JointStateInterface* js_iface = robot_hw_->get<hardware_interface::JointStateInterface>();
  if (!js_iface)
  {
    ROS_ERROR("Robot hardware abstraction does not have a JointStates interface!. Not adding dummy casters.");
    return false;
  }

  // Add dummy casters
  double* dummy = &dummy_caster_data_;
  hardware_interface::JointStateHandle caster_left_1("caster_left_1_joint", dummy, dummy, dummy);
  hardware_interface::JointStateHandle caster_left_2("caster_left_2_joint", dummy, dummy, dummy);
  hardware_interface::JointStateHandle caster_right_1("caster_right_1_joint", dummy, dummy, dummy);
  hardware_interface::JointStateHandle caster_right_2("caster_right_2_joint", dummy, dummy, dummy);

  js_iface->registerHandle(caster_left_1);
  js_iface->registerHandle(caster_left_2);
  js_iface->registerHandle(caster_right_1);
  js_iface->registerHandle(caster_right_2);

  return true;
}

ORO_CREATE_COMPONENT(reem_hardware::ReemHardware);

} // namespace
