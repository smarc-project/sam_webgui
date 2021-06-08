#ifndef ROSWASM_SAM_H
#define ROSWASM_SAM_H

#include <roswasm_webgui/roswasm_widget.h>
#include <roswasm_webgui/imgui/imgui.h>
#include <roswasm_webgui/imgui/imgui_internal.h>

#include <rosgraph_msgs/Log.h>

#include <std_msgs/String.h>
#include <std_srvs/SetBool.h>
#include <diagnostic_msgs/DiagnosticArray.h>

#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/BatteryState.h>
#include <sensor_msgs/Temperature.h>
#include <sensor_msgs/FluidPressure.h>
#include <nav_msgs/Odometry.h>

#include <sam_msgs/PercentStamped.h>

#include <sam_msgs/ThrusterAngles.h>
#include <sam_msgs/BallastAngles.h>
//#include <sam_msgs/Leak.h>
#include <smarc_msgs/Leak.h>

#include <sam_msgs/ConsumedChargeFeedback.h>
#include <sam_msgs/CircuitStatusStampedArray.h>
#include <sam_msgs/ConsumedChargeArray.h>
#include <sam_msgs/UavcanUpdateBattery.h>

#include <smarc_msgs/DualThrusterFeedback.h>
#include <smarc_msgs/DualThrusterRPM.h>
#include <smarc_msgs/ThrusterRPM.h>
#include <smarc_msgs/CTD.h>
#include <smarc_msgs/Sidescan.h>
#include <smarc_msgs/SensorStatus.h>

#include <uavcan_ros_msgs/CircuitStatus.h>
#include <uavcan_ros_msgs/UavcanNodeStatusNamedArray.h>
#include <uavcan_ros_msgs/UavcanRestartNode.h>

#include <cola2_msgs/DVL.h>

#include <sbg_driver/SbgEkfEuler.h>

#include <list>
#include <chrono>
#include <unordered_map>


namespace roswasm_webgui {

bool draw_ballast_angles(sam_msgs::BallastAngles& msg, roswasm::Publisher& pub);
bool draw_percent(sam_msgs::PercentStamped& msg, roswasm::Publisher& pub);
bool draw_thruster_rpms(smarc_msgs::DualThrusterRPM& msg, roswasm::Publisher& pub);
bool draw_thruster_rpm(smarc_msgs::ThrusterRPM& msg, roswasm::Publisher& pub);
bool draw_thruster_angles(sam_msgs::ThrusterAngles& msg, roswasm::Publisher& pub);
// // bool draw_bar_signed(sam_msgs::ThrusterAngles& msg, roswasm::Publisher& pub);
// bool draw_bar_signed(const ImVec2 pos, const ImVec2 size, const int v_fb, const int rangeBar, const int v_cmd, bool draw_value = true);

extern bool guiDebug;
extern int drawTabs(int _guiMode, const std::map<int, const char*> _modeMap);

class SamActuatorWidget {
private:
    TopicWidget<sam_msgs::ThrusterAngles>* thruster_angles;
    TopicWidget<smarc_msgs::ThrusterRPM>* thruster1_rpm;
    TopicWidget<smarc_msgs::ThrusterRPM>* thruster2_rpm;

    TopicWidget<smarc_msgs::DualThrusterRPM>* thruster_rpms;
    roswasm::Publisher rpm_pub;
    roswasm::Publisher rpm1_pub;
    roswasm::Publisher rpm2_pub;
    roswasm::Timer pub_timer;
    bool rpm_pub_enabled;
    TopicWidget<sam_msgs::PercentStamped>* lcg_actuator;
    TopicWidget<std_msgs::Bool>* lcg_control_enable;
    TopicWidget<std_msgs::Float64>* lcg_control_setpoint;
    TopicWidget<sam_msgs::PercentStamped>* vbs_actuator;
    TopicWidget<std_msgs::Bool>* vbs_control_enable;
    TopicWidget<std_msgs::Float64>* vbs_control_setpoint;
    TopicWidget<sam_msgs::BallastAngles>* tcg_actuator;
    TopicWidget<std_msgs::Bool>* tcg_control_enable;
    TopicWidget<std_msgs::Float64>* tcg_control_setpoint;

public:
    void pub_callback(const ros::TimerEvent& e);
    void show_window(bool& show_actuator_window);
    SamActuatorWidget(roswasm::NodeHandle& nh);
};

class SamDashboardWidget {
private:
    bool was_leak, was_panic = false;
    TopicBuffer<smarc_msgs::Leak>* leak;
    TopicBuffer<std_msgs::String>* panic;
    TopicBuffer<sensor_msgs::NavSatFix>* gps;
    TopicBuffer<sensor_msgs::BatteryState>* battery;
    // TopicBuffer<nav_msgs::Odometry>* odom;
    TopicBuffer<sam_msgs::PercentStamped>* vbs_fb;
    TopicBuffer<sam_msgs::PercentStamped>* lcg;

    //TopicBuffer<smarc_msgs::ThrusterRPM>* rpm1;
    //TopicBuffer<smarc_msgs::ThrusterRPM>* rpm2;
    TopicBuffer<smarc_msgs::DualThrusterFeedback>* rpms;
    // TopicBuffer<smarc_msgs::ThrusterRPM>* rpm1;rpm1_pub
    TopicBuffer<cola2_msgs::DVL>* dvl;
    TopicBuffer<std_msgs::Float64>* odom_x;
    TopicBuffer<std_msgs::Float64>* odom_y;
    TopicBuffer<std_msgs::Float64>* depth;
    TopicBuffer<std_msgs::Float64>* pitch;
    TopicBuffer<std_msgs::Float64>* roll;
    TopicBuffer<std_msgs::Float64>* yaw;
    TopicBuffer<sensor_msgs::Temperature>* motorTemp;
    const ImVec4 emergency_color = ImVec4(1.0f, 0.0f, 0.0f, 1.00f);
public:
    bool is_emergency() { return was_leak || was_panic; }
    void show_window(bool& show_dashboard_window);
    SamDashboardWidget(roswasm::NodeHandle& nh);
};

// class SamDashboardWidget2 {
// private:
//     bool was_leak;
//     TopicBuffer<smarc_msgs::Leak>* leak;
//     TopicBuffer<sensor_msgs::NavSatFix>* gps;
//     TopicBuffer<sensor_msgs::BatteryState>* battery;
//     TopicBuffer<nav_msgs::Odometry>* odom;
//     TopicBuffer<sam_msgs::PercentStamped>* vbs_fb;
//     TopicBuffer<sam_msgs::PercentStamped>* lcg;
//     // TopicBuffer<smarc_msgs::DualThrusterFeedback>* rpms;
//     TopicBuffer<smarc_msgs::ThrusterRPM>* rpm1;
//     TopicBuffer<smarc_msgs::ThrusterRPM>* rpm2;
//     TopicBuffer<std_msgs::Float64>* depth;
//     TopicBuffer<std_msgs::Float64>* pitch;
//     TopicBuffer<std_msgs::Float64>* roll;
//     TopicBuffer<std_msgs::Float64>* yaw;
// public:
//     bool is_emergency() { return was_leak; }
//     void show_window(bool& show_dashboard2_window);
//     SamDashboardWidget2(roswasm::NodeHandle& nh);
// };

// ---------------------------------------- SamMonitorWidget ----------------------------------------
class SamMonitorWidget {
private:
    TopicBuffer<sensor_msgs::BatteryState>* battery;
    // TopicBuffer<uavcan_ros_msgs::CircuitStatus>* circuit;
    TopicBuffer<sam_msgs::CircuitStatusStampedArray>* circuit;
    roswasm::Subscriber subCharge;
    void callbackCharge(const sam_msgs::ConsumedChargeArray& msg);
    std::unordered_map<int, float> circuitCharges;
    roswasm::ServiceCallbackClient batteryService;
    void batteryCallback(const sam_msgs::UavcanUpdateBattery::Response& res, bool result);
    int8_t batteryUpdateResponse = -1;
    // std::list<rosgraph_msgs::Log> btLogList;
    // roswasm::Subscriber* subLog;
    // void callbackLog(const rosgraph_msgs::Log& msg);
    // TopicBuffer<sam_msgs::ConsumedChargeFeedback>* charge;
    TopicBuffer<uavcan_ros_msgs::UavcanNodeStatusNamedArray>* uavcan;
    // TopicBuffer<nav_msgs::Odometry>* odom;
    TopicBuffer<sam_msgs::PercentStamped>* vbs_fb;
    TopicBuffer<sam_msgs::PercentStamped>* vbs_cmd;
    TopicBuffer<sensor_msgs::FluidPressure>* vbs_pressure;
    TopicBuffer<sensor_msgs::Temperature>* vbs_temp;
    TopicBuffer<sam_msgs::PercentStamped>* lcg;
    TopicBuffer<smarc_msgs::DualThrusterFeedback>* thrusters_fb;
    TopicBuffer<smarc_msgs::DualThrusterRPM>* thrusters_cmd;
    TopicBuffer<smarc_msgs::ThrusterRPM>* thruster1_cmd;
    TopicBuffer<smarc_msgs::ThrusterRPM>* thruster2_cmd;
    TopicBuffer<smarc_msgs::ThrusterFeedback>* thruster1_fb;
    TopicBuffer<smarc_msgs::ThrusterFeedback>* thruster2_fb;
    TopicBuffer<smarc_msgs::CTD>* ctd;
    // DVL
    TopicBuffer<cola2_msgs::DVL>* dvl;
    roswasm::Subscriber dvl_status_subscriber;
    void dvl_status_callback(const smarc_msgs::SensorStatus& msg);
    uint32_t lastUpdateDVL = 0;
    smarc_msgs::SensorStatus dvl_status;
    roswasm::ServiceCallbackClient dvlEnableService;
    void dvlEnableCallback(const std_srvs::SetBool::Response& res, bool result);
    int8_t dvlEnableResponse = -1;
    // SSS
    TopicBuffer<smarc_msgs::Sidescan>* sss;
    roswasm::Subscriber sss_status_subscriber;
    void sss_status_callback(const smarc_msgs::SensorStatus& msg);
    uint32_t lastUpdateSSS = 0;
    smarc_msgs::SensorStatus sss_status;
    roswasm::ServiceCallbackClient sssEnableService;
    void sssEnableCallback(const std_srvs::SetBool::Response& res, bool result);
    int8_t sssEnableResponse = -1;

    roswasm::ServiceCallbackClient first_service;
    void service_callback(const uavcan_ros_msgs::UavcanRestartNode::Response& res, bool result);
    TopicBuffer<diagnostic_msgs::DiagnosticArray>* system;
    roswasm::Subscriber subSystem;
    void callbackSystem(const diagnostic_msgs::DiagnosticArray& msg);
    uint32_t lastSystemUpdate = 0;
    int nvMode = -1;
    bool jetsonClocks = false;
    std::string uptime = "";
    std::vector<std::pair<int, float>> cpu{{0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 0}};
    std::pair<int, float> gpu{0, 0};
    std::pair<float, float> ram{0, 0};
    std::vector<float> jTemp{0, 0, 0, 0, 0, 0, 0, 0};

    TopicBuffer<std_msgs::Float64>* odom_x;
    TopicBuffer<std_msgs::Float64>* odom_y;
    TopicBuffer<std_msgs::Float64>* depth;
    TopicBuffer<std_msgs::Float64>* pitch;
    TopicBuffer<std_msgs::Float64>* roll;
    TopicBuffer<std_msgs::Float64>* yaw;
    TopicBuffer<sensor_msgs::Temperature>* motorTemp;
    TopicBuffer<sensor_msgs::FluidPressure>* motorPressure;
    TopicBuffer<sbg_driver::SbgEkfEuler>* sbg_euler;
    // TopicBuffer<rosgraph_msgs::Log>* log;
    const ImVec4 emergency_color = ImVec4(1.0f, 0.0f, 0.0f, 1.00f);
    const ImVec4 warning_color = ImVec4(0.87f, 0.57f, 0.0f, 1.00f);
    const ImVec4 good_color = ImVec4(0.0f, 0.71f, 0.06f, 1.00f);
    const ImVec4 unknown_color = ImVec4(0.50f, 0.50f, 0.50f, 1.00f);
public:
    void show_window(bool& show_monitor_window, bool guiDebug);
    SamMonitorWidget(roswasm::NodeHandle& nh);
};
// -------------------------------------- SamMonitorWidget end --------------------------------------

// ---------------------------------------- SamLogWidget ----------------------------------------
class SamLogWidget {
private:
    roswasm::NodeHandle* nhLocal;
    std::list<rosgraph_msgs::Log> mainLogList;
    std::list<rosgraph_msgs::Log> errorLogList;
    std::list<rosgraph_msgs::Log> warningLogList;
    std::list<rosgraph_msgs::Log> btLogList;
    roswasm::Subscriber subLog;
    bool subLogEnabled = false;
    bool subLogInitialState = true;
    void callbackLog(const rosgraph_msgs::Log& msg);
    bool logEnable = false;
public:
    void show_window(bool& show_roslog_window, bool guiDebug);
    SamLogWidget(roswasm::NodeHandle& nh);
};
// -------------------------------------- SamLogWidget end --------------------------------------

class SamTeleopWidget {
private:
    bool enabled;
    sam_msgs::ThrusterAngles angles_msg;
    smarc_msgs::ThrusterRPM rpm1_msg;
    smarc_msgs::ThrusterRPM rpm2_msg;
    roswasm::Publisher rpm1_pub;
    roswasm::Publisher rpm2_pub;
    smarc_msgs::DualThrusterRPM rpm_msg;
    roswasm::Publisher rpm_pub;
    // roswasm::Publisher* angle_pub;
    roswasm::Publisher angle_pub;
    roswasm::Timer pub_timer;
public:
    void pub_callback(const ros::TimerEvent& e);
    void show_window(bool& show_teleop_window);
    SamTeleopWidget(roswasm::NodeHandle& nh);
};

} // namespace roswasm_webgui

#endif // ROSWASM_EXAMPLES_H
