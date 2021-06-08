#include <sam_webgui/roswasm_sam.h>

#include <roswasm_webgui/imgui/imgui.h>
#include <roswasm_webgui/imgui/imgui_internal.h>

namespace roswasm_webgui {

static void HelpMarker(const char* desc)
{
    ImGui::TextDisabled("(?)");
    if (ImGui::IsItemHovered())
    {
        ImGui::BeginTooltip();
        ImGui::PushTextWrapPos(ImGui::GetFontSize() * 35.0f);
        ImGui::TextUnformatted(desc);
        ImGui::PopTextWrapPos();
        ImGui::EndTooltip();
    }
}

bool draw_ballast_angles(sam_msgs::BallastAngles& msg, roswasm::Publisher& pub)
{
    ImGui::PushID("Angles cmd slider");
    ImGui::SliderFloat("", &msg.weight_1_offset_radians, -1.6f, 1.6f, "%.2frad");
    if (ImGui::IsItemDeactivatedAfterChange()) {
        msg.weight_2_offset_radians = msg.weight_1_offset_radians;
        pub.publish(msg);
    }
    ImGui::PopID();
    ImGui::SameLine();
    ImGui::InputFloat("Angles cmd input", &msg.weight_1_offset_radians, 0.0f, 0.0f, "%.2f");
    if (ImGui::IsItemDeactivatedAfterChange()) {
        msg.weight_2_offset_radians = msg.weight_1_offset_radians;
        pub.publish(msg);
    }

    return false;
}

bool draw_percent(sam_msgs::PercentStamped& msg, roswasm::Publisher& pub)
{
    ImGui::PushID("Percent cmd slider");
    ImGui::SliderFloat("", &msg.value, 0.0f, 100.0f, "%.2f%%");
    if (ImGui::IsItemDeactivatedAfterChange()) {
        pub.publish(msg);
    }
    bool lock = ImGui::IsItemActive();

    ImGui::PopID();
    ImGui::SameLine();
    ImGui::InputFloat("Percent cmd input", &msg.value, 0.0f, 0.0f, "%.2f");
    if (ImGui::IsItemDeactivatedAfterChange()) {
        pub.publish(msg);
    }

    return lock;
}

bool draw_thruster_rpm(smarc_msgs::ThrusterRPM& msg, roswasm::Publisher& pub)
{
    ImGui::PushID("RPM cmd slider");
    ImGui::SliderInt("", &msg.rpm, -2000, 2000, "%drpm");
    if (ImGui::IsItemDeactivatedAfterChange()) {
        pub.publish(msg);
    }
    bool lock = ImGui::IsItemActive();

    ImGui::PopID();
    ImGui::SameLine();
    ImGui::InputInt("RPM cmd input", &msg.rpm);
    if (ImGui::IsItemDeactivatedAfterChange()) {
        pub.publish(msg);
    }

    return lock;
}

bool draw_thruster_rpms(smarc_msgs::DualThrusterRPM& msg, roswasm::Publisher& pub)

{
    ImGui::PushID("First cmd slider");
    ImGui::Text("Thruster 1");
    ImGui::SameLine();
    ImGui::SliderInt("", &msg.thruster_front.rpm, -1000, 1000, "%drpm");
    if (ImGui::IsItemDeactivatedAfterChange()) {
        pub.publish(msg);
    }
    bool lock = ImGui::IsItemActive();
    ImGui::PopID();
    ImGui::SameLine();
    ImGui::InputInt("First cmd input", &msg.thruster_front.rpm);
    if (ImGui::IsItemDeactivatedAfterChange()) {
        pub.publish(msg);
    }

    ImGui::PushID("Second cmd slider");
    ImGui::Text("Thruster 2");
    ImGui::SameLine();
    ImGui::SliderInt("", &msg.thruster_back.rpm, -1000, 1000, "%drpm");
    if (ImGui::IsItemDeactivatedAfterChange()) {
        pub.publish(msg);
    }
    lock = lock || ImGui::IsItemActive();
    ImGui::PopID();
    ImGui::SameLine();
    ImGui::InputInt("Second cmd input", &msg.thruster_back.rpm);
    if (ImGui::IsItemDeactivatedAfterChange()) {
        pub.publish(msg);
    }

    return lock;
}
// */

bool draw_thruster_angles(sam_msgs::ThrusterAngles& msg, roswasm::Publisher& pub)
{
    const double pi_ = 3.14159265359;
    ImGui::PushID("First cmd slider");
    ImGui::Text("Hori (deg)");
    ImGui::SameLine();
    static float thruster_horizontal_degrees = 0.0;
    ImGui::SliderFloat("", &thruster_horizontal_degrees, -9.0f, 9.0f, "%.2f");
    msg.thruster_horizontal_radians = thruster_horizontal_degrees*pi_/180.0;
    if (ImGui::IsItemDeactivatedAfterChange()) {
        pub.publish(msg);
    }
    ImGui::PopID();
    ImGui::SameLine();
    ImGui::InputFloat("First cmd input", &thruster_horizontal_degrees, 0.0f, 0.0f, "%.2f");
    msg.thruster_horizontal_radians = thruster_horizontal_degrees*pi_/180.0;
    if (ImGui::IsItemDeactivatedAfterChange()) {
        pub.publish(msg);
    }

    ImGui::PushID("Second cmd slider");
    ImGui::Text("Vert (deg)");
    ImGui::SameLine();
    static float thruster_vertical_degrees = 0.0;
    ImGui::SliderFloat("", &thruster_vertical_degrees, -9.0f, 9.0f, "%.2f");
    msg.thruster_vertical_radians = thruster_vertical_degrees*pi_/180.0;
    if (ImGui::IsItemDeactivatedAfterChange()) {
        pub.publish(msg);
    }
    ImGui::PopID();
    ImGui::SameLine();
    ImGui::InputFloat("Second cmd input", &thruster_vertical_degrees, 0.0f, 0.0f, "%.2f");
    msg.thruster_vertical_radians = thruster_vertical_degrees*pi_/180.0;
    if (ImGui::IsItemDeactivatedAfterChange()) {
        pub.publish(msg);
    }

    return false;
}

SamActuatorWidget::SamActuatorWidget(roswasm::NodeHandle& nh) : rpm_pub_enabled(false)
{
    //thruster_angles = new TopicPairWidget<geometry_msgs::Pose2D, std_msgs::Float64>(nh, DrawFloatPair("Hori (rad)", -0.1, 0.18, "Vert (rad)", -0.1, 0.15), "core/thrust_vector_cmd", "core/thrust_fb1", "core/thrust_fb2");
    //thruster_rpms = new TopicPairWidget<geometry_msgs::Pose2D, std_msgs::Float64>(nh, DrawFloatPair("Thruster 1", -1000., 1000., "Thruster 2", -1000., 1000.), "core/rpm_cmd", "core/rpm_fb1", "core/rpm_fb2");
    thruster_angles = new TopicWidget<sam_msgs::ThrusterAngles>(nh, &draw_thruster_angles, "core/thrust_vector_cmd");
    // thruster_rpms = new TopicWidget<sam_msgs::ThrusterRPMs>(nh, &draw_thruster_rpms, "core/rpm_cmd", "core/rpm_fb");

    thruster_rpms = new TopicWidget<smarc_msgs::DualThrusterRPM>(nh, &draw_thruster_rpms, "core/thrusters_cmd", "core/thrusters_cmd");
    rpm_pub = nh.advertise<smarc_msgs::DualThrusterRPM>("core/thrusters_cmd", 1000);

    thruster1_rpm = new TopicWidget<smarc_msgs::ThrusterRPM>(nh, &draw_thruster_rpm, "core/thruster1_cmd");
    thruster2_rpm = new TopicWidget<smarc_msgs::ThrusterRPM>(nh, &draw_thruster_rpm, "core/thruster2_cmd");
    rpm1_pub = nh.advertise<smarc_msgs::ThrusterRPM>("core/thruster1_cmd", 1000);
    rpm2_pub = nh.advertise<smarc_msgs::ThrusterRPM>("core/thruster2_cmd", 1000);

    lcg_actuator = new TopicWidget<sam_msgs::PercentStamped>(nh, &draw_percent, "core/lcg_cmd", "core/lcg_fb");
    lcg_control_enable = new TopicWidget<std_msgs::Bool>(nh, &draw_bool, "ctrl/lcg/pid_enable");
    lcg_control_setpoint = new TopicWidget<std_msgs::Float64>(nh, DrawFloat64(-1.6, 1.6), "ctrl/lcg/setpoint"); //, -1.6, 1.6)

    vbs_actuator = new TopicWidget<sam_msgs::PercentStamped>(nh, &draw_percent, "core/vbs_cmd", "core/vbs_fb");
    vbs_control_enable = new TopicWidget<std_msgs::Bool>(nh, &draw_bool, "ctrl/vbs/pid_enable");
    vbs_control_setpoint = new TopicWidget<std_msgs::Float64>(nh, DrawFloat64(0., 5.), "ctrl/vbs/setpoint"); //, 0 5)
        
    tcg_actuator = new TopicWidget<sam_msgs::BallastAngles>(nh, &draw_ballast_angles, "core/tcg_cmd");
    tcg_control_enable = new TopicWidget<std_msgs::Bool>(nh, &draw_bool, "ctrl/tcg/pid_enable");
    tcg_control_setpoint = new TopicWidget<std_msgs::Float64>(nh, DrawFloat64(-1.6, 1.6), "ctrl/tcg/setpoint"); //, -1.6, 1.6)

    pub_timer = nh.createTimer(roswasm::Duration(0.08), std::bind(&SamActuatorWidget::pub_callback, this, std::placeholders::_1));
    pub_timer.stop();
}

void SamActuatorWidget::pub_callback(const ros::TimerEvent& e)
{
    if (rpm_pub_enabled) {
        // rpm_pub.publish(thruster_rpms->get_msg());
        rpm1_pub.publish(thruster1_rpm->get_msg());
        rpm2_pub.publish(thruster2_rpm->get_msg());
    }
}

void SamActuatorWidget::show_window(bool& show_actuator_window)
{
    ImGui::SetNextWindowSize(ImVec2(500, 500), ImGuiCond_FirstUseEver);
    ImGui::Begin("Actuator controls", &show_actuator_window);

    if (ImGui::CollapsingHeader("Thruster Angles", ImGuiTreeNodeFlags_Framed | ImGuiTreeNodeFlags_DefaultOpen)) {
        ImGui::PushID("Angles");
        thruster_angles->show_widget();
        ImGui::PopID();
    }
    if (ImGui::CollapsingHeader("Thruster RPMs", ImGuiTreeNodeFlags_Framed | ImGuiTreeNodeFlags_DefaultOpen)) {
        ImGui::PushID("RPMs");
        // thruster_rpms->show_widget();
        ImGui::PushID("First");
        ImGui::Text("Thruster 1");
        ImGui::SameLine();
        thruster1_rpm->show_widget();
        ImGui::PopID();
        ImGui::PushID("Second");
        ImGui::Text("Thruster 2");
        ImGui::SameLine();
        thruster2_rpm->show_widget();
        ImGui::PopID();
        ImGui::Checkbox("Publish RPMs at 10hz", &rpm_pub_enabled);
        ImGui::PopID();
        if (rpm_pub_enabled) { // && pub_timer == nullptr) {
            // pub_timer = new roswasm::Timer(0.08, std::bind(&SamActuatorWidget::pub_callback, this, std::placeholders::_1));
            pub_timer.start();
        }
        else { //if (!rpm_pub_enabled && pub_timer != nullptr) {
            // delete pub_timer;
            // pub_timer = nullptr;
            pub_timer.stop();
        }
    }

    if (ImGui::CollapsingHeader("Longitudinal Centre of Gravity (LCG)", ImGuiTreeNodeFlags_Framed | ImGuiTreeNodeFlags_DefaultOpen)) {
        ImGui::PushID("LCG");
        ImGui::Text("Pos (%%)");
        ImGui::SameLine();
        ImGui::PushID("Actuator");
        lcg_actuator->show_widget();
        ImGui::PopID();
        lcg_control_enable->show_widget();
        ImGui::Text("Pitch (rad)");
        ImGui::SameLine();
        ImGui::PushID("Control");
        lcg_control_setpoint->show_widget();
        ImGui::PopID();
        ImGui::PopID();
    }
    if (ImGui::CollapsingHeader("Variable Buoyancy System (VBS)", ImGuiTreeNodeFlags_Framed | ImGuiTreeNodeFlags_DefaultOpen)) {
        ImGui::PushID("VBS");
        ImGui::Text("Pos (%%)");
        ImGui::SameLine();
        ImGui::PushID("Actuator");
        vbs_actuator->show_widget();
        ImGui::PopID();
        vbs_control_enable->show_widget();
        ImGui::Text("Depth (m)");
        ImGui::SameLine();
        ImGui::PushID("Control");
        vbs_control_setpoint->show_widget();
        ImGui::PopID();
        ImGui::PopID();
    }
    if (ImGui::CollapsingHeader("Transversal Centre of Gravity (TCG)", ImGuiTreeNodeFlags_Framed | ImGuiTreeNodeFlags_DefaultOpen)) {
        ImGui::PushID("TCG");
        ImGui::Text("Pos (rad)");
        ImGui::SameLine();
        ImGui::PushID("Actuator");
        tcg_actuator->show_widget();
        ImGui::PopID();
        tcg_control_enable->show_widget();
        ImGui::Text("Roll (rad)");
        ImGui::SameLine();
        ImGui::PushID("Control");
        tcg_control_setpoint->show_widget();
        ImGui::PopID();
        ImGui::PopID();
    }

    ImGui::End();
}

SamDashboardWidget::SamDashboardWidget(roswasm::NodeHandle& nh) : was_leak(false)
{
    leak = new TopicBuffer<smarc_msgs::Leak>(nh, "core/leak");
    panic = new TopicBuffer<std_msgs::String>(nh, "core/panic_fb");
    gps = new TopicBuffer<sensor_msgs::NavSatFix>(nh, "core/gps");
    battery = new TopicBuffer<sensor_msgs::BatteryState>(nh, "core/battery");
    // odom = new TopicBuffer<nav_msgs::Odometry>(nh, "dr/odom", 1000);
    vbs_fb = new TopicBuffer<sam_msgs::PercentStamped>(nh, "core/vbs_fb", 1000);
    lcg = new TopicBuffer<sam_msgs::PercentStamped>(nh, "core/lcg_fb", 1000);
    rpms = new TopicBuffer<smarc_msgs::DualThrusterFeedback>(nh, "core/thrusters_fb", 1000);
    //rpms = new TopicBuffer<sam_msgs::ThrusterRPMs>(nh, "core/rpm_fb", 1000);
    // rpm1 = new TopicBuffer<smarc_msgs::ThrusterRPM>(nh, "core/thruster1_fb", 1000);
    // rpm2 = new TopicBuffer<smarc_msgs::ThrusterRPM>(nh, "core/thruster2_fb", 1000);
    dvl = new TopicBuffer<cola2_msgs::DVL>(nh, "core/dvl", 1000);
    odom_x = new TopicBuffer<std_msgs::Float64>(nh, "ctrl/odom_listener/x_feedback", 1000);
    odom_y = new TopicBuffer<std_msgs::Float64>(nh, "ctrl/odom_listener/y_feedback", 1000);
    depth = new TopicBuffer<std_msgs::Float64>(nh, "ctrl/odom_listener/depth_feedback", 1000);
    pitch = new TopicBuffer<std_msgs::Float64>(nh, "ctrl/odom_listener/pitch_feedback", 1000);
    roll = new TopicBuffer<std_msgs::Float64>(nh, "ctrl/odom_listener/roll_feedback", 1000);
    yaw = new TopicBuffer<std_msgs::Float64>(nh, "ctrl/odom_listener/yaw_feedback", 1000);
    motorTemp = new TopicBuffer<sensor_msgs::Temperature>(nh, "core/motor_temp", 1000);
}

void SamDashboardWidget::show_window(bool& show_dashboard_window)
{
    ImGui::Begin("Status dashboard", &show_dashboard_window);

    std::chrono::system_clock::time_point tp = std::chrono::system_clock::now();
    std::chrono::system_clock::duration dtn = tp.time_since_epoch();
    uint32_t current_time_epoch = dtn.count() * std::chrono::system_clock::period::num / std::chrono::system_clock::period::den;
    // if (strcmp(panic->get_msg().data.c_str(), "") || was_panic)
    if (!panic->get_msg().data.empty() || was_panic)
    {
        ImGui::PushID(65);
        char label[64];

        sprintf(label, ">>>>>>>>>> Panic!!!! Reason: %s <<<<<<<<<<", panic->get_msg().data.c_str());
        ImGui::PushStyleColor(ImGuiCol_Button, emergency_color);
        ImGui::PushStyleColor(ImGuiCol_ButtonHovered, emergency_color);
        ImGui::PushStyleColor(ImGuiCol_ButtonActive, emergency_color);
        ImGui::Button(label, ImVec2(ImGui::GetWindowContentRegionWidth(), 50));

        ImGui::PopStyleColor(3);
        ImGui::PopID();
        was_panic = true;
    }

    if (ImGui::CollapsingHeader("Critical Info", ImGuiTreeNodeFlags_Framed | ImGuiTreeNodeFlags_DefaultOpen)) {
        was_leak = was_leak || leak->get_msg().value;

        float sz = ImGui::GetTextLineHeight();
        std::string status_text;
        ImColor status_color;
        if (!was_leak) {
            status_text = "No leaks!";
            status_color = ImColor(0, 255, 0);
        }
        else {
            status_text = "Leak!!!!!";
            status_color = ImColor(255, 0, 0);
        }
        ImVec2 p = ImGui::GetCursorScreenPos();
        ImGui::GetWindowDrawList()->AddRectFilled(p, ImVec2(p.x+sz, p.y+sz), status_color);
        ImGui::Dummy(ImVec2(sz, sz));
        ImGui::SameLine();
        ImGui::Text("%s", status_text.c_str());

        ImGui::SameLine(150);
        ImGui::Text("Battery: %.0f%%", 100.*battery->get_msg().percentage);

        ImGui::SameLine(300);
        ImGui::Text("Motor temp: %.0f °C", motorTemp->get_msg().temperature-273.15);
    }

    if (ImGui::CollapsingHeader("GPS and depth", ImGuiTreeNodeFlags_Framed | ImGuiTreeNodeFlags_DefaultOpen)) {
        ImGui::Text("Lat: %.5f", gps->get_msg().latitude);
        ImGui::SameLine(150);
        ImGui::Text("Lon: %.5f", gps->get_msg().longitude);
        ImGui::SameLine(300);
        const uint32_t dvlMsgAgeThresh = 8;
        const bool dvlMsgOld = dvlMsgAgeThresh < current_time_epoch-dvl->get_msg().header.stamp.sec ? true : false;
        if (dvlMsgOld)
        {
            ImGui::Text("Altitude: ---");
        }
        else
        {
            ImGui::Text("Altitude: %.1fm", dvl->get_msg().altitude);
        }
    }

    if (ImGui::CollapsingHeader("DR translation", ImGuiTreeNodeFlags_Framed | ImGuiTreeNodeFlags_DefaultOpen)) {
        ImGui::Text("X: %.2fm", odom_x->get_msg().data);
        ImGui::SameLine(150);
        ImGui::Text("Y: %.2fm", odom_y->get_msg().data);
        ImGui::SameLine(300);
        ImGui::Text("Depth: %.2fm", depth->get_msg().data);
    }

    if (ImGui::CollapsingHeader("DR rotation", ImGuiTreeNodeFlags_Framed | ImGuiTreeNodeFlags_DefaultOpen)) {
        ImGui::Text("Roll: %.2fdeg", 180./M_PI*roll->get_msg().data);
        ImGui::SameLine(150);
        ImGui::Text("Pitch: %.2fdeg", 180./M_PI*pitch->get_msg().data);
        ImGui::SameLine(300);
        ImGui::Text("Yaw: %.2fdeg", 180./M_PI*yaw->get_msg().data);
    }

    if (ImGui::CollapsingHeader("Actuator feedback", ImGuiTreeNodeFlags_Framed | ImGuiTreeNodeFlags_DefaultOpen)) {
        ImGui::Text("VBS pos: %.2f%%", vbs_fb->get_msg().value);
        ImGui::SameLine(150);
        ImGui::Text("LCG pos: %.2f%%", lcg->get_msg().value);
        ImGui::SameLine(300);
        ImGui::Text("RPMs: F %d, B %d rpm", rpms->get_msg().thruster_front.rpm.rpm, rpms->get_msg().thruster_back.rpm.rpm);
        // ImGui::Text("RPMs: F %d, B %d rpm", rpm1->get_msg().rpm.rpm, rpm2->get_msg().rpm.rpm);
    }

    ImGui::End();
}


// SamDashboardWidget2::SamDashboardWidget2(roswasm::NodeHandle& nh) : was_leak(false)
// {
//     leak = new TopicBuffer<smarc_msgs::Leak>(nh, "core/leak_fb");
//     gps = new TopicBuffer<sensor_msgs::NavSatFix>(nh, "core/gps");
//     battery = new TopicBuffer<sensor_msgs::BatteryState>(nh, "core/battery_fb");
//     odom = new TopicBuffer<nav_msgs::Odometry>(nh, "dr/odom", 1000);
//     vbs_fb = new TopicBuffer<sam_msgs::PercentStamped>(nh, "core/vbs_fb", 1000);
//     lcg = new TopicBuffer<sam_msgs::PercentStamped>(nh, "core/lcg_fb", 1000);
//     // rpms = new TopicBuffer<smarc_msgs::DualThrusterFeedback>(nh, "core/thrusters_fb", 1000);
//     rpm1 = new TopicBuffer<smarc_msgs::ThrusterRPM>(nh, "core/thruster1_cmd", 1000);
//     rpm2 = new TopicBuffer<smarc_msgs::ThrusterRPM>(nh, "core/thruster2_cmd", 1000);
//     depth = new TopicBuffer<std_msgs::Float64>(nh, "ctrl/depth_feedback", 1000);
//     pitch = new TopicBuffer<std_msgs::Float64>(nh, "ctrl/pitch_feedback", 1000);
//     roll = new TopicBuffer<std_msgs::Float64>(nh, "ctrl/roll_feedback", 1000);
//     yaw = new TopicBuffer<std_msgs::Float64>(nh, "ctrl/yaw_feedback", 1000);
// }

// void SamDashboardWidget2::show_window(bool& show_dashboard_window)
// {
//     ImGui::SetNextWindowSize(ImVec2(500, 243), ImGuiCond_FirstUseEver);

//     ImGuiWindowFlags window_flags = 0;
//     window_flags |= ImGuiWindowFlags_MenuBar;
//     ImGui::Begin("Experiments dashboard", &show_dashboard_window, window_flags);

//     // Menu
//     if (ImGui::BeginMenuBar())
//     {
//         if (ImGui::BeginMenu("Strange"))
//         {
//             ImGui::MenuItem("Green");
//             ImGui::MenuItem("Fish");
//             // ImGui::MenuItem("Teleop", NULL, &show_teleop_window);
//             // ImGui::MenuItem("Main menu bar", NULL, &show_app_main_menu_bar);
//             // ImGui::MenuItem("Console", NULL, &show_app_console);
//             // ImGui::MenuItem("Log", NULL, &show_app_log);
//             // ImGui::MenuItem("Simple layout", NULL, &show_app_layout);
//             // ImGui::MenuItem("Property editor", NULL, &show_app_property_editor);
//             // ImGui::MenuItem("Long text display", NULL, &show_app_long_text);
//             // ImGui::MenuItem("Auto-resizing window", NULL, &show_app_auto_resize);
//             // ImGui::MenuItem("Constrained-resizing window", NULL, &show_app_constrained_resize);
//             // ImGui::MenuItem("Simple overlay", NULL, &show_app_simple_overlay);
//             // ImGui::MenuItem("Manipulating window titles", NULL, &show_app_window_titles);
//             // ImGui::MenuItem("Custom rendering", NULL, &show_app_custom_rendering);
//             ImGui::EndMenu();
//         }
//         ImGui::EndMenuBar();
//     }
    
//     static int selectedTab = 0;
//     const std::vector<const char*> tabNames{"tab1", "tab2", "tab3", "tab4"};
//     for (int i = 0; i < 4; i++)
//     {
//         if (i > 0) ImGui::SameLine();
//         ImGui::PushID(i);
//         if(selectedTab == i) {
//             // ImGui::PushStyleColor(ImGuiCol_Button, (ImVec4)ImColor::HSV(0.55f, 1.0f, 1.0f));
//             ImGui::PushStyleColor(ImGuiCol_Button, ImGui::GetColorU32(ImGuiCol_FrameBgActive));
//         } else {
//             // ImGui::PushStyleColor(ImGuiCol_Button, (ImVec4)ImColor::HSV(0.0f, 0.0f, 1.0f));
//             ImGui::PushStyleColor(ImGuiCol_Button, ImGui::GetColorU32(ImGuiCol_FrameBg));
//         }
//         // ImGui::PushStyleColor(ImGuiCol_ButtonHovered, (ImVec4)ImColor::HSV(0.55f, 0.4f, 1.0f));
//         // ImGui::PushStyleColor(ImGuiCol_ButtonActive, (ImVec4)ImColor::HSV(0.59f, 1.0f, 1.0f));
//         ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImGui::GetColorU32(ImGuiCol_FrameBgHovered));
//         ImGui::PushStyleColor(ImGuiCol_ButtonActive, ImGui::GetColorU32(ImGuiCol_FrameBgActive));
//         if(ImGui::Button(tabNames[i], ImVec2(50,20))){
//             selectedTab = i;
//         }
//         ImGui::PopStyleColor(3);
//         ImGui::PopID();
//     }
//     if(guiDebug)
//     {
//         ImGui::Text("Choosen tab: %d", selectedTab);
//     }

//     if(selectedTab == 1) {
//         if (ImGui::CollapsingHeader("Critical Info", ImGuiTreeNodeFlags_Framed | ImGuiTreeNodeFlags_DefaultOpen)) {
//             was_leak = was_leak || leak->get_msg().value;

//             float sz = ImGui::GetTextLineHeight();
//             std::string status_text;
//             ImColor status_color;
//             if (!was_leak) {
//                 status_text = "No leaks!";
//                 status_color = ImColor(0, 255, 0);
//             }
//             else {
//                 status_text = "Leak!!!!!";
//                 status_color = ImColor(255, 0, 0);
//             }
//             ImVec2 p = ImGui::GetCursorScreenPos();
//             ImGui::GetWindowDrawList()->AddRectFilled(p, ImVec2(p.x+sz, p.y+sz), status_color);
//             ImGui::Dummy(ImVec2(sz, sz));
//             ImGui::SameLine();
//             ImGui::Text("%s", status_text.c_str());

//             ImGui::SameLine(150);
//             ImGui::Text("Battery: %.0f%%", 100.*battery->get_msg().percentage);
//         }

//         if (ImGui::CollapsingHeader("GPS and depth", ImGuiTreeNodeFlags_Framed | ImGuiTreeNodeFlags_DefaultOpen)) {
//             ImGui::Text("Lat: %.5f", gps->get_msg().latitude);
//             ImGui::SameLine(150);
//             ImGui::Text("Lon: %.5f", gps->get_msg().longitude);
//             ImGui::SameLine(300);
//             ImGui::Text("Depth: %.2fm", depth->get_msg().data);
//         }

//         if (ImGui::CollapsingHeader("DR translation", ImGuiTreeNodeFlags_Framed | ImGuiTreeNodeFlags_DefaultOpen)) {
//             ImGui::Text("X: %.2fm", odom->get_msg().pose.pose.position.x);
//             ImGui::SameLine(150);
//             ImGui::Text("Y: %.2fm", odom->get_msg().pose.pose.position.y);
//             ImGui::SameLine(300);
//             ImGui::Text("Z: %.2fm", odom->get_msg().pose.pose.position.z);
//         }

//         if (ImGui::CollapsingHeader("DR rotation", ImGuiTreeNodeFlags_Framed | ImGuiTreeNodeFlags_DefaultOpen)) {
//             ImGui::Text("Roll: %.2fdeg", 180./M_PI*roll->get_msg().data);
//             ImGui::SameLine(150);
//             ImGui::Text("Pitch: %.2fdeg", 180./M_PI*pitch->get_msg().data);
//             ImGui::SameLine(300);
//             ImGui::Text("Yaw: %.2fdeg", 180./M_PI*yaw->get_msg().data);
//         }

//         if (ImGui::CollapsingHeader("Actuator feedback", ImGuiTreeNodeFlags_Framed | ImGuiTreeNodeFlags_DefaultOpen)) {
//             ImGui::Text("VBS pos: %.2f%%", vbs_fb->get_msg().value);
//             ImGui::SameLine(150);
//             ImGui::Text("LCG pos: %.2f%%", lcg->get_msg().value);
//             ImGui::SameLine(300);
//             ImGui::Text("RPMs: F %d, B %d rpm", rpms->get_msg().thruster_front.rpm.rpm, rpms->get_msg().thruster_back.rpm.rpm);
//         }
//     }

//     ImGui::End();
// }

// --------------------------- SamMonitorWidget ---------------------------
SamMonitorWidget::SamMonitorWidget(roswasm::NodeHandle& nh)
{
    battery = new TopicBuffer<sensor_msgs::BatteryState>(nh, "core/battery");
    circuit = new TopicBuffer<sam_msgs::CircuitStatusStampedArray>(nh, "core/circuit_status_array");
    subCharge = nh.subscribe<sam_msgs::ConsumedChargeArray>("core/consumed_charge_array", 10, &SamMonitorWidget::callbackCharge, this);
    // batteryService = nh.serviceClient<sam_msgs::UavcanUpdateBattery>("/sam/core/uavcan_update_battery", std::bind(&SamMonitorWidget::batteryCallback, this, std::placeholders::_1, std::placeholders::_2));
    batteryService = roswasm::createServiceCallbackClient<sam_msgs::UavcanUpdateBattery>(nh, "core/uavcan_update_battery"); //, &SamMonitorWidget::batteryCallback, this);
    uavcan = new TopicBuffer<uavcan_ros_msgs::UavcanNodeStatusNamedArray>(nh, "core/uavcan_status");
    // odom = new TopicBuffer<nav_msgs::Odometry>(nh, "dr/odom", 1000);
    vbs_fb = new TopicBuffer<sam_msgs::PercentStamped>(nh, "core/vbs_fb", 1000);
    vbs_cmd = new TopicBuffer<sam_msgs::PercentStamped>(nh, "core/vbs_cmd", 1000);
    vbs_pressure = new TopicBuffer<sensor_msgs::FluidPressure>(nh, "core/vbs_tank_pressure", 1000);
    vbs_temp = new TopicBuffer<sensor_msgs::Temperature>(nh, "core/vbs_tank_temperature", 1000);
    lcg = new TopicBuffer<sam_msgs::PercentStamped>(nh, "core/lcg_fb", 1000);
    thrusters_fb = new TopicBuffer<smarc_msgs::DualThrusterFeedback>(nh, "core/thrusters_fb", 1000);
    thrusters_cmd = new TopicBuffer<smarc_msgs::DualThrusterRPM>(nh, "core/thrusters_cmd", 1000);
    thruster1_fb = new TopicBuffer<smarc_msgs::ThrusterFeedback>(nh, "core/thruster1_fb", 1000);
    thruster2_fb = new TopicBuffer<smarc_msgs::ThrusterFeedback>(nh, "core/thruster2_fb", 1000);
    thruster1_cmd = new TopicBuffer<smarc_msgs::ThrusterRPM>(nh, "core/thruster1_cmd", 1000);
    thruster2_cmd = new TopicBuffer<smarc_msgs::ThrusterRPM>(nh, "core/thruster2_cmd", 1000);
    ctd = new TopicBuffer<smarc_msgs::CTD>(nh, "core/ctd", 1000);
    // DVL
    dvl = new TopicBuffer<cola2_msgs::DVL>(nh, "core/dvl", 1000);
    dvl_status_subscriber = nh.subscribe<smarc_msgs::SensorStatus>("core/dvl_status", 10, &SamMonitorWidget::dvl_status_callback, this);
    dvlEnableService = roswasm::createServiceCallbackClient<std_srvs::SetBool>(nh, "core/toggle_dvl");
    // SSS
    sss = new TopicBuffer<smarc_msgs::Sidescan>(nh, "payload/sidescan", 1000);
    sss_status_subscriber = nh.subscribe<smarc_msgs::SensorStatus>("payload/sidescan_status", 10, &SamMonitorWidget::sss_status_callback, this);
    sssEnableService = roswasm::createServiceCallbackClient<std_srvs::SetBool>(nh, "payload/toggle_sidescan");
    // first_service = nh.serviceClient<uavcan_ros_msgs::UavcanRestartNode>("/core/uavcan_restart_node", std::bind(&SamMonitorWidget::service_callback, this, std::placeholders::_1, std::placeholders::_2));
    // first_service = createServiceCallbackClient<uavcan_ros_msgs::UavcanRestartNode>(nh, "/core/uavcan_restart_node"); //, std::bind(&SamMonitorWidget::service_callback, this, std::placeholders::_1, std::placeholders::_2));
    first_service = roswasm::createServiceCallbackClient<uavcan_ros_msgs::UavcanRestartNode>(nh, "core/uavcan_restart_node");
    // first_service = nh.serviceClient<uavcan_ros_msgs::UavcanRestartNode>("/sam/core/uavcan_restart_node", std::bind(&SamMonitorWidget::service_callback, this, std::placeholders::_1, std::placeholders::_2));
    system = new TopicBuffer<diagnostic_msgs::DiagnosticArray>(nh, "core/jetson_diagnostics", 1000);
    subSystem = nh.subscribe<diagnostic_msgs::DiagnosticArray>("core/jetson_diagnostics", 10, &SamMonitorWidget::callbackSystem, this);

    depth = new TopicBuffer<std_msgs::Float64>(nh, "ctrl/depth_feedback", 1000);
    pitch = new TopicBuffer<std_msgs::Float64>(nh, "ctrl/pitch_feedback", 1000);
    roll = new TopicBuffer<std_msgs::Float64>(nh, "ctrl/roll_feedback", 1000);
    yaw = new TopicBuffer<std_msgs::Float64>(nh, "ctrl/yaw_feedback", 1000);
    motorTemp = new TopicBuffer<sensor_msgs::Temperature>(nh, "core/motor_temp", 1000);
    motorPressure = new TopicBuffer<sensor_msgs::FluidPressure>(nh, "core/motor_oil_pressure", 1000);
    sbg_euler = new TopicBuffer<sbg_driver::SbgEkfEuler>(nh, "sbg/ekf_euler", 1000);
}

void SamMonitorWidget::show_window(bool& show_dashboard_window, bool guiDebug)
{
    ImGui::SetNextWindowSize(ImVec2(1000, 500), ImGuiCond_FirstUseEver);
    ImGui::Begin("Monitoring dashboard", &show_dashboard_window);

    std::chrono::system_clock::time_point tp = std::chrono::system_clock::now();
    std::chrono::system_clock::duration dtn = tp.time_since_epoch();
    uint32_t current_time_epoch = dtn.count() * std::chrono::system_clock::period::num / std::chrono::system_clock::period::den;

    static int selectedTab = 0;
    const std::vector<const char*> tabNames{"Overview", "Electrical", "UAVCAN", "Jetson", "Comms", "Payloads"};
    for (int i = 0; i < tabNames.size(); i++)
    {
        if (i > 0) ImGui::SameLine();
        ImGui::PushID(i);
        if(selectedTab == i) {
            ImGui::PushStyleColor(ImGuiCol_Button, ImGui::GetColorU32(ImGuiCol_FrameBgActive));
        } else {
            ImGui::PushStyleColor(ImGuiCol_Button, ImGui::GetColorU32(ImGuiCol_FrameBg));
        }
        ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImGui::GetColorU32(ImGuiCol_FrameBgHovered));
        ImGui::PushStyleColor(ImGuiCol_ButtonActive, ImGui::GetColorU32(ImGuiCol_FrameBgActive));
        if(ImGui::Button(tabNames[i], ImVec2(85,20))){
            selectedTab = i;
        }
        ImGui::PopStyleColor(3);
        ImGui::PopID();
    }
    if(guiDebug)
    {
        ImGui::Text("Choosen tab: %d", selectedTab);
        ImGui::Text("%u", current_time_epoch);
    }

    // const std::vector<const char*> names{"Setup", "Monitor", "Control", "Service", "Experiments"};
    if(selectedTab == 1) {
        {
            ImGui::BeginChild("Battery", ImVec2(475, 80), false, 0);
            
            ImGui::Text("Batteries");
            const std::vector<std::pair<int, const char*>> batteryColumns{
                {30,"ID"},
                {70,"SoC [%]"},
                {90,"Voltage [V]"},
                {90,"Current [A]"},
                {90,"Charge [Ah]"},
                {105,"Capacity [Ah]"},
                // {70,"Status"},
                // {70,"Health"},
            };
            ImGui::Columns(batteryColumns.size(), "batteryTable");
            ImGui::Separator();
            
            for(int i = 0; i<batteryColumns.size(); i++)
            {
                ImGui::SetColumnWidth(i,batteryColumns[i].first); ImGui::Text("%s", batteryColumns[i].second); ImGui::NextColumn();
            }
            ImGui::Separator();

            ImGui::Text("1"); ImGui::NextColumn();
            ImGui::Text("%.1f", 100.*battery->get_msg().percentage); ImGui::NextColumn();
            ImGui::Text("%.2f", battery->get_msg().voltage); ImGui::NextColumn();
            ImGui::Text("%.2f", battery->get_msg().current); ImGui::NextColumn();
            ImGui::Text("%.2f", battery->get_msg().charge); ImGui::NextColumn();
            ImGui::Text("%.2f", battery->get_msg().capacity); ImGui::NextColumn();
            // ImGui::Text("UNKNOWN"); ImGui::NextColumn();
            // ImGui::Text("UNKNOWN"); ImGui::NextColumn();
            
            ImGui::EndChild();
            ImGui::SameLine();
            ImGui::BeginChild("Battery update", ImVec2(120, 80), false, 0);
            ImGui::Text("Update battery");
            ImGui::Separator();
            if (ImGui::Button("Update"))
            {
                batteryUpdateResponse = -1;
                ImGui::OpenPopup("batteryUpdate");
            }
            if (ImGui::BeginPopup("batteryUpdate"))
            {
                ImGui::BeginChild("Battery update context", ImVec2(200, 70), false, 0);
                ImGui::Text("Update battery charge");
                ImGui::Separator();
                sam_msgs::UavcanUpdateBattery::Request req;
                static int update_mode = 0;
                ImGui::Combo("", &update_mode, "Was full\0Is full\0Add charge\0On mains\0\0");
                if(update_mode == 0)
                {
                    req.command = 2;
                    req.charge = 0;
                }
                else if(update_mode == 1)
                {
                    req.command = 1;
                    req.charge = 0;
                }
                else if(update_mode == 2)
                {
                    // ImGui::SameLine();
                    static float f0 = 0.0f;
                    ImGui::InputFloat("Ah", &f0, 0.0f, 0.0f);
                    req.command = 3;
                    req.charge = f0;
                }
                else if(update_mode == 3)
                {
                    req.command = 4;
                    req.charge = 0;
                }

                ImGui::EndChild();
                if(ImGui::Button("SEND", ImVec2(100, 20)))
                {
                    batteryUpdateResponse = 0;
                    batteryService.call<sam_msgs::UavcanUpdateBattery>(req, std::bind(&SamMonitorWidget::batteryCallback, this, std::placeholders::_1, std::placeholders::_2));
                }
                ImGui::SameLine();
                if (batteryUpdateResponse == -2)
                {
                    ImGui::TextColored(ImVec4(1.0f, 0.0f, 0.0f, 1.00f), "Error!");
                }
                else if (batteryUpdateResponse == 1)
                {
                    ImGui::TextColored(ImVec4(0.0f, 0.71f, 0.06f, 1.00f), "Success!");
                }
                else if (batteryUpdateResponse == 2)
                {
                    ImGui::TextColored(ImVec4(0.87f, 0.57f, 0.0f, 1.00f), "Invalid!");
                }
                ImGui::EndPopup();
            }
            ImGui::EndChild();
        }
        ImGui::Columns(2, "rail_split", false);
        ImGui::SetColumnWidth(0, 545);
        {
            ImGui::BeginChild("RailsL", ImVec2(0, 200), false, 0);

            std::unordered_map<int, const char*> circuitNames = {
                {1,"Main"},
                {2,"ESC1"},
                {3,"ESC2"},
                {4,"ESC3"},
                {5,"20v"},
                {6,"12v"},
                {7,"7v"},
                {8,"5v"},
                {9,"3v3"}
            };

            // std::unordered_map<int, float> circuitCharges;
            // for

            int sizzy = circuit->get_msg().array.size();
            ImGui::Text("Power rails");
            const std::vector<std::pair<int, const char*>> circuitColumns{
                {30, "ID"},
                {45, "Rail"},
                {90, "Voltage [V]"},
                {90, "Current [A]"},
                {75, "Power [W]"},
                {100, "Status"},
                {100, "Consumed [Ah]"},
            };
            ImGui::Columns(circuitColumns.size(), "circuitTable");
            ImGui::Separator();
            
            for(int i = 0; i<circuitColumns.size(); i++)
            {
                ImGui::SetColumnWidth(i,circuitColumns[i].first); ImGui::Text("%s", circuitColumns[i].second); ImGui::NextColumn();
            }
            ImGui::Separator();

            std::pair<ImVec4, const char*> circuitModeToColoredString(const std::uint8_t mode);

            static int selectedRail = -1;
            for (int i = 0; i < circuit->get_msg().array.size(); i++)
            {
                const int id = circuit->get_msg().array[i].circuit.circuit_id;
                const float voltage = circuit->get_msg().array[i].circuit.voltage;
                const float current = circuit->get_msg().array[i].circuit.current;
                const float power = voltage*current;
                const std::pair<ImVec4, const char*> _mode = circuitModeToColoredString(circuit->get_msg().array[i].circuit.error_flags);
                char label[32];
                sprintf(label, "%d", id);
                if (ImGui::Selectable(label, selectedRail == id, ImGuiSelectableFlags_SpanAllColumns))
                {
                    selectedRail = id;
                }
                ImGui::NextColumn();
                if (circuitNames.find(id) != circuitNames.end())
                {
                    ImGui::Text("%s", circuitNames[id]);
                }
                else
                {
                    ImGui::Text("-");
                }
                ImGui::NextColumn();
                ImGui::Text("%.2f", voltage); ImGui::NextColumn();
                ImGui::Text("%.3f", current); ImGui::NextColumn();
                ImGui::Text("%.2f", power); ImGui::NextColumn();
                // ImGui::Text("%d", circuit->get_msg().array[i].circuit.error_flags); ImGui::NextColumn();
                ImGui::TextColored(_mode.first, "%s", _mode.second); ImGui::NextColumn();
                if (circuitCharges.find(id) != circuitCharges.end())
                {
                    ImGui::Text("%.4f", circuitCharges[id]);
                }
                else
                {
                    ImGui::Text("-");
                }
                ImGui::NextColumn();
            }
            ImGui::Columns(1);
            // ImGui::Separator();
            ImGui::EndChild();
        }
        // {
        //     ImGui::BeginChild("ChildR", ImVec2(ImGui::GetWindowContentRegionWidth() * 0.5f, 260), false, 0);
        //     ImGui::Text("Temperatures");
        //     ImGui::Columns(4, "mycolumns"); // 4-ways, with border
        //     ImGui::SetColumnWidth(0,50);
        //     ImGui::Separator();
        //     ImGui::Text("ID"); ImGui::NextColumn();
        //     ImGui::Text("Name"); ImGui::NextColumn();
        //     ImGui::Text("Path"); ImGui::NextColumn();
        //     ImGui::Text("Hovered"); ImGui::NextColumn();
        //     ImGui::Separator();
        //     // const char* names[3] = { "One", "Two", "Three" };
        //     const char* paths[3] = { "/path/one", "/path/two", "/path/three" };
        //     static int selected = -1;
        //     for (int i = 0; i < 3; i++)
        //     {
        //         char label[32];
        //         sprintf(label, "%04d", i);
        //         if (ImGui::Selectable(label, selected == i, ImGuiSelectableFlags_SpanAllColumns))
        //             selected = i;
        //         bool hovered = ImGui::IsItemHovered();
        //         ImGui::NextColumn();
        //         ImGui::Text("%s", names[i]); ImGui::NextColumn();
        //         ImGui::Text("%s", names[i]); ImGui::NextColumn();
        //         ImGui::Text("%d", hovered); ImGui::NextColumn();
        //     }
        //     ImGui::Columns(1);
        //     ImGui::EndChild();
        // }
        ImGui::NextColumn();
        // ImGui::SameLine();
        {
            ImGui::BeginChild("RailsR", ImGui::GetContentRegionAvail(), false, 0);
            ImGui::Text("Power details");
            ImGui::BeginChild("RailsR2", ImVec2(0, 0), true, 0);
            ImGui::EndChild();
            ImGui::EndChild();
        }
    }
    else if(selectedTab == 2)
    {
        static int selectedUavcan = -1;
        {
            std::pair<ImVec4, const char*> healthToColoredString(const std::uint8_t health);
            std::pair<ImVec4, const char*> modeToColoredString(const std::uint8_t mode);

            ImVec2 _avail = ImGui::GetContentRegionAvail();
            ImGui::BeginChild("UAVCAN", ImVec2(645, _avail[1]), false, 0);
            int sizzy = uavcan->get_msg().array.size();

            ImGui::Text("UAVCAN network");
            const std::vector<std::pair<int, const char*>> uavcanColumns{
                {35, "ID"},
                {260,"Name"},
                {85, "Uptime"},
                {70, "Health"},
                {120,"Mode"},
                {75, "VSSC"},
            };
            ImGui::Columns(uavcanColumns.size(), "uavcanTable");
            ImGui::Separator();
            
            for(int i = 0; i<uavcanColumns.size(); i++)
            {
                ImGui::SetColumnWidth(i,uavcanColumns[i].first); ImGui::Text("%s", uavcanColumns[i].second); ImGui::NextColumn();
            }
            ImGui::Separator();

            for (int i = 0; i < uavcan->get_msg().array.size(); i++)
            {
                const std::pair<ImVec4, const char*> _health = healthToColoredString(uavcan->get_msg().array[i].ns.health);
                const std::pair<ImVec4, const char*> _mode = modeToColoredString(uavcan->get_msg().array[i].ns.mode);
                char label[32];
                sprintf(label, "%d", uavcan->get_msg().array[i].id);
                if (ImGui::Selectable(label, selectedUavcan == i, ImGuiSelectableFlags_SpanAllColumns))
                {
                    selectedUavcan = i;
                }
                ImGui::NextColumn();
                if(uavcan->get_msg().array[i].id == 98)
                {
                    ImGui::Text("smarc.sam.uavcan_bridge.publisher");
                }
                else if(uavcan->get_msg().array[i].id == 100)
                {
                    ImGui::Text("smarc.sam.uavcan_bridge.services");
                }
                else
                {
                    ImGui::Text("%s", uavcan->get_msg().array[i].name.c_str());
                }
                ImGui::NextColumn();
                const uint32_t uptime_s = uavcan->get_msg().array[i].ns.uptime_sec;
                const uint32_t min_s = 60;
                const uint32_t hour_s = 60*60;
                const uint32_t day_s = 60*60*24;
                const uint32_t year_s = 60*60*24*365;
                if (uptime_s >= 0 && uptime_s < min_s)
                {
                    ImGui::Text("%ds", uptime_s); ImGui::NextColumn();
                }
                else if (uptime_s >= min_s && uptime_s < hour_s)
                {
                    ImGui::Text("%d:%02d", uptime_s/min_s, uptime_s%min_s); ImGui::NextColumn();
                }
                else if (uptime_s >= hour_s && uptime_s < 3*day_s)
                {
                    ImGui::Text("%d:%02d:%02d", uptime_s/hour_s, (uptime_s%hour_s)/min_s, ((uptime_s%hour_s)%min_s)%min_s); ImGui::NextColumn();
                }
                else if (uptime_s >= 3*day_s)
                {
                    ImGui::Text("%d days", uptime_s/day_s); ImGui::NextColumn();
                }
                else
                {
                    ImGui::Text("unknown"); ImGui::NextColumn();
                }
                ImGui::TextColored(_health.first, "%s", _health.second); ImGui::NextColumn();
                ImGui::TextColored(_mode.first, "%s", _mode.second); ImGui::NextColumn();
                ImGui::Text("0x%04X", uavcan->get_msg().array[i].ns.vendor_specific_status_code); ImGui::NextColumn();
            }
            ImGui::Columns(1);
            ImGui::EndChild();
        }
            ImGui::SameLine();
        {
            ImGui::BeginChild("UAVCAN2", ImGui::GetContentRegionAvail(), false, 0);
                ImGui::Text("UAVCAN details");
                ImGui::BeginChild("UAVCAN3", ImVec2(0, 0), true, 0);

                if (selectedUavcan != -1)
                {
                    const uint8_t _id = uavcan->get_msg().array[selectedUavcan].id;
                    if (97 <= _id && _id <= 100)
                    {
                        // Ros nodes
                        ImGui::Text("This is a ROS node");
                    }
                    else if (_id == 1)
                    {
                        // Time master
                        ImGui::Text("This is a time master");
                    }
                    else if (_id == 127)
                    {
                        // Babel
                        ImGui::Text("This is a Babel");
                    }
                    else
                    {
                        ImGui::Columns(2, "uavcanTable2", false);
                        ImGui::SetColumnWidth(0, 40);
                        ImGui::Text("ID"); ImGui::NextColumn();
                        ImGui::Text("Name"); ImGui::NextColumn();
                        ImGui::Separator();
                        ImGui::Text("%d", _id); ImGui::NextColumn();
                        ImGui::Text("%s", uavcan->get_msg().array[selectedUavcan].name.c_str());
                        ImGui::Separator();
                        ImGui::Columns(1);
                        if (97 <= _id && _id <= 100)
                        {
                            // Ros nodes
                        }
                        else
                        {
                            if(ImGui::Button("RESTART", ImVec2(60, 20)))
                            {
                                uavcan_ros_msgs::UavcanRestartNode::Request req;
                                req.node_id = uavcan->get_msg().array[selectedUavcan].id;
                                first_service.call<uavcan_ros_msgs::UavcanRestartNode>(req, std::bind(&SamMonitorWidget::service_callback, this, std::placeholders::_1, std::placeholders::_2));
                            }
                        }
                    }
                }
                else
                {
                    ImGui::Text("No node selected :(");
                }
                ImGui::EndChild();
            ImGui::EndChild();
        }
        // {

        //     // ImGui::BeginChild("ChildL", ImVec2(ImGui::GetWindowContentRegionWidth() * 0.5f, 300), false, ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoScrollWithMouse);
        //     ImGui::BeginChild("ChildL", ImVec2(ImGui::GetWindowContentRegionWidth() * 0.5f, 300), false, 0);

        //     ImGui::Text("Batteries");
        //     const std::vector<std::pair<int, const char*>> circuitColumns{
        //         {30,"ID"},
        //         {70,"Rail [V]"},
        //         {90,"Voltage [V]"},
        //         {90,"Current [A]"},
        //         {70,"Status"},
        //     };
        //     ImGui::Columns(circuitColumns.size(), "circuitTable");
        //     ImGui::Separator();
            
        //     for(int i = 0; i<circuitColumns.size(); i++)
        //     {
        //         ImGui::SetColumnWidth(i,circuitColumns[i].first); ImGui::Text("%s", circuitColumns[i].second); ImGui::NextColumn();
        //     }
        //     ImGui::Separator();
        //     ImGui::Text("%d", circuit->get_msg().circuit_id); ImGui::NextColumn();
        //     ImGui::Text("name"); ImGui::NextColumn();
        //     ImGui::Text("%.2f", circuit->get_msg().voltage); ImGui::NextColumn();
        //     ImGui::Text("%.2f", circuit->get_msg().current); ImGui::NextColumn();
        //     ImGui::Text("%d", circuit->get_msg().error_flags); ImGui::NextColumn();
        //     // ImGui::Text("With border:");
        //     // ImGui::Columns(4, "mycolumns"); // 4-ways, with border
        //     // ImGui::SetColumnWidth(0,50);
        //     // ImGui::Separator();
        //     // ImGui::Text("ID"); ImGui::NextColumn();
        //     // ImGui::Text("Name"); ImGui::NextColumn();
        //     // ImGui::Text("Path"); ImGui::NextColumn();
        //     // ImGui::Text("Hovered"); ImGui::NextColumn();
        //     // ImGui::Separator();
        //     // const char* names[3] = { "One", "Two", "Three" };
        //     // const char* paths[3] = { "/path/one", "/path/two", "/path/three" };
        //     static int selected = -1;
        //     // for (int i = 0; i < 2; i++)
        //     // {
        //     //     char label[32];
        //     //     sprintf(label, "%04d", i);
        //     //     if (ImGui::Selectable(label, selected == i, ImGuiSelectableFlags_SpanAllColumns))
        //     //         selected = i;
        //     //     bool hovered = ImGui::IsItemHovered();
        //     //     ImGui::NextColumn();
        //     //     ImGui::TextColored(ImGui::GetStyleColorVec4(ImGuiCol_Text), "%s", names[i]); ImGui::NextColumn();
        //     //     ImGui::TextColored(ImGui::GetStyleColorVec4(ImGuiCol_Text), "%s", names[i]); ImGui::NextColumn();
        //     //     ImGui::Text("%d", hovered); ImGui::NextColumn();
        //     // }
        //     ImGui::Columns(1);
        //     ImGui::EndChild();
        // }
        // // ImGui::SameLine();
        // {
        //     // ImGui::BeginChild("ChildR", ImVec2(0, 260), false, ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoScrollWithMouse);
        //     ImGui::BeginChild("ChildR", ImVec2(0, 260), false, 0);
        //     ImGui::Text("Temperatures");
        //     ImGui::Columns(4, "mycolumns"); // 4-ways, with border
        //     ImGui::SetColumnWidth(0,50);
        //     ImGui::Separator();
        //     ImGui::Text("ID"); ImGui::NextColumn();
        //     ImGui::Text("Name"); ImGui::NextColumn();
        //     ImGui::Text("Path"); ImGui::NextColumn();
        //     ImGui::Text("Hovered"); ImGui::NextColumn();
        //     ImGui::Separator();
        //     // const char* names[3] = { "One", "Two", "Three" };
        //     const char* paths[3] = { "/path/one", "/path/two", "/path/three" };
        //     static int selected = -1;
        //     for (int i = 0; i < 3; i++)
        //     {
        //         char label[32];
        //         sprintf(label, "%04d", i);
        //         if (ImGui::Selectable(label, selected == i, ImGuiSelectableFlags_SpanAllColumns))
        //             selected = i;
        //         bool hovered = ImGui::IsItemHovered();
        //         ImGui::NextColumn();
        //         ImGui::Text("%s", names[i]); ImGui::NextColumn();
        //         ImGui::Text("%s", names[i]); ImGui::NextColumn();
        //         ImGui::Text("%d", hovered); ImGui::NextColumn();
        //     }
        //     ImGui::Columns(1);
        //     ImGui::EndChild();
        // }
    } else if(selectedTab == 0) {
        const int subWindowHeight = 220;
        {
            // ImVec2 motorWindowPos = ImGui::GetCursorScreenPos();
            // ImVec2 motorOrigin = ImGui::GetCursorPos();
            
            // ImGui::PushStyleColor(ImGuiCol_ChildBg, warning_color);
            ImGui::BeginChild("Motors", ImVec2(210, subWindowHeight), true, 0);
            // ImGui::PopStyleColor(1);
            {
                ImGui::BeginChild("motorTitle", ImVec2(60, 20), false, 0);
                ImGui::Text("Motors");
                ImGui::Separator();
                ImGui::EndChild();
            }
            // ImGui::PushID(456);
            // ImGui::PopStyleColor(1);
            // ImGui::PopID();
            // {
            //     ImGui::BeginChild("motorMain", ImVec2(300, 130), false, 0);
                ImVec2 motorWindowPos = ImGui::GetCursorScreenPos();
                ImVec2 motorOrigin = ImGui::GetCursorPos();

                const float rangeBar = 1500.0f;

                const uint32_t motorMsgAgeThresh = 8;
                const bool motorMsgOld = motorMsgAgeThresh < current_time_epoch-thrusters_fb->get_msg().thruster_front.header.stamp.sec ? true : false;
                const int v_fb = motorMsgOld ? 0 : thrusters_fb->get_msg().thruster_front.rpm.rpm;
                const int v_fb2 = motorMsgOld ? 0 : thrusters_fb->get_msg().thruster_back.rpm.rpm;

                const int v_cmd = thrusters_cmd->get_msg().thruster_front.rpm ? thrusters_cmd->get_msg().thruster_front.rpm : 0;
                const int v_cmd2 = thrusters_cmd->get_msg().thruster_back.rpm ? thrusters_cmd->get_msg().thruster_back.rpm : 0;

                // const bool motorMsgOld1 = motorMsgAgeThresh < current_time_epoch-thruster1_fb->get_msg().header.stamp.sec ? true : false;
                // const int v_fb = motorMsgOld1 ? 0 : thruster1_fb->get_msg().rpm.rpm;
                // const bool motorMsgOld2 = motorMsgAgeThresh < current_time_epoch-thruster1_fb->get_msg().header.stamp.sec ? true : false;
                // const int v_fb2 = motorMsgOld2 ? 0 : thruster2_fb->get_msg().rpm.rpm;

                // const int v_cmd = thruster1_cmd->get_msg().rpm ? thruster1_cmd->get_msg().rpm : 0;
                // const int v_cmd2 = thruster2_cmd->get_msg().rpm ? thruster2_cmd->get_msg().rpm : 0;
                
                // static int v_fb = 0;
                // ImGui::SliderInt("fb", &v_fb, -1000, 1000);
                // static int v_cmd = 0;
                // ImGui::SliderInt("cmd", &v_cmd, -1000, 1000);
                // const int M2Offset = 200;
                // ImGui::Text("Front %c", "|/-\\"[(int)(ImGui::GetTime() / 0.05f) & 3]); ImGui::SameLine(M2Offset);
                // ImGui::Text("Rear %c", "|/-\\"[(int)(ImGui::GetTime() / 0.05f) & 3]);
                // ImGui::Text("Torque"); ImGui::SameLine(M2Offset);
                ImGui::Text("Pressure");
                ImGui::Text("%.2f bar", motorPressure->get_msg().fluid_pressure/100000.0f);
                // ImGui::Text("Torque");
                // ImGui::Text("%.2f Nm", thrusters_fb->get_msg().thruster_front.torque); ImGui::SameLine(M2Offset);
                // ImGui::Text("%.2f Nm", thrusters_fb->get_msg().thruster_back.torque);
                // ImGui::Text("Current"); ImGui::SameLine(M2Offset);
                // ImGui::Text("Current");
                // ImGui::Text("%.2f A", thrusters_fb->get_msg().thruster_front.current); ImGui::SameLine(M2Offset);
                // ImGui::Text("%.2f A", thrusters_fb->get_msg().thruster_back.current);
                // ImGui::Text("Temp"); ImGui::SameLine(M2Offset);
                // ImGui::Text("Temp");
                // ImGui::Text("%.0f °C", motorTemp->get_msg().temperature-273.15); ImGui::SameLine(M2Offset);
                // ImGui::Text("%.0f °C", motorTemp->get_msg().temperature-273.15);
                // ImGui::SameLine();
                const int motorOffset = 100;

                const ImVec2 size = ImVec2(40, 100);
                // draw_bar_signed(pos, size, v_fb, v_cmd, rangeBar, true);
                const ImVec2 pos = ImVec2(motorOffset, -10);
                {
                    // const ImVec2 pos = ImVec2(motorOffset, -10);

                    const ImVec2 barPos = ImVec2(motorWindowPos.x + pos.x, motorWindowPos.y + pos.y);
                    const bool outOfRange = std::abs(v_fb) > rangeBar ? true : false;
                    const float frac_fb =  (outOfRange) ? (v_fb < 0 ? -1.0f : 1.0f) : v_fb / rangeBar;
                    const float frac_cmd =  (std::abs(v_cmd) > rangeBar) ? (v_cmd < 0 ? -1.0f : 1.0f) : v_cmd / rangeBar;

                    const ImVec2 barLength = ImVec2((size.y/2-3)*frac_fb, (size.y/2-3)*frac_cmd);
                    const ImVec2 bar_p1 = ImVec2(barPos.x + 3, barPos.y + size.y/2);
                    const ImVec2 bar_p2 = ImVec2(barPos.x + size.x - 3, barPos.y + size.y/2);
                    if (outOfRange)
                    {
                        ImGui::GetWindowDrawList()->AddRectFilled(bar_p1, ImVec2(bar_p2.x, bar_p2.y - barLength[0]), IM_COL32(195,0,0,255)); // fill
                    }
                    else
                    {
                        ImGui::GetWindowDrawList()->AddRectFilled(bar_p1, ImVec2(bar_p2.x, bar_p2.y - barLength[0]), ImGui::GetColorU32(ImGuiCol_PlotHistogram)); // fill
                    }
                    ImGui::GetWindowDrawList()->AddLine(ImVec2(barPos.x + 2, bar_p2.y - barLength[1]), ImVec2(barPos.x + size.x - 2, bar_p2.y - barLength[1]), IM_COL32(255,0,0,255), 3); // line
                    ImGui::GetWindowDrawList()->AddRect(barPos, ImVec2(barPos.x + size.x, barPos.y + size.y), ImGui::GetColorU32(ImGuiCol_FrameBgActive)); // border
                    // ImGui::Dummy(size);

                    char label1[10];
                    sprintf(label1, "%d", v_fb);
                    const ImVec2 barLabelPos = ImVec2(motorOrigin.x + pos.x + size.x / 2, motorOrigin.y + pos.y + size.y);
                    ImGui::SetCursorPos(ImVec2(barLabelPos.x - ImGui::CalcTextSize(label1).x / 2, barLabelPos.y));
                    ImGui::Text("%s", label1);

                    const ImVec2 m1LabelPos = ImVec2(motorOrigin.x + pos.x + size.x / 2, motorOrigin.y + pos.y - 15);
                    ImGui::SetCursorPos(ImVec2(m1LabelPos.x - ImGui::CalcTextSize("Front").x / 2, m1LabelPos.y));
                    ImGui::Text("Front");
                }

                {
                    const ImVec2 pos2 = ImVec2(motorOffset + 50, -10);

                    const ImVec2 barPos2 = ImVec2(motorWindowPos.x + pos2.x, motorWindowPos.y + pos2.y);
                    const bool outOfRange2 = std::abs(v_fb2) > rangeBar ? true : false;
                    const float frac_fb2 =  (outOfRange2) ? (v_fb2 < 0 ? -1.0f : 1.0f) : v_fb2 / rangeBar;
                    const float frac_cmd2 =  (std::abs(v_cmd2) > rangeBar) ? (v_cmd2 < 0 ? -1.0f : 1.0f) : v_cmd2 / rangeBar;

                    const ImVec2 barLength2 = ImVec2((size.y/2-3)*frac_fb2, (size.y/2-3)*frac_cmd2);
                    const ImVec2 bar_p12 = ImVec2(barPos2.x + 3, barPos2.y + size.y/2);
                    const ImVec2 bar_p22 = ImVec2(barPos2.x + size.x - 3, barPos2.y + size.y/2);
                    if (outOfRange2)
                    {
                        ImGui::GetWindowDrawList()->AddRectFilled(bar_p12, ImVec2(bar_p22.x, bar_p22.y - barLength2[0]), IM_COL32(195,0,0,255)); // fill
                    }
                    else
                    {
                        ImGui::GetWindowDrawList()->AddRectFilled(bar_p12, ImVec2(bar_p22.x, bar_p22.y - barLength2[0]), ImGui::GetColorU32(ImGuiCol_PlotHistogram)); // fill
                    }
                    ImGui::GetWindowDrawList()->AddLine(ImVec2(barPos2.x + 2, bar_p22.y - barLength2[1]), ImVec2(barPos2.x + size.x - 2, bar_p22.y - barLength2[1]), IM_COL32(255,0,0,255), 3); // line
                    ImGui::GetWindowDrawList()->AddRect(barPos2, ImVec2(barPos2.x + size.x, barPos2.y + size.y), ImGui::GetColorU32(ImGuiCol_FrameBgActive)); // border
                    // ImGui::Dummy(size);

                    char label12[10];
                    sprintf(label12, "%d", v_fb2);
                    const ImVec2 barLabelPos2 = ImVec2(motorOrigin.x + pos2.x + size.x / 2, motorOrigin.y + pos2.y + size.y);
                    ImGui::SetCursorPos(ImVec2(barLabelPos2.x - ImGui::CalcTextSize(label12).x / 2, barLabelPos2.y));
                    ImGui::Text("%s", label12);

                    const ImVec2 m2LabelPos = ImVec2(motorOrigin.x + pos2.x + size.x / 2, motorOrigin.y + pos2.y - 15);
                    ImGui::SetCursorPos(ImVec2(m2LabelPos.x - ImGui::CalcTextSize("Back").x / 2, m2LabelPos.y));
                    ImGui::Text("Back");
                }
                {
                    ImGui::SetCursorPosY(motorOrigin.y + pos.y + size.y + 20);
                    ImGui::Separator();
                    ImGui::Columns(3, "motorTable", false);
                    ImGui::SetColumnWidth(0, motorOffset);
                    ImGui::SetColumnWidth(1, 54);

                    ImGui::Text("Torque  [Nm]"); ImGui::NextColumn();
                    char label1[10];
                    sprintf(label1, "%.2f", thrusters_fb->get_msg().thruster_front.torque);
                    ImGui::SetCursorPosX(ImGui::GetCursorPosX() + ImGui::GetColumnWidth() - ImGui::CalcTextSize(label1).x - ImGui::GetScrollX() - 2 * ImGui::GetStyle().ItemSpacing.x);
                    ImGui::Text("%s", label1); ImGui::NextColumn();
                    ImGui::Text("%.2f", thrusters_fb->get_msg().thruster_back.torque); ImGui::NextColumn();

                    ImGui::Text("Current [A]"); ImGui::NextColumn();
                    sprintf(label1, "%.1f", thrusters_fb->get_msg().thruster_front.current);
                    ImGui::SetCursorPosX(ImGui::GetCursorPosX() + ImGui::GetColumnWidth() - ImGui::CalcTextSize(label1).x - ImGui::GetScrollX() - 2 * ImGui::GetStyle().ItemSpacing.x);
                    ImGui::Text("%s", label1); ImGui::NextColumn();
                    ImGui::Text("%.1f", thrusters_fb->get_msg().thruster_back.current); ImGui::NextColumn();

                    ImGui::Text("Temp    [°C]"); ImGui::NextColumn();
                    sprintf(label1, "%.0f", motorTemp->get_msg().temperature-273.15);
                    ImGui::SetCursorPosX(ImGui::GetCursorPosX() + ImGui::GetColumnWidth() - ImGui::CalcTextSize(label1).x - ImGui::GetScrollX() - 2 * ImGui::GetStyle().ItemSpacing.x);
                    ImGui::Text("%s", label1); ImGui::NextColumn();
                    ImGui::Text("%.0f", motorTemp->get_msg().temperature-273.15);
                    ImGui::Columns(1);
                }
                /* separate motor msgs
                {
                    ImGui::SetCursorPosY(motorOrigin.y + pos.y + size.y + 20);
                    ImGui::Separator();
                    ImGui::Columns(3, "motorTable", false);
                    ImGui::SetColumnWidth(0, motorOffset);
                    ImGui::SetColumnWidth(1, 54);
                    // ImGui::SetColumnWidth(2, 50);

                    ImGui::Text("Torque  [Nm]"); ImGui::NextColumn();
                    char label1[10];
                    sprintf(label1, "%.2f", thruster1_fb->get_msg().torque);
                    ImGui::SetCursorPosX(ImGui::GetCursorPosX() + ImGui::GetColumnWidth() - ImGui::CalcTextSize(label1).x - ImGui::GetScrollX() - 2 * ImGui::GetStyle().ItemSpacing.x);
                    ImGui::Text("%s", label1); ImGui::NextColumn();
                    // sprintf(label1, "%.2f", thrusters_fb->get_msg().thruster_back.torque);
                    // ImGui::SetCursorPosX(ImGui::GetCursorPosX() + ImGui::GetColumnWidth() - ImGui::CalcTextSize(label1).x - ImGui::GetScrollX() - 2 * ImGui::GetStyle().ItemSpacing.x);
                    // ImGui::Text("%s", label1); ImGui::NextColumn();
                    ImGui::Text("%.2f", thruster2_fb->get_msg().torque); ImGui::NextColumn();

                    ImGui::Text("Current [A]"); ImGui::NextColumn();
                    sprintf(label1, "%.1f", thruster1_fb->get_msg().current);
                    ImGui::SetCursorPosX(ImGui::GetCursorPosX() + ImGui::GetColumnWidth() - ImGui::CalcTextSize(label1).x - ImGui::GetScrollX() - 2 * ImGui::GetStyle().ItemSpacing.x);
                    ImGui::Text("%s", label1); ImGui::NextColumn();
                    // sprintf(label1, "%.1f", thrusters_fb->get_msg().thruster_back.current);
                    // ImGui::SetCursorPosX(ImGui::GetCursorPosX() + ImGui::GetColumnWidth() - ImGui::CalcTextSize(label1).x - ImGui::GetScrollX() - 2 * ImGui::GetStyle().ItemSpacing.x);
                    // ImGui::Text("%s", label1); ImGui::NextColumn();
                    // ImGui::Text("%.1f", thrusters_fb->get_msg().thruster_front.current); ImGui::NextColumn();
                    ImGui::Text("%.1f", thruster2_fb->get_msg().current); ImGui::NextColumn();

                    ImGui::Text("Temp    [°C]"); ImGui::NextColumn();
                    sprintf(label1, "%.0f", motorTemp->get_msg().temperature-273.15);
                    ImGui::SetCursorPosX(ImGui::GetCursorPosX() + ImGui::GetColumnWidth() - ImGui::CalcTextSize(label1).x - ImGui::GetScrollX() - 2 * ImGui::GetStyle().ItemSpacing.x);
                    ImGui::Text("%s", label1); ImGui::NextColumn();
                    // ImGui::SetCursorPosX(ImGui::GetCursorPosX() + ImGui::GetColumnWidth() - ImGui::CalcTextSize(label1).x - ImGui::GetScrollX() - 2 * ImGui::GetStyle().ItemSpacing.x);
                    // ImGui::Text("%s", label1); ImGui::NextColumn();
                    // ImGui::Text("%.0f", motorTemp->get_msg().temperature-273.15); ImGui::NextColumn();
                    ImGui::Text("%.0f", motorTemp->get_msg().temperature-273.15);
                    ImGui::Columns(1);
                }*/






            // }
            // ImGui::Text("System: %u", current_time_epoch);
            // ImGui::Text("Stamp:  %u", thrusters_fb->get_msg().thruster_front.header.stamp.sec);
            // ImGui::Text("diff:   %u", current_time_epoch-thrusters_fb->get_msg().thruster_front.header.stamp.sec);


            // ImGui::Text("Motors");

            // std::string text = "1";
            // ImGui::SetCursorPosX(ImGui::GetCursorPosX() + ImGui::GetColumnWidth() - ImGui::CalcTextSize(text.c_str()).x - ImGui::GetScrollX() - 2 * ImGui::GetStyle().ItemSpacing.x);
            // ImGui::Text("%s", text);

            // const std::vector<std::pair<int, const char*>> batteryColumns{
            //     {30,"ID"},
            //     {70,"SoC [%]"},
            //     {90,"Voltage [V]"},
            //     {90,"Current [A]"},
            //     {90,"Charge [Ah]"},
            //     {105,"Capacity [Ah]"},
            //     // {70,"Status"},
            //     // {70,"Health"},
            // };
            // ImGui::Columns(batteryColumns.size(), "batteryTable");
            // ImGui::Separator();
            
            // for(int i = 0; i<batteryColumns.size(); i++)
            // {
            //     ImGui::SetColumnWidth(i,batteryColumns[i].first); ImGui::Text("%s", batteryColumns[i].second); ImGui::NextColumn();
            // }
            // ImGui::Separator();

            // ImGui::Text("1"); ImGui::NextColumn();
            // ImGui::Text("%.1f", 100.*battery->get_msg().percentage); ImGui::NextColumn();
            // ImGui::Text("%.2f", battery->get_msg().voltage); ImGui::NextColumn();
            // ImGui::Text("%.2f", battery->get_msg().current); ImGui::NextColumn();
            // ImGui::Text("%.2f", battery->get_msg().charge); ImGui::NextColumn();
            // ImGui::Text("%.2f", battery->get_msg().capacity); ImGui::NextColumn();
            // // ImGui::Text("UNKNOWN"); ImGui::NextColumn();
            // // ImGui::Text("UNKNOWN"); ImGui::NextColumn();
            ImGui::EndChild();
        }
        ImGui::SameLine();
        {
            const int ctdWidth = 200;
            ImGui::BeginChild("CTD", ImVec2(ctdWidth, subWindowHeight), true, 0);

            const uint32_t ctdMsgAgeThresh = 8;
            const bool ctdMsgOld = ctdMsgAgeThresh < current_time_epoch-ctd->get_msg().header.stamp.sec ? true : false;
            std::string status_text;
            ImVec4 status_color4;
            char label1[10];
            if (ctdMsgOld)
            {
                sprintf(label1, "%s", "Inactive");
                status_color4 = warning_color;
            }
            else
            {
                sprintf(label1, "%s", "Active");
                status_color4 = good_color;
            }

            ImGui::Columns(3, "ctdTitle", false);
            ImGui::SetColumnWidth(0,50);
            ImGui::SetColumnWidth(1, ctdWidth - 80);
            ImGui::Text("CTD"); ImGui::NextColumn();
            ImGui::SetCursorPosX(ImGui::GetCursorPosX() + ImGui::GetColumnWidth() - ImGui::CalcTextSize(label1).x - ImGui::GetScrollX() - 2 * ImGui::GetStyle().ItemSpacing.x);
            ImGui::Text("%s", label1); ImGui::NextColumn();
            ImGui::PushID(26);
            ImGui::PushStyleColor(ImGuiCol_Button, status_color4);
            ImGui::PushStyleColor(ImGuiCol_ButtonHovered, status_color4);
            ImGui::PushStyleColor(ImGuiCol_ButtonActive, status_color4);
            ImGui::Button("", ImVec2(15,15));
            ImGui::PopStyleColor(3);
            ImGui::PopID();
            ImGui::Columns(1);

            // ImGui::Text("CTD");
            ImGui::Separator();
            ImGui::Text("Conductivity %.2f mS/cm", ctd->get_msg().conductivity);
            ImGui::Text("Temperature  %.2f °C", ctd->get_msg().temperature);
            ImGui::Text("Depth        %.2f m", ctd->get_msg().depth);
            ImGui::EndChild();
        }
        ImGui::SameLine();
        {
            const int dvlWidth = 200;
            ImGui::BeginChild("DVL", ImVec2(dvlWidth, subWindowHeight), true, 0);

            const uint32_t dvlMsgAgeThresh = 8;
            const bool dvlMsgOld = dvlMsgAgeThresh < current_time_epoch-dvl->get_msg().header.stamp.sec ? true : false;
            std::string status_text;
            ImVec4 status_color4;
            char label1[10];
            if (dvlMsgOld)
            {
                sprintf(label1, "%s", "Inactive");
                status_color4 = warning_color;
            }
            else
            {
                sprintf(label1, "%s", "Active");
                status_color4 = good_color;
            }

            ImGui::Columns(3, "dvlTitle", false);
            ImGui::SetColumnWidth(0,50);
            ImGui::SetColumnWidth(1, dvlWidth - 80);
            ImGui::Text("DVL"); ImGui::NextColumn();
            ImGui::SetCursorPosX(ImGui::GetCursorPosX() + ImGui::GetColumnWidth() - ImGui::CalcTextSize(label1).x - ImGui::GetScrollX() - 2 * ImGui::GetStyle().ItemSpacing.x);
            ImGui::Text("%s", label1); ImGui::NextColumn();
            ImGui::PushID(26);
            ImGui::PushStyleColor(ImGuiCol_Button, status_color4);
            ImGui::PushStyleColor(ImGuiCol_ButtonHovered, status_color4);
            ImGui::PushStyleColor(ImGuiCol_ButtonActive, status_color4);
            ImGui::Button("", ImVec2(15,15));
            ImGui::PopStyleColor(3);
            ImGui::PopID();
            ImGui::Columns(1);
            
            ImGui::Separator();

            {
                static int lastClicked = 0;
                ImVec4 status_color1 = ImGui::GetStyleColorVec4(ImGuiCol_Button), status_color2 = ImGui::GetStyleColorVec4(ImGuiCol_Button);
                if (lastClicked == 1)
                {
                    if (dvlEnableResponse == 1)
                    {
                        status_color1 = good_color;
                    }
                    else if (dvlEnableResponse == 2)
                    {
                        status_color1 = warning_color;
                    }
                    else
                    {
                        status_color1 = emergency_color;
                    }
                }
                else if (lastClicked == 2)
                {
                    if (dvlEnableResponse == 1)
                    {
                        status_color2 = good_color;
                    }
                    else if (dvlEnableResponse == 2)
                    {
                        status_color2 = warning_color;
                    }
                    else
                    {
                        status_color2 = emergency_color;
                    }
                }
                ImGui::PushID(27);
                ImGui::PushStyleColor(ImGuiCol_Button, status_color1);
                ImGui::PushStyleColor(ImGuiCol_ButtonHovered, status_color1);
                if(ImGui::Button("START", ImVec2(70, 20)))
                {
                    std_srvs::SetBool::Request req;
                    const bool request = true;
                    req.data = request;
                    dvlEnableService.call<std_srvs::SetBool>(req, std::bind(&SamMonitorWidget::dvlEnableCallback, this, std::placeholders::_1, std::placeholders::_2));
                    lastClicked = 1;
                }
                ImGui::PopStyleColor(2);
                ImGui::PopID();
                ImGui::SameLine();

                ImGui::PushID(28);
                ImGui::PushStyleColor(ImGuiCol_Button, status_color2);
                ImGui::PushStyleColor(ImGuiCol_ButtonHovered, status_color2);
                if(ImGui::Button("STOP", ImVec2(70, 20)))
                {
                    std_srvs::SetBool::Request req;
                    const bool request = false;
                    req.data = request;
                    dvlEnableService.call<std_srvs::SetBool>(req, std::bind(&SamMonitorWidget::dvlEnableCallback, this, std::placeholders::_1, std::placeholders::_2));
                    lastClicked = 2;
                }
                ImGui::PopStyleColor(2);
                ImGui::PopID();
            }

            ImGui::Text("Velocity [m/s]");
            if (dvlMsgOld)
            {
                ImGui::Text("x ---"); //ImGui::SameLine(60);
                ImGui::Text("y ---"); //ImGui::SameLine(50);
                ImGui::Text("z ---");
                ImGui::Separator();
                ImGui::Text("Velocity covariance");
                ImGui::Text("11: ---"); //ImGui::SameLine(50);
                ImGui::Text("22: ---"); //ImGui::SameLine(50);
                ImGui::Text("33: ---");
                ImGui::Separator();
                ImGui::Text("Altitude: ---"); ImGui::SameLine(60);
            }
            else
            {
                ImGui::Text("x %.3f", dvl->get_msg().velocity.x); //ImGui::SameLine(60);
                ImGui::Text("y %.3f", dvl->get_msg().velocity.y); //ImGui::SameLine(50);
                ImGui::Text("z %.3f", dvl->get_msg().velocity.z);
                ImGui::Separator();
                ImGui::Text("Velocity covariance");
                ImGui::Text("11: %.3f", dvl->get_msg().velocity_covariance[0]); //ImGui::SameLine(50);
                ImGui::Text("22: %.3f", dvl->get_msg().velocity_covariance[4]); //ImGui::SameLine(50);
                ImGui::Text("33: %.3f", dvl->get_msg().velocity_covariance[8]);
                ImGui::Separator();
                ImGui::Text("Altitude: %.1f m", dvl->get_msg().altitude); ImGui::SameLine(60);
            }

            ImGui::EndChild();
        }
        ImGui::SameLine();
        {
            const int vbsWidth = 140;
            ImGui::BeginChild("VBS", ImVec2(vbsWidth, subWindowHeight), true, 0);

            const uint32_t vbsMsgAgeThresh = 8;
            const bool vbsMsgOld = vbsMsgAgeThresh < current_time_epoch-vbs_fb->get_msg().header.stamp.sec ? true : false;
            std::string status_text;
            ImVec4 status_color4;
            char label1[10];
            if (vbsMsgOld)
            {
                sprintf(label1, "%s", "Inactive");
                status_color4 = warning_color;
            }
            else
            {
                sprintf(label1, "%s", "Active");
                status_color4 = good_color;
            }

            ImGui::Columns(3, "vbsTitle", false);
            ImGui::SetColumnWidth(0,50);
            ImGui::SetColumnWidth(1, vbsWidth - 80);
            ImGui::Text("VBS"); ImGui::NextColumn();
            ImGui::SetCursorPosX(ImGui::GetCursorPosX() + ImGui::GetColumnWidth() - ImGui::CalcTextSize(label1).x - ImGui::GetScrollX() - 2 * ImGui::GetStyle().ItemSpacing.x);
            ImGui::Text("%s", label1); ImGui::NextColumn();
            ImGui::PushID(26);
            ImGui::PushStyleColor(ImGuiCol_Button, status_color4);
            ImGui::PushStyleColor(ImGuiCol_ButtonHovered, status_color4);
            ImGui::PushStyleColor(ImGuiCol_ButtonActive, status_color4);
            ImGui::Button("", ImVec2(15,15));
            ImGui::PopStyleColor(3);
            ImGui::PopID();
            ImGui::Columns(1);
            
            ImGui::Separator();
            ImGui::Text("Position: %.1f%%", vbs_fb->get_msg().value);
            ImGui::Text("Setpoint: %.1f%%", vbs_cmd->get_msg().value);
            ImGui::Separator();
            ImGui::Text("Pressure");
            ImGui::Text("%.2f bar", vbs_pressure->get_msg().fluid_pressure/100000.0f);
            ImGui::Separator();
            ImGui::Text("Variance");
            ImGui::Text("%.2f bar", vbs_pressure->get_msg().variance/100000.0f);
            ImGui::Separator();
            ImGui::Text("Temperature");
            ImGui::Text("%.1f °C", vbs_temp->get_msg().temperature-273.15);

            ImGui::EndChild();
        }
        ImGui::SameLine();
        {
            const int sbgWidth = 140;
            ImGui::BeginChild("SBG", ImVec2(sbgWidth, subWindowHeight), true, 0);

            const uint32_t sbgMsgAgeThresh = 8;
            const bool sbgMsgOld = sbgMsgAgeThresh < current_time_epoch-sbg_euler->get_msg().header.stamp.sec ? true : false;
            std::string status_text;
            ImVec4 status_color4;
            char label1[10];
            if (sbgMsgOld)
            {
                sprintf(label1, "%s", "Inactive");
                status_color4 = warning_color;
            }
            else
            {
                sprintf(label1, "%s", "Active");
                status_color4 = good_color;
            }

            ImGui::Columns(3, "sbgTitle", false);
            ImGui::SetColumnWidth(0,50);
            ImGui::SetColumnWidth(1, sbgWidth - 80);
            ImGui::Text("SBG"); ImGui::NextColumn();
            ImGui::SetCursorPosX(ImGui::GetCursorPosX() + ImGui::GetColumnWidth() - ImGui::CalcTextSize(label1).x - ImGui::GetScrollX() - 2 * ImGui::GetStyle().ItemSpacing.x);
            ImGui::Text("%s", label1); ImGui::NextColumn();
            ImGui::PushID(26);
            ImGui::PushStyleColor(ImGuiCol_Button, status_color4);
            ImGui::PushStyleColor(ImGuiCol_ButtonHovered, status_color4);
            ImGui::PushStyleColor(ImGuiCol_ButtonActive, status_color4);
            ImGui::Button("", ImVec2(15,15));
            ImGui::PopStyleColor(3);
            ImGui::PopID();
            ImGui::Columns(1);
            ImGui::Separator();
            ImGui::Text("Heading: %.1f°", sbg_euler->get_msg().angle.z * (180.0/3.14));
            ImGui::Text("Roll:    %.1f°", sbg_euler->get_msg().angle.x * (180.0/3.14));
            ImGui::Text("Pitch:   %.1f°", sbg_euler->get_msg().angle.y * (180.0/3.14));

            ImGui::EndChild();
        }
        // >>>>>>>>>>>>>>>>>>>>>>>>>>>>>> Next row <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
        {
            const int sssWidth = 200;
            ImGui::BeginChild("SSS", ImVec2(sssWidth, subWindowHeight), true, 0);

            const uint32_t sssMsgAgeThresh = 8;
            const bool sssStatusOld = sssMsgAgeThresh < current_time_epoch-lastUpdateSSS ? true : false;
            const bool sssMsgOld = sssMsgAgeThresh < current_time_epoch-sss->get_msg().header.stamp.sec ? true : false;
            std::string status_text;
            ImVec4 status_color4;
            char label1[20];
            if (!sssStatusOld)
            {
                const uint8_t status = sss_status.sensor_status;
                if (status == 0)
                {
                    sprintf(label1, "%s", "OFF");
                    status_color4 = warning_color;
                }
                else if (status == 1)
                {
                    sprintf(label1, "%s", "Active");
                    status_color4 = good_color;
                }
                else if (status == 2)
                {
                    sprintf(label1, "%s", "Error");
                    status_color4 = emergency_color;
                }
            }
            else if (!sssMsgOld)
            {
                sprintf(label1, "%s", "Publishing");
                status_color4 = warning_color;
            }
            else
            {
                sprintf(label1, "%s", "Unknown");
                status_color4 = unknown_color;
            }

            ImGui::Columns(3, "sssTitle", false);
            ImGui::SetColumnWidth(0,50);
            ImGui::SetColumnWidth(1, sssWidth - 80);
            ImGui::Text("SSS"); ImGui::NextColumn();
            ImGui::SetCursorPosX(ImGui::GetCursorPosX() + ImGui::GetColumnWidth() - ImGui::CalcTextSize(label1).x - ImGui::GetScrollX() - 2 * ImGui::GetStyle().ItemSpacing.x);
            ImGui::Text("%s", label1); ImGui::NextColumn();
            ImGui::PushID(26);
            ImGui::PushStyleColor(ImGuiCol_Button, status_color4);
            ImGui::PushStyleColor(ImGuiCol_ButtonHovered, status_color4);
            ImGui::PushStyleColor(ImGuiCol_ButtonActive, status_color4);
            ImGui::Button("", ImVec2(15,15));
            ImGui::PopStyleColor(3);
            ImGui::PopID();
            ImGui::Columns(1);
            
            ImGui::Separator();

            // ImGui::Text("Velocity [m/s]");
            // if (sssMsgOld)
            // {
            //     ImGui::Text("x ---"); //ImGui::SameLine(60);
            //     ImGui::Text("y ---"); //ImGui::SameLine(50);
            //     ImGui::Text("z ---");
            //     ImGui::Separator();
            //     ImGui::Text("Velocity covariance");
            //     ImGui::Text("11: ---"); //ImGui::SameLine(50);
            //     ImGui::Text("22: ---"); //ImGui::SameLine(50);
            //     ImGui::Text("33: ---");
            //     ImGui::Separator();
            //     ImGui::Text("Altitude: ---"); ImGui::SameLine(60);
            // }
            // else
            // {
            //     ImGui::Text("x %.3f", sss->get_msg().velocity.x); //ImGui::SameLine(60);
            //     ImGui::Text("y %.3f", sss->get_msg().velocity.y); //ImGui::SameLine(50);
            //     ImGui::Text("z %.3f", sss->get_msg().velocity.z);
            //     ImGui::Separator();
            //     ImGui::Text("Velocity covariance");
            //     ImGui::Text("11: %.3f", sss->get_msg().velocity_covariance[0]); //ImGui::SameLine(50);
            //     ImGui::Text("22: %.3f", sss->get_msg().velocity_covariance[4]); //ImGui::SameLine(50);
            //     ImGui::Text("33: %.3f", sss->get_msg().velocity_covariance[8]);
            //     ImGui::Separator();
            //     ImGui::Text("Altitude: %.1f m", sss->get_msg().altitude); ImGui::SameLine(60);
            // }

            // std_srvs::SetBool::Request req;

            static int lastClicked = 0;
            ImVec4 status_color1 = ImGui::GetStyleColorVec4(ImGuiCol_Button), status_color2 = ImGui::GetStyleColorVec4(ImGuiCol_Button);
            if (lastClicked == 1)
            {
                if (sssEnableResponse == 1)
                {
                    status_color1 = good_color;
                }
                else if (sssEnableResponse == 2)
                {
                    status_color1 = warning_color;
                }
                else
                {
                    status_color1 = emergency_color;
                }
            }
            else if (lastClicked == 2)
            {
                if (sssEnableResponse == 1)
                {
                    status_color2 = good_color;
                }
                else if (sssEnableResponse == 2)
                {
                    status_color2 = warning_color;
                }
                else
                {
                    status_color2 = emergency_color;
                }
            }
            ImGui::PushID(27);
            ImGui::PushStyleColor(ImGuiCol_Button, status_color1);
            ImGui::PushStyleColor(ImGuiCol_ButtonHovered, status_color1);
            if(ImGui::Button("START", ImVec2(80, 20)))
            {
                std_srvs::SetBool::Request req;
                const bool request = true;
                req.data = request;
                sssEnableService.call<std_srvs::SetBool>(req, std::bind(&SamMonitorWidget::sssEnableCallback, this, std::placeholders::_1, std::placeholders::_2));
                lastClicked = 1;
            }
            ImGui::PopStyleColor(2);
            ImGui::PopID();
            ImGui::SameLine();

            ImGui::PushID(28);
            ImGui::PushStyleColor(ImGuiCol_Button, status_color2);
            ImGui::PushStyleColor(ImGuiCol_ButtonHovered, status_color2);
            if(ImGui::Button("STOP", ImVec2(80, 20)))
            {
                std_srvs::SetBool::Request req;
                const bool request = false;
                req.data = request;
                sssEnableService.call<std_srvs::SetBool>(req, std::bind(&SamMonitorWidget::sssEnableCallback, this, std::placeholders::_1, std::placeholders::_2));
                lastClicked = 2;
            }
            ImGui::PopStyleColor(2);
            ImGui::PopID();

            ImGui::EndChild();
        }
        
    }
    else if(selectedTab == 3)
    {
        const int jHeigth = 220;
        {
            const ImVec2 jSize = ImVec2(600, jHeigth);
            ImGui::BeginChild("Jetson", jSize, true, 0);

            const std::vector<const char*> powerModes{"No limit", "10W", "15W", "30W 8 Cores", "30W 6 Cores", "30W 4 Cores", "30W 2 Cores", "15W Desktop"};
            ImGui::Text("Performance");
            ImGui::Separator();
            ImGui::Columns(2);
            ImGui::SetColumnWidth(0, jSize[0]/2);
            ImGui::Text("Power mode: %s", powerModes[nvMode]);
            ImGui::NextColumn();
            if (jetsonClocks)
            {
                ImGui::Text("Jetson clocks: active");
            }
            else
            {
                ImGui::Text("Jetson clocks: inactive");
            }
            // ImGui::Text("Power mode: %d", nvMode);
            ImGui::Columns(1);
            ImGui::Text("Uptime: %s", uptime.c_str());
            ImGui::Text(" ");
            ImGui::Separator();
            // ImGui::SetColumnWidth(1, 230);
            // ImGui::SetColumnWidth(2, 40);
            // ImGui::SetColumnWidth(3, 230);

            // ImGui::Text("Core"); ImGui::NextColumn();
            // ImGui::Text("Utilization"); ImGui::NextColumn();
            // ImGui::Text("Core"); ImGui::NextColumn();
            // ImGui::Text("Utilization"); ImGui::NextColumn();
            ImGui::Columns(6);
            ImGui::SetColumnWidth(0, 40);
            ImGui::SetColumnWidth(1, 150);
            ImGui::SetColumnWidth(2, 110);
            ImGui::SetColumnWidth(3, 40);
            ImGui::SetColumnWidth(4, 150);
            ImGui::SetColumnWidth(5, 110);
            char buf[2];
            sprintf(buf, "");
            for(int i = 0; i < 4; i++)
            {
                ImGui::Text("CPU%d", i+1); ImGui::NextColumn();
                ImGui::ProgressBar(cpu[i].first/100.0, ImVec2(-1, 15), buf); ImGui::NextColumn();
                // ImGui::Text("%d%% @ %.1fGHz", cpu[i].first, cpu[i].second); ImGui::NextColumn();
                ImGui::Text("%d%% ", cpu[i].first); ImGui::SameLine(40);
                ImGui::Text("@ %.1fGHz", cpu[i].second); ImGui::NextColumn();
                ImGui::Text("CPU%d", i+5); ImGui::NextColumn();
                ImGui::ProgressBar(cpu[i+4].first/100.0, ImVec2(-1, 15), buf); ImGui::NextColumn();
                ImGui::Text("%d%% ", cpu[i+4].first); ImGui::SameLine(40);
                ImGui::Text("@ %.1fGHz", cpu[i+4].second); ImGui::NextColumn();
            }

            ImGui::Columns(1);
            ImGui::Text("GPU"); ImGui::SameLine();
            ImGui::ProgressBar(gpu.first/100.0, ImVec2(400, 15), buf); ImGui::SameLine();
            ImGui::Text("%d%% @ %.0fMHz", gpu.first, gpu.second);
            ImGui::Text("RAM"); ImGui::SameLine();
            ImGui::ProgressBar(ram.first/ram.second, ImVec2(400, 15), buf); ImGui::SameLine();
            ImGui::Text("%.0f%% - %.1fGB/%.1fGB", 100*ram.first/ram.second, ram.first, ram.second);
            ImGui::EndChild();
        }
        ImGui::SameLine();
        {
            ImGui::BeginChild("Jetson2", ImVec2(320, 200), true, 0);
            ImGui::Text("Temperatures");
            ImGui::Separator();

            char buf[2];
            sprintf(buf, "");
            const std::vector<std::pair<const char*, float>> thermalZones{{"Tdiode", 109}, {"AO", 109}, {"iwlwifi", 120}, {"FAN", 70}, {"AUX", 82}, {"GPU", 88}, {"Tboard", 107}, {"CPU", 86}};
            ImGui::Columns(4, "jetsonTemps", false);
            ImGui::SetColumnWidth(0, 60);
            ImGui::SetColumnWidth(1, 150);
            ImGui::SetColumnWidth(2, 70);
            // ImGui::SetColumnWidth(3, 20);
            for(int i = 0; i < thermalZones.size(); i++)
            {
                ImGui::Text("%s", thermalZones[i].first); ImGui::NextColumn();
                ImGui::ProgressBar(jTemp[i]/thermalZones[i].second, ImVec2(-1, 15), buf); ImGui::NextColumn();
                ImGui::Text("%.1f/%.0f", jTemp[i], thermalZones[i].second); ImGui::NextColumn();
                ImGui::Text("°C"); ImGui::NextColumn();
            }

            ImGui::EndChild();
        }
    }
    else if(selectedTab == 4)
    {
        static int selected = 0;
        ImGui::Text("Jetson"); ImGui::SameLine(100);
        ImGui::InputInt("message", &selected);
        {
            ImGui::BeginChild("Jetsons", ImGui::GetContentRegionAvail(), true, 0);
            ImGui::Text("name:        %s", system->get_msg().status[selected].name.c_str());
            ImGui::Text("message:     %s", system->get_msg().status[selected].message.c_str());
            ImGui::Text("hardware_id: %s", system->get_msg().status[selected].hardware_id.c_str());
            ImGui::Text("values");
            for(int i = 0; i<system->get_msg().status[selected].values.size(); i++)
            {
                ImGui::Text("key: %s", system->get_msg().status[selected].values[i].key.c_str()); ImGui::SameLine();
                ImGui::Text("value: %s", system->get_msg().status[selected].values[i].value.c_str());
            }
            // ImGui::Text("CPU: %d", system->get_msg().status[1]);
            ImGui::EndChild();
        }
    }

    ImGui::End();
}
void SamMonitorWidget::callbackCharge(const sam_msgs::ConsumedChargeArray& msg)
{
    for (int i = 0; i < msg.array.size(); i++)
    {
        circuitCharges[msg.array[i].circuit_id] = msg.array[i].charge;
    }
}
void SamMonitorWidget::service_callback(const uavcan_ros_msgs::UavcanRestartNode::Response& res, bool result)
{
    //topics = res.topics;
    // topics.clear();
    // std::copy_if(res.topics.begin(), res.topics.end(), std::back_inserter(topics), [](const std::string& s){return s.find("compressedDepth") == std::string::npos;});
}
void SamMonitorWidget::batteryCallback(const sam_msgs::UavcanUpdateBattery::Response& res, bool result)
{
    if (result) // Service call successfull
    {
        if (res.success)    // Update successfull
        {
            batteryUpdateResponse = 1;
        }
        else    // Invalid request
        {
            batteryUpdateResponse = 2;
        }
    }
    else    // Service call failed
    {
        batteryUpdateResponse = -2;
    }
}
void SamMonitorWidget::callbackSystem(const diagnostic_msgs::DiagnosticArray& msg)
{
    lastSystemUpdate = msg.header.stamp.sec;
    for(int i = 0; i < msg.status.size(); i++)
    {
        if(strcmp(msg.status[i].name.c_str(), "jetson_stats board status") == 0)
        {
            for(int j = 0; j < msg.status[i].values.size(); j++)
            {
                if(strcmp(msg.status[i].values[j].key.c_str(), "NV Power") == 0)
                {
                    nvMode = std::stoi(msg.status[i].values[j].value);
                }
                else if(strcmp(msg.status[i].values[j].key.c_str(), "jetson_clocks") == 0)
                {
                    if (strcmp(msg.status[i].values[j].value.c_str(), "active") == 0)
                    {
                        jetsonClocks = true;
                    }
                    else
                    {
                        jetsonClocks = false;
                    }
                }
                else if(strcmp(msg.status[i].values[j].key.c_str(), "Up Time") == 0)
                {
                    uptime = msg.status[i].values[j].value.c_str();
                }
            }
        }
        else if(strcmp(msg.status[i].name.c_str(), "jetson_stats cpu CPU1") == 0)
        {
            int freq = 0;
            int div = 1;
            for(int j = 0; j < msg.status[i].values.size(); j++)
            {
                if(strcmp(msg.status[i].values[j].key.c_str(), "Val") == 0)
                {
                    cpu[0].first = std::stoi(msg.status[i].values[j].value);
                }
                else if(strcmp(msg.status[i].values[j].key.c_str(), "Freq") == 0)
                {
                    freq = std::stoi(msg.status[i].values[j].value);
                }
                else if(strcmp(msg.status[i].values[j].key.c_str(), "Unit") == 0)
                {
                    if(strcmp(msg.status[i].values[j].value.c_str(), "khz") == 0)
                    {
                        div = 1000000;
                    }
                    else if(strcmp(msg.status[i].values[j].value.c_str(), "mhz") == 0)
                    {
                        div = 1000;
                    }
                }
            }
            cpu[0].second = (float)freq/div;
        }
        else if(strcmp(msg.status[i].name.c_str(), "jetson_stats cpu CPU2") == 0)
        {
            int freq = 0;
            int div = 1;
            for(int j = 0; j < msg.status[i].values.size(); j++)
            {
                if(strcmp(msg.status[i].values[j].key.c_str(), "Val") == 0)
                {
                    cpu[1].first = std::stoi(msg.status[i].values[j].value);
                }
                else if(strcmp(msg.status[i].values[j].key.c_str(), "Freq") == 0)
                {
                    freq = std::stoi(msg.status[i].values[j].value);
                }
                else if(strcmp(msg.status[i].values[j].key.c_str(), "Unit") == 0)
                {
                    if(strcmp(msg.status[i].values[j].value.c_str(), "khz") == 0)
                    {
                        div = 1000000;
                    }
                    else if(strcmp(msg.status[i].values[j].value.c_str(), "mhz") == 0)
                    {
                        div = 1000;
                    }
                }
            }
            cpu[1].second = (float)freq/div;
        }
        else if(strcmp(msg.status[i].name.c_str(), "jetson_stats cpu CPU3") == 0)
        {
            int freq = 0;
            int div = 1;
            for(int j = 0; j < msg.status[i].values.size(); j++)
            {
                if(strcmp(msg.status[i].values[j].key.c_str(), "Val") == 0)
                {
                    cpu[2].first = std::stoi(msg.status[i].values[j].value);
                }
                else if(strcmp(msg.status[i].values[j].key.c_str(), "Freq") == 0)
                {
                    freq = std::stoi(msg.status[i].values[j].value);
                }
                else if(strcmp(msg.status[i].values[j].key.c_str(), "Unit") == 0)
                {
                    if(strcmp(msg.status[i].values[j].value.c_str(), "khz") == 0)
                    {
                        div = 1000000;
                    }
                    else if(strcmp(msg.status[i].values[j].value.c_str(), "mhz") == 0)
                    {
                        div = 1000;
                    }
                }
            }
            cpu[2].second = (float)freq/div;
        }
        else if(strcmp(msg.status[i].name.c_str(), "jetson_stats cpu CPU4") == 0)
        {
            int freq = 0;
            int div = 1;
            for(int j = 0; j < msg.status[i].values.size(); j++)
            {
                if(strcmp(msg.status[i].values[j].key.c_str(), "Val") == 0)
                {
                    cpu[3].first = std::stoi(msg.status[i].values[j].value);
                }
                else if(strcmp(msg.status[i].values[j].key.c_str(), "Freq") == 0)
                {
                    freq = std::stoi(msg.status[i].values[j].value);
                }
                else if(strcmp(msg.status[i].values[j].key.c_str(), "Unit") == 0)
                {
                    if(strcmp(msg.status[i].values[j].value.c_str(), "khz") == 0)
                    {
                        div = 1000000;
                    }
                    else if(strcmp(msg.status[i].values[j].value.c_str(), "mhz") == 0)
                    {
                        div = 1000;
                    }
                }
            }
            cpu[3].second = (float)freq/div;
        }
        else if(strcmp(msg.status[i].name.c_str(), "jetson_stats cpu CPU5") == 0)
        {
            int freq = 0;
            int div = 1;
            for(int j = 0; j < msg.status[i].values.size(); j++)
            {
                if(strcmp(msg.status[i].values[j].key.c_str(), "Val") == 0)
                {
                    cpu[4].first = std::stoi(msg.status[i].values[j].value);
                }
                else if(strcmp(msg.status[i].values[j].key.c_str(), "Freq") == 0)
                {
                    freq = std::stoi(msg.status[i].values[j].value);
                }
                else if(strcmp(msg.status[i].values[j].key.c_str(), "Unit") == 0)
                {
                    if(strcmp(msg.status[i].values[j].value.c_str(), "khz") == 0)
                    {
                        div = 1000000;
                    }
                    else if(strcmp(msg.status[i].values[j].value.c_str(), "mhz") == 0)
                    {
                        div = 1000;
                    }
                }
            }
            cpu[4].second = (float)freq/div;
        }
        else if(strcmp(msg.status[i].name.c_str(), "jetson_stats cpu CPU6") == 0)
        {
            int freq = 0;
            int div = 1;
            for(int j = 0; j < msg.status[i].values.size(); j++)
            {
                if(strcmp(msg.status[i].values[j].key.c_str(), "Val") == 0)
                {
                    cpu[5].first = std::stoi(msg.status[i].values[j].value);
                }
                else if(strcmp(msg.status[i].values[j].key.c_str(), "Freq") == 0)
                {
                    freq = std::stoi(msg.status[i].values[j].value);
                }
                else if(strcmp(msg.status[i].values[j].key.c_str(), "Unit") == 0)
                {
                    if(strcmp(msg.status[i].values[j].value.c_str(), "khz") == 0)
                    {
                        div = 1000000;
                    }
                    else if(strcmp(msg.status[i].values[j].value.c_str(), "mhz") == 0)
                    {
                        div = 1000;
                    }
                }
            }
            cpu[5].second = (float)freq/div;
        }
        else if(strcmp(msg.status[i].name.c_str(), "jetson_stats cpu CPU7") == 0)
        {
            int freq = 0;
            int div = 1;
            for(int j = 0; j < msg.status[i].values.size(); j++)
            {
                if(strcmp(msg.status[i].values[j].key.c_str(), "Val") == 0)
                {
                    cpu[6].first = std::stoi(msg.status[i].values[j].value);
                }
                else if(strcmp(msg.status[i].values[j].key.c_str(), "Freq") == 0)
                {
                    freq = std::stoi(msg.status[i].values[j].value);
                }
                else if(strcmp(msg.status[i].values[j].key.c_str(), "Unit") == 0)
                {
                    if(strcmp(msg.status[i].values[j].value.c_str(), "khz") == 0)
                    {
                        div = 1000000;
                    }
                    else if(strcmp(msg.status[i].values[j].value.c_str(), "mhz") == 0)
                    {
                        div = 1000;
                    }
                }
            }
            cpu[6].second = (float)freq/div;
        }
        else if(strcmp(msg.status[i].name.c_str(), "jetson_stats cpu CPU8") == 0)
        {
            int freq = 0;
            int div = 1;
            for(int j = 0; j < msg.status[i].values.size(); j++)
            {
                if(strcmp(msg.status[i].values[j].key.c_str(), "Val") == 0)
                {
                    cpu[7].first = std::stoi(msg.status[i].values[j].value);
                }
                else if(strcmp(msg.status[i].values[j].key.c_str(), "Freq") == 0)
                {
                    freq = std::stoi(msg.status[i].values[j].value);
                }
                else if(strcmp(msg.status[i].values[j].key.c_str(), "Unit") == 0)
                {
                    if(strcmp(msg.status[i].values[j].value.c_str(), "khz") == 0)
                    {
                        div = 1000000;
                    }
                    else if(strcmp(msg.status[i].values[j].value.c_str(), "mhz") == 0)
                    {
                        div = 1000;
                    }
                }
            }
            cpu[7].second = (float)freq/div;
        }
        else if(strcmp(msg.status[i].name.c_str(), "jetson_stats gpu") == 0)
        {
            int freq = 0;
            int div = 1;
            for(int j = 0; j < msg.status[i].values.size(); j++)
            {
                if(strcmp(msg.status[i].values[j].key.c_str(), "Val") == 0)
                {
                    gpu.first = std::stoi(msg.status[i].values[j].value);
                }
                else if(strcmp(msg.status[i].values[j].key.c_str(), "Freq") == 0)
                {
                    freq = std::stoi(msg.status[i].values[j].value);
                }
                else if(strcmp(msg.status[i].values[j].key.c_str(), "Unit") == 0)
                {
                    if(strcmp(msg.status[i].values[j].value.c_str(), "khz") == 0)
                    {
                        div = 1000;
                    }
                    else if(strcmp(msg.status[i].values[j].value.c_str(), "mhz") == 0)
                    {
                        div = 1;
                    }
                }
            }
            gpu.second = (float)freq/div;
        }
        else if(strcmp(msg.status[i].name.c_str(), "jetson_stats mem ram") == 0)
        {
            int use = 0;
            int tot = 1;
            int div = 1;
            for(int j = 0; j < msg.status[i].values.size(); j++)
            {
                if(strcmp(msg.status[i].values[j].key.c_str(), "Use") == 0)
                {
                    use = std::stoi(msg.status[i].values[j].value);
                }
                else if(strcmp(msg.status[i].values[j].key.c_str(), "Total") == 0)
                {
                    tot = std::stoi(msg.status[i].values[j].value);
                }
                else if(strcmp(msg.status[i].values[j].key.c_str(), "Unit") == 0)
                {
                    if(strcmp(msg.status[i].values[j].value.c_str(), "kB") == 0)
                    {
                        div = 1000000;
                    }
                    else if(strcmp(msg.status[i].values[j].value.c_str(), "MB") == 0)
                    {
                        div = 1000;
                    }
                }
            }
            ram.first = (float)use/div;
            ram.second = (float)tot/div;
        }
        else if(strcmp(msg.status[i].name.c_str(), "jetson_stats temp") == 0)
        {
            for(int j = 0; j < msg.status[i].values.size(); j++)
            {
                if(strcmp(msg.status[i].values[j].key.c_str(), "Tdiode") == 0)
                {
                    jTemp[0] = std::stof(msg.status[i].values[j].value);
                }
                else if(strcmp(msg.status[i].values[j].key.c_str(), "AO") == 0)
                {
                    jTemp[1] = std::stof(msg.status[i].values[j].value);
                }
                else if(strcmp(msg.status[i].values[j].key.c_str(), "iwlwifi") == 0)
                {
                    jTemp[2] = std::stof(msg.status[i].values[j].value);
                }
                else if(strcmp(msg.status[i].values[j].key.c_str(), "thermal") == 0)
                {
                    jTemp[3] = std::stof(msg.status[i].values[j].value);
                }
                else if(strcmp(msg.status[i].values[j].key.c_str(), "AUX") == 0)
                {
                    jTemp[4] = std::stof(msg.status[i].values[j].value);
                }
                else if(strcmp(msg.status[i].values[j].key.c_str(), "GPU") == 0)
                {
                    jTemp[5] = std::stof(msg.status[i].values[j].value);
                }
                else if(strcmp(msg.status[i].values[j].key.c_str(), "Tboard") == 0)
                {
                    jTemp[6] = std::stof(msg.status[i].values[j].value);
                }
                else if(strcmp(msg.status[i].values[j].key.c_str(), "CPU") == 0)
                {
                    jTemp[7] = std::stof(msg.status[i].values[j].value);
                }
            }

        }
    }
    // const int mainListLimit = 100;
    // const int errorListLimit = 50;
    // const int warningListLimit = 50;
    // const int btListLimit = 50;
    
    // const uint8_t msgLevel = msg.level;
    // const bool aboutRosBridge = strcmp(msg.name.c_str(), "/sam/rosbridge_websocket") == 0 ? true : false;

    // if(!aboutRosBridge || msgLevel >= 4)
    // {
    //     mainLogList.push_front(msg);
    //     const int mainListOversize = mainLogList.size() - mainListLimit;
    //     for(int i = 0; i < mainListOversize; i++)
    //     {
    //         mainLogList.pop_back();
    //     }
    // }

    // const bool errorLog = msgLevel >= 8 ? true : false;
    // if(errorLog)
    // {
    //     errorLogList.push_front(msg);
    //     const int errorListOversize = errorLogList.size() - errorListLimit;
    //     for(int i = 0; i < errorListOversize; i++)
    //     {
    //         errorLogList.pop_back();
    //     }
    // }

    // const bool warningLog = msgLevel == 4 ? true : false;
    // if(warningLog)
    // {
    //     warningLogList.push_front(msg);
    //     const int warningListOversize = warningLogList.size() - warningListLimit;
    //     for(int i = 0; i < warningListOversize; i++)
    //     {
    //         warningLogList.pop_back();
    //     }
    // }

    // const bool aboutBT = strcmp(msg.name.c_str(), "/sam/sam_bt") == 0 ? true : false;
    // if(aboutBT)
    // {
    //     btLogList.push_front(msg);
    //     const int btListOversize = btLogList.size() - btListLimit;
    //     for(int i = 0; i < btListOversize; i++)
    //     {
    //         btLogList.pop_back();
    //     }
    // }
}
void SamMonitorWidget::sss_status_callback(const smarc_msgs::SensorStatus& msg)
{
    const std::chrono::system_clock::time_point tp = std::chrono::system_clock::now();
    const std::chrono::system_clock::duration dtn = tp.time_since_epoch();
    lastUpdateSSS = dtn.count() * std::chrono::system_clock::period::num / std::chrono::system_clock::period::den;
    sss_status = msg;
}
void SamMonitorWidget::sssEnableCallback(const std_srvs::SetBool::Response& res, bool result)
{
    if (result) // Service call successfull
    {
        if (res.success)    // Update successfull
        {
            sssEnableResponse = 1;
        }
        else    // Invalid request
        {
            sssEnableResponse = 2;
        }
    }
    else    // Service call failed
    {
        sssEnableResponse = -2;
    }
}
void SamMonitorWidget::dvl_status_callback(const smarc_msgs::SensorStatus& msg)
{
    const std::chrono::system_clock::time_point tp = std::chrono::system_clock::now();
    const std::chrono::system_clock::duration dtn = tp.time_since_epoch();
    lastUpdateDVL = dtn.count() * std::chrono::system_clock::period::num / std::chrono::system_clock::period::den;
    dvl_status = msg;
}
void SamMonitorWidget::dvlEnableCallback(const std_srvs::SetBool::Response& res, bool result)
{
    if (result) // Service call successfull
    {
        if (res.success)    // Update successfull
        {
            dvlEnableResponse = 1;
        }
        else    // Invalid request
        {
            dvlEnableResponse = 2;
        }
    }
    else    // Service call failed
    {
        dvlEnableResponse = -2;
    }
}
// -------------------------------------- SamMonitorWidget end --------------------------------------

std::pair<ImVec4, const char*> healthToColoredString(const std::uint8_t health)
{
    static const std::unordered_map<std::uint8_t, std::pair<ImVec4, const char*>> map
    {
        { 0, { ImVec4(0.0f, 0.71f, 0.06f, 1.00f),   "OK" }},       // Green (dirty)
        { 1, { ImVec4(0.87f, 0.57f, 0.0f, 1.00f),   "WARNING" }},  // Orange
        { 2, { ImColor(235, 0, 211),                "ERROR" }},    // Magenta
        { 3, { ImVec4(1.0f, 0.0f, 0.0f, 1.00f),     "CRITICAL" }}  // Red
    };
    try
    {
        return map.at(health);
    }
    catch (std::out_of_range&)
    {
        return { ImVec4(1.0f, 0.0f, 0.0f, 1.00f), std::to_string(health).c_str() };
    }
}

std::pair<ImVec4, const char*> modeToColoredString(const std::uint8_t mode)
{
    static const std::unordered_map<std::uint8_t, std::pair<ImVec4, const char*>> map
    {
        { 0, { ImVec4(0.0f, 0.71f, 0.06f, 1.00f),   "OPERATIONAL" }},      // Green (dirty)
        { 1, { ImColor(0, 255, 255),                "INITIALIZATION" }},   // Cyan
        { 2, { ImVec4(0.87f, 0.57f, 0.0f, 1.00f),   "MAINTENANCE" }},      // Orange
        { 3, { ImColor(235, 0, 211),                "SOFTWARE_UPDATE" }},  // Magenta
        { 7, { ImVec4(1.0f, 0.0f, 0.0f, 1.00f),     "OFFLINE" }}           // Red
    };
    try
    {
        return map.at(mode);
    }
    catch (std::out_of_range&)
    {
        return { ImVec4(1.0f, 0.0f, 0.0f, 1.00f), std::to_string(mode).c_str() };
    }
}

std::pair<ImVec4, const char*> circuitModeToColoredString(const std::uint8_t mode)
{
    static const std::unordered_map<std::uint8_t, std::pair<ImVec4, const char*>> map
    {
        { 0, { ImVec4(0.0f, 0.71f, 0.06f, 1.00f),   "GOOD" }},     // Green (dirty)
        { 1, { ImVec4(1.0f, 0.0f, 0.0f, 1.00f), "OVERVOLTAGE" }},   // Red
        { 2, { ImVec4(1.0f, 0.0f, 0.0f, 1.00f), "UNDERVOLTAGE" }},  // Red
        { 4, { ImVec4(1.0f, 0.0f, 0.0f, 1.00f), "OVERCURRENT" }},   // Red
        { 8, { ImVec4(1.0f, 0.0f, 0.0f, 1.00f), "UNDERCURRENT" }}   // Red
    };
    try
    {
        return map.at(mode);
    }
    catch (std::out_of_range&)
    {
        return { ImVec4(1.0f, 0.0f, 0.0f, 1.00f), std::to_string(mode).c_str() };
    }
}

// ---------------------------------------- SamLogWidget ----------------------------------------
SamLogWidget::SamLogWidget(roswasm::NodeHandle& nh)
{
    // subLog = nh->subscribe<rosgraph_msgs::Log>("/rosout_agg", std::bind(&SamLogWidget::callbackLog, this, std::placeholders::_1), 10);

    // subLog = nh->subscribe<rosgraph_msgs::Log>("/rosout", std::bind(&SamLogWidget::callbackLog, this, std::placeholders::_1), 10);
    nhLocal = &nh;
    subLogEnabled = false;
    subLogInitialState = true;
    // subLog = nullptr;
}
void SamLogWidget::show_window(bool& show_roslog_window, bool guiDebug)
{
    // ImGuiIO& io = ImGui::GetIO();
    // ImVec4 col = ImGui::GetStyle().Colors[23];

    
    if(logEnable && !subLogEnabled)
    {
        subLog = nhLocal->subscribe<rosgraph_msgs::Log>("/rosout", 10, &SamLogWidget::callbackLog, this);
        subLogEnabled = true;
        // subLog = nhLocal.subscribe<rosgraph_msgs::Log>("/rosout", std::bind(&SamLogWidget::callbackLog, this, std::placeholders::_1), 10);
    }
    else if(!logEnable && subLogEnabled)
    {
        // delete subLog;
        subLog.shutdown();
        subLogEnabled = false;
        // subLog = nullptr;
    }
    ImGui::SetNextWindowSize(ImVec2(472, 400), ImGuiCond_FirstUseEver);
    ImGui::Begin("Log", &show_roslog_window);

    ImGui::Checkbox("Enable", &logEnable);
    
    if (logEnable && subLogInitialState)
    {
        ImGui::OpenPopup("logPrompt");
    }
    if (ImGui::BeginPopup("logPrompt"))
    {
        ImGui::BeginChild("Log warning", ImVec2(250, 70), false, 0);
        ImGui::TextWrapped("Subscribing to the logs demands a lot of resources from the system and should only be done if neccessary!");
        ImGui::Separator();
        

        ImGui::EndChild();
        if(ImGui::Button("I understand, show me the logs", ImVec2(250, 20)))
        {
            subLogInitialState = false;
            ImGui::CloseCurrentPopup();
        }
        if(ImGui::Button("Nevermind", ImVec2(250, 20)))
        {
            logEnable = false;
            ImGui::CloseCurrentPopup();
        }

        ImGui::EndPopup();
    }

    ImGui::SameLine();

    // std::chrono::system_clock::time_point tp = std::chrono::system_clock::now();
    // std::chrono::system_clock::duration dtn = tp.time_since_epoch();
    // uint32_t current_time_epoch = dtn.count() * std::chrono::system_clock::period::num / std::chrono::system_clock::period::den;
    if (!subLogInitialState)
    {
        static int selectedTab = 0;
        const std::vector<const char*> tabNames{"All", "Error", "Warning", "UAVCAN"};
        for (int i = 0; i < tabNames.size(); i++)
        {
            if (i > 0) ImGui::SameLine();
            ImGui::PushID(i);
            if(selectedTab == i) {
                ImGui::PushStyleColor(ImGuiCol_Button, ImGui::GetColorU32(ImGuiCol_FrameBgActive));
            } else {
                ImGui::PushStyleColor(ImGuiCol_Button, ImGui::GetColorU32(ImGuiCol_FrameBg));
            }
            ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImGui::GetColorU32(ImGuiCol_FrameBgHovered));
            ImGui::PushStyleColor(ImGuiCol_ButtonActive, ImGui::GetColorU32(ImGuiCol_FrameBgActive));
            if(ImGui::Button(tabNames[i], ImVec2(85,20))){
                selectedTab = i;
            }
            ImGui::PopStyleColor(3);
            ImGui::PopID();
        }
        if(guiDebug)
        {
            ImGui::Text("Choosen tab: %d", selectedTab);
            // ImGui::Text("%u", current_time_epoch);
        }

        if(selectedTab == 0)
        {
            {
                ImGui::BeginChild("mainLog", ImGui::GetContentRegionAvail(), true, 0);
                // ImGui::BeginChild("mainLog", ImVec2(600, 200), true, 0);

                ImGui::Text("Log");
                if(guiDebug)
                {
                    ImGui::SameLine(); ImGui::Text("List size: %lu", mainLogList.size());
                }
            
                ImGui::Separator();
                
                {
                    ImGui::Columns(1, "mainLogList");

                    // ImGui::SetColumnWidth(0, 100);
                    // ImGui::SetColumnWidth(1, 400);
                    // ImGui::Text("Message"); ImGui::NextColumn();
                    // ImGui::Text("Function"); ImGui::NextColumn();
                    // ImGui::Separator();

                    static std::list<rosgraph_msgs::Log>::iterator selectedLog;
                    for (auto msgIt = mainLogList.begin(); msgIt != mainLogList.end(); msgIt++)
                    {
                        char logLabel[32];
                        sprintf(logLabel, "%s", msgIt->msg.c_str());
                        if (ImGui::Selectable(logLabel, selectedLog == msgIt, ImGuiSelectableFlags_SpanAllColumns))
                        {
                            selectedLog = msgIt;
                        }
                        // ImGui::NextColumn();
                        // ImGui::Text("%s", msgIt->function.c_str()); ImGui::NextColumn();
                    }
                    ImGui::Columns(1);
                }
                ImGui::EndChild();
            }
        }
        else if(selectedTab == 1)
        {
            {
                ImGui::BeginChild("errorLog", ImGui::GetContentRegionAvail(), true, 0);

                ImGui::Text("Log");
                if(guiDebug)
                {
                    ImGui::SameLine(); ImGui::Text("List size: %lu", errorLogList.size());
                }
            
                ImGui::Separator();
                
                {
                    ImGui::Columns(1, "errorLogList");

                    // ImGui::SetColumnWidth(0, 100);
                    // ImGui::SetColumnWidth(1, 400);
                    // ImGui::Text("Message"); ImGui::NextColumn();
                    // ImGui::Text("Function"); ImGui::NextColumn();
                    // ImGui::Separator();

                    static std::list<rosgraph_msgs::Log>::iterator selectedLog;
                    for (auto msgIt = errorLogList.begin(); msgIt != errorLogList.end(); msgIt++)
                    {
                        char logLabel[32];
                        sprintf(logLabel, "%s", msgIt->msg.c_str());
                        if (ImGui::Selectable(logLabel, selectedLog == msgIt, ImGuiSelectableFlags_SpanAllColumns))
                        {
                            selectedLog = msgIt;
                        }
                        // ImGui::NextColumn();
                        // ImGui::Text("%s", msgIt->function.c_str()); ImGui::NextColumn();
                    }
                    ImGui::Columns(1);
                }
                ImGui::EndChild();
            }
        }
        else if(selectedTab == 2)
        {
            {
                ImGui::BeginChild("warningLog", ImGui::GetContentRegionAvail(), true, 0);

                ImGui::Text("Log");
                if(guiDebug)
                {
                    ImGui::SameLine(); ImGui::Text("List size: %lu", warningLogList.size());
                }
            
                ImGui::Separator();
                
                {
                    ImGui::Columns(1, "warningLogList");

                    // ImGui::SetColumnWidth(0, 100);
                    // ImGui::SetColumnWidth(1, 400);
                    // ImGui::Text("Message"); ImGui::NextColumn();
                    // ImGui::Text("Function"); ImGui::NextColumn();
                    // ImGui::Separator();

                    static std::list<rosgraph_msgs::Log>::iterator selectedLog;
                    for (auto msgIt = warningLogList.begin(); msgIt != warningLogList.end(); msgIt++)
                    {
                        char logLabel[32];
                        sprintf(logLabel, "%s", msgIt->msg.c_str());
                        if (ImGui::Selectable(logLabel, selectedLog == msgIt, ImGuiSelectableFlags_SpanAllColumns))
                        {
                            selectedLog = msgIt;
                        }
                        // ImGui::NextColumn();
                        // ImGui::Text("%s", msgIt->function.c_str()); ImGui::NextColumn();
                    }
                    ImGui::Columns(1);
                }
                ImGui::EndChild();
            }
        }
        else if(selectedTab == 5)
        {
            {
                ImGui::BeginChild("btLog", ImVec2(600, 200), true, 0);

                ImGui::Text("Log");
                if(guiDebug)
                {
                    ImGui::SameLine(); ImGui::Text("List size: %lu", btLogList.size());
                }
            
                ImGui::Separator();
                
                {
                    ImGui::Columns(2, "btLogList");
                    ImGui::Separator();

                    ImGui::SetColumnWidth(0, 100); ImGui::Text("Function"); ImGui::NextColumn();
                    ImGui::SetColumnWidth(1, 400); ImGui::Text("Message"); ImGui::NextColumn();

                    ImGui::Separator();


                    static std::list<rosgraph_msgs::Log>::iterator selectedLog;
                    for (auto msgIt = btLogList.begin(); msgIt != btLogList.end(); msgIt++)
                    {
                        char logLabel[32];
                        sprintf(logLabel, "%s", msgIt->function.c_str());
                        if (ImGui::Selectable(logLabel, selectedLog == msgIt, ImGuiSelectableFlags_SpanAllColumns))
                        {
                            selectedLog = msgIt;
                        }
                        ImGui::NextColumn();
                        ImGui::Text("%s", msgIt->msg.c_str()); ImGui::NextColumn();
                    }
                    ImGui::Columns(1);
                }
                ImGui::EndChild();
            }
            {
                ImGui::BeginChild("mainLogUnfiltered", ImVec2(600, 200), true, 0);

                ImGui::Text("Log unfiltered");
                ImGui::Separator();
                ImGui::Columns(1, "unLogList");
                ImGui::Separator();

                // ImGui::SetColumnWidth(0, 100);
                // ImGui::SetColumnWidth(1, 400);
                // ImGui::Text("Message"); ImGui::NextColumn();
                // ImGui::Text("Function"); ImGui::NextColumn();

                ImGui::Separator();
                static std::list<rosgraph_msgs::Log>::iterator selectedLog;
                for (auto msgIt = mainLogList.begin(); msgIt != mainLogList.end(); msgIt++)
                {
                    char logLabel[32];
                    sprintf(logLabel, "%s", msgIt->msg.c_str());
                    if (ImGui::Selectable(logLabel, selectedLog == msgIt, ImGuiSelectableFlags_SpanAllColumns))
                    {
                        selectedLog = msgIt;
                    }
                    // ImGui::NextColumn();
                    // ImGui::Text("%s", msgIt->function.c_str()); ImGui::NextColumn();
                }
                ImGui::Columns(1);
                ImGui::EndChild();
            }
        }
    }
    
    ImGui::End();
}
void SamLogWidget::callbackLog(const rosgraph_msgs::Log& msg)
{
    const int mainListLimit = 100;
    const int errorListLimit = 50;
    const int warningListLimit = 50;
    const int btListLimit = 50;
    
    const uint8_t msgLevel = msg.level;
    const bool aboutRosBridge = strcmp(msg.name.c_str(), "/sam/rosbridge_websocket") == 0 ? true : false;

    if(!aboutRosBridge || msgLevel >= 4)
    {
        mainLogList.push_front(msg);
        const int mainListOversize = mainLogList.size() - mainListLimit;
        for(int i = 0; i < mainListOversize; i++)
        {
            mainLogList.pop_back();
        }
    }

    const bool errorLog = msgLevel >= 8 ? true : false;
    if(errorLog)
    {
        errorLogList.push_front(msg);
        const int errorListOversize = errorLogList.size() - errorListLimit;
        for(int i = 0; i < errorListOversize; i++)
        {
            errorLogList.pop_back();
        }
    }

    const bool warningLog = msgLevel == 4 ? true : false;
    if(warningLog)
    {
        warningLogList.push_front(msg);
        const int warningListOversize = warningLogList.size() - warningListLimit;
        for(int i = 0; i < warningListOversize; i++)
        {
            warningLogList.pop_back();
        }
    }

    const bool aboutBT = strcmp(msg.name.c_str(), "/sam/sam_bt") == 0 ? true : false;
    if(aboutBT)
    {
        btLogList.push_front(msg);
        const int btListOversize = btLogList.size() - btListLimit;
        for(int i = 0; i < btListOversize; i++)
        {
            btLogList.pop_back();
        }
    }
}
// -------------------------------------- SamLogWidget end --------------------------------------

SamTeleopWidget::SamTeleopWidget(roswasm::NodeHandle& nh) : enabled(false) //, pub_timer(nullptr)
{
    angle_pub = nh.advertise<sam_msgs::ThrusterAngles>("core/thrust_vector_cmd", 1000);
    rpm_pub = nh.advertise<smarc_msgs::DualThrusterRPM>("core/thrusters_cmd", 1000);
    rpm1_pub = nh.advertise<smarc_msgs::ThrusterRPM>("core/thruster1_cmd", 1000);
    rpm2_pub = nh.advertise<smarc_msgs::ThrusterRPM>("core/thruster2_cmd", 1000);
    pub_timer = nh.createTimer(roswasm::Duration(0.08), std::bind(&SamTeleopWidget::pub_callback, this, std::placeholders::_1));
    pub_timer.stop();
}

void SamTeleopWidget::pub_callback(const ros::TimerEvent& e)
{
    if (enabled) {
        angle_pub.publish(angles_msg);
        // rpm_pub.publish(rpm_msg);
        rpm1_pub.publish(rpm1_msg);
        rpm2_pub.publish(rpm2_msg);
    }
}

void SamTeleopWidget::show_window(bool& show_teleop_window)
{
    ImGuiIO& io = ImGui::GetIO();
    ImVec4 col = ImGui::GetStyle().Colors[23];

    ImGui::SetNextWindowSize(ImVec2(400, 100), ImGuiCond_FirstUseEver);
    ImGui::Begin("Keyboard teleop", &show_teleop_window);

    angles_msg.thruster_vertical_radians = angles_msg.thruster_horizontal_radians = 0.0f;
    rpm_msg.thruster_front.rpm = rpm_msg.thruster_back.rpm = 0;
    rpm1_msg.rpm = rpm2_msg.rpm = 0;
    static int teleopRPM = 800;

    float sz = ImGui::GetTextLineHeight();
    ImGui::Columns(2, "teleopControls", false);
    ImGui::SetColumnWidth(0, 200);
    {
        ImGui::Checkbox("Teleop enabled", &enabled);
        if (enabled) {
            pub_timer.start();
            // pub_timer = new roswasm::Timer(0.08, std::bind(&SamTeleopWidget::pub_callback, this, std::placeholders::_1));
        }
        else { // if (!enabled && pub_timer != nullptr) {
            pub_timer.stop();
            // delete pub_timer;
            // pub_timer = nullptr;
        }
    }
    ImGui::NextColumn();
    {
        const int rpmCap = 1500;
        ImGui::SliderInt("rpm", &teleopRPM, 0, rpmCap); //, "%d", ImGuiSliderFlags_AlwaysClamp);  <-------------------- TODO: upgrade when imgui upgraded
        if(teleopRPM < 0)   // <-------------------- TODO: upgrade when imgui upgraded
        {
            teleopRPM = 0;
        }
        else if(teleopRPM > rpmCap)   // <-------------------- TODO: upgrade when imgui upgraded
        {
            teleopRPM = rpmCap;
        }
        char rpmToolTip[100];
        sprintf(rpmToolTip, "Set teleop rpm command\n"
                        "Capped @ %d rpm\n"
                        "CTRL+click to input value", rpmCap);
        ImGui::SameLine(); HelpMarker(rpmToolTip);
    }
    ImGui::NextColumn();
    // ImGui::Columns(1);
    ImGui::BeginGroup();
    // ImGui::BeginGroup();
    // ImGui::BeginChild("mainLogUnfiltered", ImVec2(600, 200), true, 0);
    // ImGui::Dummy(ImVec2(sz, 0.75f*sz));
    // ImGui::Checkbox("Teleop enabled", &enabled);
    // if (enabled && pub_timer == nullptr) {
    //     pub_timer = new roswasm::Timer(0.08, std::bind(&SamTeleopWidget::pub_callback, this, std::placeholders::_1));
    // }
    // else if (!enabled && pub_timer != nullptr) {
    //     delete pub_timer;
    //     pub_timer = nullptr;
    // }
    // ImGui::EndGroup();

    // ImGui::SameLine();

    ImGui::BeginGroup();
    ImGui::Dummy(ImVec2(sz, 1.5f*sz));
    bool key_down = enabled && io.KeysDownDuration[263] >= 0.0f;
    if (key_down) {
        ImGui::PushStyleColor(ImGuiCol_Button, col);
    }
    if (ImGui::ArrowButton("##left", ImGuiDir_Left) || key_down) {
        angles_msg.thruster_horizontal_radians = -0.10;
    }
    if (key_down) {
        ImGui::PopStyleColor();   
    }
    ImGui::EndGroup();
    ImGui::SameLine();
    ImGui::BeginGroup();
    key_down = enabled && io.KeysDownDuration[265] >= 0.0f;
    if (key_down) {
        ImGui::PushStyleColor(ImGuiCol_Button, col);
    }
    if (ImGui::ArrowButton("##up", ImGuiDir_Up) || key_down) {
        angles_msg.thruster_vertical_radians = 0.10;
    }
    if (key_down) {
        ImGui::PopStyleColor();   
    }
    key_down = enabled && io.KeysDownDuration[264] >= 0.0f;
    if (key_down) {
        ImGui::PushStyleColor(ImGuiCol_Button, col);
    }
    if (ImGui::ArrowButton("##down", ImGuiDir_Down) || key_down) {
        angles_msg.thruster_vertical_radians = -0.10;
    }
    if (key_down) {
        ImGui::PopStyleColor();   
    }
    ImGui::EndGroup();
    ImGui::SameLine();
    ImGui::BeginGroup();
    ImGui::Dummy(ImVec2(sz, 1.5f*sz));
    key_down = enabled && io.KeysDownDuration[262] >= 0.0f;
    if (key_down) {
        ImGui::PushStyleColor(ImGuiCol_Button, col);
    }
    if (ImGui::ArrowButton("##right", ImGuiDir_Right) || key_down) {
        angles_msg.thruster_horizontal_radians = 0.10;
    }
    if (key_down) {
        ImGui::PopStyleColor();   
    }
    ImGui::EndGroup();

    ImGui::SameLine();

    ImGui::BeginGroup();
    ImGui::Dummy(ImVec2(sz, sz));
    ImGui::Text("Thrust Vector");
    ImGui::EndGroup();

    // ImGui::SameLine();
    ImGui::NextColumn();


    ImGui::BeginGroup();
    key_down = enabled && io.KeysDownDuration[87] >= 0.0f;
    if (key_down) {
        ImGui::PushStyleColor(ImGuiCol_Button, col);
    }
    if (ImGui::Button("w") || key_down) {
        rpm1_msg.rpm = rpm2_msg.rpm = teleopRPM;
        rpm_msg.thruster_front.rpm = teleopRPM;
        rpm_msg.thruster_back.rpm = teleopRPM;
    }
    if (key_down) {
        ImGui::PopStyleColor();   
    }
    key_down = enabled && io.KeysDownDuration[83] >= 0.0f;
    if (key_down) {
        ImGui::PushStyleColor(ImGuiCol_Button, col);
    }
    if (ImGui::Button("s") || key_down) {
        rpm1_msg.rpm = rpm2_msg.rpm = -teleopRPM;
        rpm_msg.thruster_front.rpm = -teleopRPM;
        rpm_msg.thruster_back.rpm = -teleopRPM;
    }
    if (key_down) {
        ImGui::PopStyleColor();   
    }
    ImGui::EndGroup();
    ImGui::SameLine();
    ImGui::BeginGroup();
    ImGui::Text("Forward");
    ImGui::Dummy(ImVec2(sz, 0.5f*sz));
    ImGui::Text("Reverse");
    ImGui::EndGroup();
    ImGui::EndGroup();
    ImGui::Columns(1);

    ImGui::End();

    //ImGui::Text("Keys down:");      for (int i = 0; i < IM_ARRAYSIZE(io.KeysDown); i++) if (io.KeysDownDuration[i] >= 0.0f)     { ImGui::SameLine(); ImGui::Text("%d (%.02f secs)", i, io.KeysDownDuration[i]); }
}


} // namespace roswasm_webgui
