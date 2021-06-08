#include <stdio.h>

#ifdef __EMSCRIPTEN__
#include <emscripten.h>
#endif

#include <roswasm_webgui/imgui/imgui.h>
#include <roswasm_webgui/imgui/imgui_impl_glfw.h>
#include <roswasm_webgui/imgui/imgui_impl_opengl3.h>

#ifdef ROSWASM_NATIVE
#include <GL/glew.h> 
#else
#define GLFW_INCLUDE_ES3
#include <GLES3/gl3.h>
#endif
#include <GLFW/glfw3.h>

#include <roswasm/roswasm.h>
#include <roswasm_webgui/roswasm_monlaunch.h>
#include <roswasm_webgui/roswasm_image.h>
#include <roswasm_webgui/roswasm_examples.h>

#include <sam_webgui/roswasm_sam.h>

roswasm::NodeHandle* nh;
roswasm_webgui::MonlaunchWidget* monlaunch_widget;
roswasm_webgui::ImageWidget* image_widget;
roswasm_webgui::SamActuatorWidget* actuator_widget;
roswasm_webgui::SamDashboardWidget* dashboard_widget;
// roswasm_webgui::SamDashboardWidget2* dashboard_widget2;
roswasm_webgui::SamTeleopWidget* teleop_widget;
roswasm_webgui::SamMonitorWidget* monitor_widget;
roswasm_webgui::SamLogWidget* roslog_widget;

GLFWwindow* g_window;
ImVec4 clear_color = ImVec4(0.25f, 0.45f, 0.55f, 1.00f);
ImVec4 emergency_color = ImVec4(1.0f, 0.0f, 0.0f, 1.00f);
ImVec4 warning_color = ImVec4(0.87f, 0.57f, 0.0f, 1.00f);
ImVec4 good_color = ImVec4(0.0f, 0.71f, 0.06f, 1.00f);
std::string colorString = "";
float colorR, colorG, colorB;
float col[4] = {0,0,0,0};
int winSpacing = 30;
int winWindth1 = 470;
// static int guiMode = 100;
static int guiMode = 200;
static int guiModeOld = -1;
// const std::vector<const char*> tabNames{"Setup", "Monitor", "Control", "Service", "Experiments"};
const std::map<int, const char*> guiModes = {{100, "Setup"}, {200, "Monitor"}, {250, "Cameras"}, {300, "Control"}, {400, "Service"}, {999, "Develop"}};
// const std::map<const char*, int> guiModes = {{"Setup", 100}, {"Monitor", 200}, {"Cameras", 250}, {"Control", 300}, {"Service", 400}, {"Experiments", 999}};
std::map<const char*, std::vector<int>> nodeStates;

ImGuiContext* imgui = 0;
bool show_demo_window = false;
bool show_monlaunch_window = true;
bool show_image_window = true;
bool show_actuator_window = true;
bool show_dashboard_window = true;
bool show_experiment_dash_window = true;
bool show_teleop_window = false;
bool show_monitor_window = false;
bool show_guiStats_window = false;
bool show_roslog_window = true;

// bool dark_mode = true;
bool dark_mode = true;
bool emergency = false;
bool guiDebug = false;
bool nodeCrashed = false;
bool publishPanic = false;

roswasm::Publisher* panic_pub;
// roswasm::Timer* panic_pub_timer;

// void pub_panic_callback(const ros::TimerEvent& e)
// {
//   std_msgs::String msg;
//   msg.data = "ROS_GUI";
//   panic_pub->publish(msg);
// }

// Forward declarations
int drawTabs(int _guiMode, const std::map<int, const char*> _modeMap);

#ifndef ROSWASM_NATIVE
EM_JS(int, canvas_get_width, (), {
    return Module.canvas.width;
});

EM_JS(int, canvas_get_height, (), {
    return Module.canvas.height;
});

EM_JS(void, resizeCanvas, (), {
    js_resizeCanvas();
});
#endif

void loop()
{
    if (glfwWindowShouldClose(g_window)) {
        roswasm::shutdown();
        return;
    }

  // if (publishPanic && panic_pub_timer == nullptr) {
  //   // panic_pub_timer = new roswasm::Timer(0.08, std::bind(&pub_panic_callback, std::placeholders::_1));
  //   panic_pub_timer = new roswasm::Timer(0.08, pub_panic_callback);
  //   // panic_pub_timer = new roswasm::Timer(0.08, std::bind(&pub_panic_callback, this, std::placeholders::_1));
  // }

#ifndef ROSWASM_NATIVE
  int width = canvas_get_width();
  int height = canvas_get_height();
  glfwSetWindowSize(g_window, width, height);
    // glfwSetWindowSize(g_window, canvas_get_width(), canvas_get_height());
#endif
    glfwPollEvents();

  //ImGui::SetCurrentContext(imgui);

  ImGui_ImplOpenGL3_NewFrame();
  ImGui_ImplGlfw_NewFrame();
  ImGui::NewFrame();

  {
#ifndef ROSWASM_NATIVE
    ImGui::SetNextWindowPos(ImVec2(width-210, height-70), ImGuiCond_FirstUseEver);
#else
  ImGui::SetNextWindowPos(ImVec2(30, 30), ImGuiCond_FirstUseEver);
#endif
    ImGui::SetNextWindowSize(ImVec2(200, 60), ImGuiCond_FirstUseEver);
    ImGui::Begin("Debug");
    ImGui::Checkbox("Debug mode", &guiDebug); ImGui::SameLine(120);
    ImGui::Checkbox("Stats", &show_guiStats_window);
    ImGui::End();
  }

  {
    ImGui::SetNextWindowPos(ImVec2(winSpacing, winSpacing), ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowSize(ImVec2(winWindth1, 200), ImGuiCond_FirstUseEver);
    ImGui::Begin("Roswasm webgui"); //, &show_another_window);

    
    // ImGui::Button("", ImVec2(13, 13));
    // ImGui::SameLine();
    // float sz = ImGui::GetTextLineHeight();
    std::string status_text;
    // ImColor status_color;
    ImVec4 status_color4;
    if (nh->ok()) {
#ifdef ROSWASM_NATIVE
        status_text = "ROS OK";
#else
        status_text = "Connected! " + nh->get_websocket_url();
#endif
        status_color4 = ImColor(0, 255, 0);
    }
    else {
#ifdef ROSWASM_NATIVE
        status_text = "ROS NOT OK!";
#else
        status_text = "Disconnected! " + nh->get_websocket_url();
#endif
        status_color4 = ImColor(255, 0, 0);
    }
    // ImVec2 p = ImGui::GetCursorScreenPos();
    // ImGui::GetWindowDrawList()->AddRectFilled(p, ImVec2(p.x+sz, p.y+sz), status_color);
    // ImGui::Dummy(ImVec2(sz, sz));
    // ImVec4 status_color4 = ImColor(0, 255, 0);
    ImGui::AlignTextToFramePadding();
    ImGui::PushID(23);
    ImGui::PushStyleColor(ImGuiCol_Button, status_color4);
    ImGui::PushStyleColor(ImGuiCol_ButtonHovered, status_color4);
    ImGui::PushStyleColor(ImGuiCol_ButtonActive, status_color4);
    ImGui::Button("", ImVec2(18,18));
    ImGui::PopStyleColor(3);
    ImGui::PopID();
    ImGui::SameLine();
    ImGui::AlignTextToFramePadding();
    ImGui::Text("%s", status_text.c_str());
    // ImGui::Text("Connected");
    ImGui::AlignTextToFramePadding();

    monlaunch_widget->get_states(nodeStates);

    nodeCrashed = false;
    int nodesCrashed = 0;
    int nodesRunning = 0;
    int nodesTotal = 0;
    for (auto LF:nodeStates)
    {
      for(auto _state:LF.second) {
        nodesTotal++;
        if(_state == 2) {
          nodeCrashed = true;
          nodesCrashed++;
        } else if(_state == 1) {
          nodesRunning++;
        }
      }
    }
    const int _buttonWidth = 150;
    const int availWidth = ImGui::GetContentRegionAvailWidth();
    ImGui::SameLine(availWidth-_buttonWidth+8);
    ImGui::PushID(1);
    char label[50];
    if (nodesTotal && nh->ok()){
      // const int _buttonWidth = 150;
      // const int availWidth = ImGui::GetContentRegionAvailWidth();
      // ImGui::SameLine();
      // ImGui::SameLine(availWidth-_buttonWidth+8);
      // ImGui::PushID(1);
      if(nodesCrashed)
      {
        ImGui::PushStyleColor(ImGuiCol_Button, emergency_color);
        ImGui::PushStyleColor(ImGuiCol_ButtonHovered, emergency_color);
        ImGui::PushStyleColor(ImGuiCol_ButtonActive, emergency_color);
        if(nodesCrashed > 1)
        {
          sprintf(label, "Nodes crashed: %d", nodesCrashed);
        }
        else {
          sprintf(label, "Node crashed!");
        }
      }
      else if(nodesRunning)
      {
        ImGui::PushStyleColor(ImGuiCol_Button, good_color);
        ImGui::PushStyleColor(ImGuiCol_ButtonHovered, good_color);
        ImGui::PushStyleColor(ImGuiCol_ButtonActive, good_color);
        sprintf(label, "Nodes running: %d/%d", nodesRunning, nodesTotal);
      }
      else
      {
        ImGui::PushStyleColor(ImGuiCol_Button, warning_color);
        ImGui::PushStyleColor(ImGuiCol_ButtonHovered, warning_color);
        ImGui::PushStyleColor(ImGuiCol_ButtonActive, warning_color);
        sprintf(label, "No nodes running");
      }
    }
    else
    {
      ImGui::PushStyleColor(ImGuiCol_Button, emergency_color);
      ImGui::PushStyleColor(ImGuiCol_ButtonHovered, emergency_color);
      ImGui::PushStyleColor(ImGuiCol_ButtonActive, emergency_color);
      sprintf(label, "No nodes detected!");
    }
    if(ImGui::Button(label, ImVec2(_buttonWidth,18))){
      // if(ImGui::SmallButton(label)){
      show_monlaunch_window = true;
    }
    ImGui::PopStyleColor(3);
    ImGui::PopID();

    // ImGui::Button("5x5", ImVec2(5, 5));
    // ImGui::SameLine();
    // ImGui::Button("8x8", ImVec2(8, 8));
    // ImGui::SameLine();
    // ImGui::Button("10x10", ImVec2(10, 10));
    // ImGui::SameLine();
    // ImGui::Button("11x11", ImVec2(11, 11));
    // ImGui::SameLine();
    // ImGui::Button("12x12", ImVec2(12, 5));
    // ImGui::SameLine();
    // ImGui::Button("13x13", ImVec2(13, 13));
    // ImGui::SameLine();
    // ImGui::Button("Button()");
    // ImGui::SameLine();
    // ImGui::SmallButton("SmallButton()");


    // if (nodeCrashed){
    //   const int _buttonWidth = 120;
    //   const int availWidth = ImGui::GetContentRegionAvailWidth();
    //   ImGui::SameLine(availWidth-_buttonWidth+8);
    //   ImGui::PushID(1);
    //   ImGui::PushStyleColor(ImGuiCol_Button, emergency_color);
    //   ImGui::PushStyleColor(ImGuiCol_ButtonHovered, emergency_color);
    //   ImGui::PushStyleColor(ImGuiCol_ButtonActive, emergency_color);
    //   ImGui::AlignTextToFramePadding();
    //   if(ImGui::Button("Node crashed!", ImVec2(_buttonWidth,15))){
    //       show_monlaunch_window = true;
    //   }
    //   ImGui::PopStyleColor(3);
    //   ImGui::PopID();
    // }

    guiMode = drawTabs(guiMode, guiModes);

    if (guiMode != guiModeOld)  //  100=Setup, 200=Monitor, 300=Control, 400=Service, 999=Develop
    {
      guiModeOld = guiMode;
      if (guiMode == 100) // Setup
      {
        show_demo_window = false;
        show_monlaunch_window = true;
        show_image_window = true;
        show_actuator_window = true;
        show_dashboard_window = true;
        show_experiment_dash_window = false;
        show_teleop_window = true;
        show_monitor_window = false;
        show_roslog_window = true;
      }
      else if (guiMode == 200)  // Monitor
      {
        show_demo_window = false;
        show_monlaunch_window = false;
        show_image_window = true;
        show_actuator_window = false;
        show_dashboard_window = true;
        show_experiment_dash_window = false;
        show_teleop_window = false;
        show_monitor_window = true;
        show_roslog_window = true;
      }
      else if (guiMode == 300)  // Control
      {
        show_demo_window = false;
        show_monlaunch_window = false;
        show_image_window = true;
        show_actuator_window = false;
        show_dashboard_window = true;
        show_experiment_dash_window = false;
        show_teleop_window = true;
        show_monitor_window = true;
        show_roslog_window = true;
      }
    }

    // bool testy = false;
    // if (ImGui::BeginTabBar("tabby", ImGuiButtonFlags_None))
    // {
    //     if (ImGui::BeginTabItem("hmm", testy, ImGuiButtonFlags_None))
    //     {
    //         ImGui::EndTabItem();
    //     }
    //     ImGui::EndTabBar();
    // }
    // Menu


    // Window display options
    const int buttonSpacing = 150;
    ImGui::Checkbox("Launch control", &show_monlaunch_window);
    ImGui::SameLine(buttonSpacing); ImGui::Checkbox("Monitor", &show_monitor_window);
    ImGui::SameLine(2*buttonSpacing); bool pushDark = ImGui::Checkbox("Dark mode", &dark_mode);
    ImGui::Checkbox("Image topic", &show_image_window);
    ImGui::SameLine(buttonSpacing); ImGui::Checkbox("Actuator controls", &show_actuator_window);
    ImGui::SameLine(2*buttonSpacing); ImGui::Checkbox("Status dashboard", &show_dashboard_window);
    ImGui::Checkbox("Log", &show_roslog_window);
    ImGui::SameLine(buttonSpacing); ImGui::Checkbox("Keyboard teleop", &show_teleop_window);
    ImGui::SameLine(2*buttonSpacing); ImGui::Checkbox("Demo widgets", &show_demo_window);      // Edit bools storing our windows open/close state

    if(pushDark){
      if(dark_mode){
        ImGui::StyleColorsDark();
      } else {
        ImGui::StyleColorsLight();
      }
    }

    // ImGui::PushID(23);
    // ImGui::PushStyleColor(ImGuiCol_Button, warning_color);
    // ImGui::PushStyleColor(ImGuiCol_ButtonHovered, warning_color);
    // ImGui::PushStyleColor(ImGuiCol_ButtonActive, warning_color);
    // if(ImGui::Button("Abort!", ImVec2(50,40)))
    // {
    //   publishPanic = true;
    //   std_msgs::String msg;
    //   // msg.data = "ROS_GUI";
    //   std::stringstream ss;
    //   ss << "ROS_GUI";
    //   msg.data = ss.str();
    //   panic_pub->publish(msg);
    // }
    // ImGui::PopStyleColor(3);
    // ImGui::PopID();
    // ImGui::SameLine();
    // ImGui::Text("%d", publishPanic);

    if(guiDebug)
    {
      ImGui::ColorEdit3("Background", (float*)&clear_color); // Edit 3 floats representing a color
      ImGui::ColorEdit3("Picky", col); // Edit 3 floats representing a color
      ImGui::Text("Color H:%f S: %f V:%f", col[0], col[1], col[2]);
      colorString = "HSV(";
      colorString += std::to_string(col[0]);
      colorString += ", ";
      colorString += std::to_string(col[1]);
      colorString += ", ";
      colorString += std::to_string(col[2]);
      colorString += ")";

      char *_chars = strdup(colorString.c_str());
      ImGui::InputText("Color", _chars, 64);
      float colorH, colorS, colorV;
      // ImGui::InputFloat("R", &colorR, 0.0f, 255.0f);
      // ImGui::InputFloat("G", &colorG, 0.0f, 255.0f);
      // ImGui::InputFloat("B", &colorB, 0.0f, 255.0f);
      // ImGui::ColorConvertRGBtoHSV(colorR, colorG, colorB, colorH, colorS, colorV);
      // ImGui::Text("Color H:%f S: %f V:%f", colorH, colorS, colorV);
    }

    ImGui::End();
  }

  if (show_guiStats_window) {
    ImGui::SetNextWindowPos(ImVec2(800, 60), ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowSize(ImVec2(winWindth1, 400), ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowBgAlpha(0.7f);
    // ImGui::Begin("Gui stats");
    // ImGui::Text("Application average %.3f ms/frame (%.1f FPS)", 1000.0f / ImGui::GetIO().Framerate, ImGui::GetIO().Framerate);
    // ImGui::Text("Active windows %d", ImGui::GetIO().MetricsActiveWindows);
    // // ImGui::Text("Visible windows %d", ImGui::GetIO().MetricsRenderWindows);
    // ImGui::Text("Vertices %d", ImGui::GetIO().MetricsRenderVertices);
    // ImGui::Text("Indices %d", ImGui::GetIO().MetricsRenderIndices);
    // ImGui::Text("Mouse x:%.3f, y:%.3f", ImGui::GetIO().MousePos[0], ImGui::GetIO().MousePos[1]);
    // ImGui::End();
    ImGui::ShowMetricsWindow();
  }

  if (show_monlaunch_window) {
    ImGui::SetNextWindowPos(ImVec2(winSpacing, 260), ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowSize(ImVec2(winWindth1, 500), ImGuiCond_FirstUseEver);
    monlaunch_widget->show_window(show_monlaunch_window);
  }

  if (show_image_window) {
    ImGui::SetNextWindowPos(ImVec2(1072, winSpacing), ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowSize(ImVec2(winWindth1, 430), ImGuiCond_FirstUseEver);
    image_widget->show_window(show_image_window);
  }

  if (show_demo_window) {
      ImGui::SetNextWindowPos(ImVec2(600, 60), ImGuiCond_FirstUseEver);
      ImGui::ShowDemoWindow(&show_demo_window);
  }

  if (show_actuator_window) {
      ImGui::SetNextWindowPos(ImVec2(winWindth1+2*winSpacing, 290), ImGuiCond_FirstUseEver);
      actuator_widget->show_window(show_actuator_window);
  }

  if (show_dashboard_window) {
      ImGui::SetNextWindowPos(ImVec2(winWindth1+2*winSpacing, winSpacing), ImGuiCond_FirstUseEver);
      dashboard_widget->show_window(show_dashboard_window);
  }

  // if (show_experiment_dash_window) {
  //     ImGui::SetNextWindowPos(ImVec2(800, 120), ImGuiCond_FirstUseEver);
  //     dashboard_widget2->show_window(show_experiment_dash_window);
  // }

  if (show_teleop_window) {
      ImGui::SetNextWindowPos(ImVec2(winWindth1+2*winSpacing, 820), ImGuiCond_FirstUseEver);
      teleop_widget->show_window(show_teleop_window);
  }

  if (show_monitor_window) {
      ImGui::SetNextWindowPos(ImVec2(winSpacing, 280), ImGuiCond_FirstUseEver);
      monitor_widget->show_window(show_monitor_window, guiDebug);
  }

  if (show_roslog_window) {
      ImGui::SetNextWindowPos(ImVec2(1072, 500), ImGuiCond_FirstUseEver);
      roslog_widget->show_window(show_roslog_window, guiDebug);
  }

  ImGui::Render();

  int display_w, display_h;
  //glfwMakeContextCurrent(g_window);
  glfwGetFramebufferSize(g_window, &display_w, &display_h);
  glViewport(0, 0, display_w, display_h);
  if (dashboard_widget->is_emergency()) {
      glClearColor(emergency_color.x, emergency_color.y, emergency_color.z, emergency_color.w);
      emergency = true;
  }
  else {
      glClearColor(clear_color.x, clear_color.y, clear_color.z, clear_color.w);
  }
  glClear(GL_COLOR_BUFFER_BIT);

  ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
  //glfwMakeContextCurrent(g_window);
  glfwSwapBuffers(g_window);
}

static void glfw_error_callback(int error, const char* description)
{
  fprintf(stderr, "Glfw Error %d: %s\n", error, description);
}

int drawTabs(int _guiMode, const std::map<int, const char*> _modeMap){
    // extern volatile bool guiDebug;
    const int availWidth = ImGui::GetContentRegionAvailWidth();
    const int numModes = _modeMap.size();
    const ImGuiStyle& style = ImGui::GetStyle();
    const int buttonWidth = (availWidth-3*style.FramePadding.x)/numModes - style.FramePadding.x;
    if(guiDebug){
    ImGui::Text("Available width: %d", availWidth);
    ImGui::Text("Number of modes: %d", numModes);
    ImGui::Text("Padding x: %f, y: %f", style.FramePadding.x, style.FramePadding.y);
    ImGui::Text("Button width: %d", buttonWidth);
    }
    // GUI mode selection
    for (auto gm:_modeMap)
    {
      if (gm.first != _modeMap.begin()->first) ImGui::SameLine();
      ImGui::PushID(gm.first);
      if(_guiMode == gm.first) {
          ImGui::PushStyleColor(ImGuiCol_Button, ImGui::GetColorU32(ImGuiCol_FrameBgActive));
      } else {
          ImGui::PushStyleColor(ImGuiCol_Button, ImGui::GetColorU32(ImGuiCol_FrameBg));
      }
      ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImGui::GetColorU32(ImGuiCol_FrameBgHovered));
      ImGui::PushStyleColor(ImGuiCol_ButtonActive, ImGui::GetColorU32(ImGuiCol_FrameBgActive));
      if(ImGui::Button(gm.second, ImVec2(buttonWidth,20))){
          _guiMode = gm.first;
      }
      ImGui::PopStyleColor(3);
      ImGui::PopID();
    }
    return _guiMode;
}

int init()
{
  glfwSetErrorCallback(glfw_error_callback);
  if( !glfwInit() )
  {
      fprintf( stderr, "Failed to initialize GLFW\n" );
      return 1;
  }

  //glfwWindowHint(GLFW_SAMPLES, 4); // 4x antialiasing
  //glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE); // We don't want the old OpenGL
  // GL 3.0 + GLSL 130
  const char* glsl_version = "#version 130";
  glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
  glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 0);

  // Open a window and create its OpenGL context
  int canvasWidth = 800;
  int canvasHeight = 600;
  g_window = glfwCreateWindow( canvasWidth, canvasHeight, "SAM GUI", NULL, NULL);
  if( g_window == NULL )
  {
    fprintf( stderr, "Failed to open GLFW window.\n" );
    glfwTerminate();
    return -1;
  }

  glfwMakeContextCurrent(g_window); // Initialize GLEW


#ifdef ROSWASM_NATIVE
    glfwSwapInterval(1); // Enable vsync

    // Initialize OpenGL loader
    bool err = glewInit() != GLEW_OK;
    if (err)
    {
        fprintf(stderr, "Failed to initialize OpenGL loader!\n");
        return 1;
    }
#endif
  // Create game objects
  // Setup Dear ImGui binding
  IMGUI_CHECKVERSION();
  ImGui::CreateContext();
  ImGuiIO& io = ImGui::GetIO(); (void)io;

  // Setup style
  // ImGui::StyleColorsLight();
  // ImGui::StyleColorsDark();
  // ImGui::StyleColorsClassic();
  if(dark_mode){
        ImGui::StyleColorsDark();
      } else {
        ImGui::StyleColorsLight();
      }

  //ImGui_ImplGlfw_InitForOpenGL(g_window, false);
  ImGui_ImplGlfw_InitForOpenGL(g_window, true);
  //ImGui_ImplOpenGL3_Init();
  ImGui_ImplOpenGL3_Init(glsl_version);

  // Load Fonts
  io.Fonts->AddFontDefault();
  /*
  io.Fonts->AddFontFromFileTTF("data/xkcd-script.ttf", 23.0f);
  io.Fonts->AddFontFromFileTTF("data/xkcd-script.ttf", 18.0f);
  io.Fonts->AddFontFromFileTTF("data/xkcd-script.ttf", 26.0f);
  io.Fonts->AddFontFromFileTTF("data/xkcd-script.ttf", 32.0f);
  */

  //imgui =  ImGui::GetCurrentContext();

  // Cursor callbacks
  /*
  glfwSetMouseButtonCallback(g_window, ImGui_ImplGlfw_MouseButtonCallback);
  glfwSetScrollCallback(g_window, ImGui_ImplGlfw_ScrollCallback);
  glfwSetKeyCallback(g_window, ImGui_ImplGlfw_KeyCallback);
  glfwSetCharCallback(g_window, ImGui_ImplGlfw_CharCallback);
  */

#ifndef ROSWASM_NATIVE
  resizeCanvas();
#endif

  // panic_pub = nh->advertise<std_msgs::String>("core/panic_cmd");
  return 0;
}

void quit()
{
  // Cleanup
  ImGui_ImplOpenGL3_Shutdown();
  ImGui_ImplGlfw_Shutdown();
  ImGui::DestroyContext();

  glfwDestroyWindow(g_window);
  glfwTerminate();
}

extern "C" int main(int argc, char** argv)
{
    if (init() != 0) return 1;

    roswasm::init(argc, argv, "roswasm_webgui");

  roswasm::init(argc, argv, "roswasm_webgui");

#ifdef ROSWASM_NATIVE
  nh = new roswasm::NodeHandle();
#else
  if (argc < 3) {
      printf("Rosbridge server ip and port not provided!");
      return 1;
  }

  std::string rosbridge_ip(argv[1]);
  std::string rosbridge_port(argv[2]);
  nh = new roswasm::NodeHandle(rosbridge_ip, rosbridge_port);
#endif

  monlaunch_widget = new roswasm_webgui::MonlaunchWidget(*nh);
  image_widget = new roswasm_webgui::ImageWidget(*nh);
  actuator_widget = new roswasm_webgui::SamActuatorWidget(*nh);
  dashboard_widget = new roswasm_webgui::SamDashboardWidget(*nh);
  // dashboard_widget2 = new roswasm_webgui::SamDashboardWidget2(*nh);
  teleop_widget = new roswasm_webgui::SamTeleopWidget(*nh);
  monitor_widget = new roswasm_webgui::SamMonitorWidget(*nh);
  roslog_widget = new roswasm_webgui::SamLogWidget(*nh);

  // panic_pub = nh->advertise<std_msgs::String>("core/panic_cmd");

  // #ifdef __EMSCRIPTEN__
  // emscripten_set_main_loop(loop, 20, 1);
  // #endif

  roswasm::Duration loop_rate(1./20.);
  roswasm::spinLoop(loop, loop_rate);
  //roswasm::spinLoop(loop);

    quit();

    return 0;
}
