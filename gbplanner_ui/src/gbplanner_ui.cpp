#include <time.h>
#include <stdio.h>
#include "gbplanner_ui.h"
#include <voxblox_msgs/FilePath.h>

// pci_initialization_trigger
namespace gbplanner_ui {

gbplanner_panel::gbplanner_panel(QWidget* parent) : rviz::Panel(parent) {

  nh.param<std::string>("uav_name", uav_name_, "uav46");
  nh.param<std::string>("voxgraph_save_path", voxgraph_save_path_, "");
  nh.param<std::string>("voxblox_save_path", voxblox_save_path_, "");
  nh.param<std::string>("planning_graph_save_path", planning_graph_save_path_, "");

  planner_client_start_planner = nh.serviceClient<std_srvs::Trigger>("/" + uav_name_ + "/planner_control_interface/std_srvs/automatic_planning");
  planner_client_stop_planner = nh.serviceClient<std_srvs::Trigger>("/" + uav_name_ + "/planner_control_interface/std_srvs/stop");
  planner_client_homing = nh.serviceClient<std_srvs::Trigger>("/" + uav_name_ + "/planner_control_interface/std_srvs/homing_trigger");
  planner_client_init_motion = nh.serviceClient<planner_msgs::pci_initialization>("/" + uav_name_ + "/pci_initialization_trigger");
  planner_client_plan_to_waypoint = nh.serviceClient<std_srvs::Trigger>("/" + uav_name_ + "/planner_control_interface/std_srvs/go_to_waypoint");
  planner_client_global_planner = nh.serviceClient<planner_msgs::pci_global>("/" + uav_name_ + "/pci_global");
  planner_client_save_voxgraph = nh.serviceClient<voxblox_msgs::FilePath>("/" + uav_name_ + "/voxgraph_mapper/save_to_file");
  planner_client_save_voxblox = nh.serviceClient<voxblox_msgs::FilePath>("/" + uav_name_ + "/gbplanner_node/save_map");
  planner_client_save_planning_graph = nh.serviceClient<planner_msgs::planner_string_trigger>("/" + uav_name_ + "/gbplanner/save_graph");
  planner_client_load_voxblox_graph = nh.serviceClient<voxblox_msgs::FilePath>("/" + uav_name_ + "/gbplanner_node/load_map");
  planner_client_load_planning_graph = nh.serviceClient<planner_msgs::planner_string_trigger>("/" + uav_name_ + "/gbplanner/load_graph");

  QVBoxLayout* v_box_layout = new QVBoxLayout;

  button_start_planner = new QPushButton;
  button_stop_planner = new QPushButton;
  button_homing = new QPushButton;
  button_init_motion = new QPushButton;
  button_plan_to_waypoint = new QPushButton;
  button_global_planner = new QPushButton;
  button_save_voxgraph = new QPushButton;
  button_save_voxblox = new QPushButton;
  button_load_voxblox = new QPushButton;

  button_start_planner->setText("Start Planner");
  button_stop_planner->setText("Stop Planner");
  button_homing->setText("Go Home");
  button_init_motion->setText("Initialization");
  button_plan_to_waypoint->setText("Plan to Waypoint");
  button_global_planner->setText("Run Global");
  button_save_voxgraph->setText("Save Voxgraph");
  button_save_voxblox->setText("Save Voxblox");
  button_load_voxblox->setText("Load Voxblox");

  v_box_layout->addWidget(button_start_planner);
  v_box_layout->addWidget(button_stop_planner);
  v_box_layout->addWidget(button_homing);
  v_box_layout->addWidget(button_init_motion);
  v_box_layout->addWidget(button_plan_to_waypoint);
  v_box_layout->addWidget(button_save_voxgraph);
  v_box_layout->addWidget(button_save_voxblox);
  v_box_layout->addWidget(button_load_voxblox);

  QVBoxLayout* global_vbox_layout = new QVBoxLayout;
  QHBoxLayout* global_hbox_layout = new QHBoxLayout;

  QLabel* text_label_ptr = new QLabel("Frontier ID:");

  global_id_line_edit = new QLineEdit();

  global_hbox_layout->addWidget(text_label_ptr);
  global_hbox_layout->addWidget(global_id_line_edit);
  global_hbox_layout->addWidget(button_global_planner);
  global_vbox_layout->addLayout(global_hbox_layout);
  v_box_layout->addLayout(global_vbox_layout);

  setLayout(v_box_layout);

  connect(button_start_planner, SIGNAL(clicked()), this, SLOT(on_start_planner_click()));
  connect(button_stop_planner, SIGNAL(clicked()), this, SLOT(on_stop_planner_click()));
  connect(button_homing, SIGNAL(clicked()), this, SLOT(on_homing_click()));
  connect(button_init_motion, SIGNAL(clicked()), this, SLOT(on_init_motion_click()));
  connect(button_plan_to_waypoint, SIGNAL(clicked()), this, SLOT(on_plan_to_waypoint_click()));
  connect(button_global_planner, SIGNAL(clicked()), this, SLOT(on_global_planner_click()));
  connect(button_save_voxgraph, SIGNAL(clicked()), this, SLOT(on_save_voxgraph_click()));
  connect(button_save_voxblox, SIGNAL(clicked()), this, SLOT(on_save_voxblox_click()));
  connect(button_load_voxblox, SIGNAL(clicked()), this, SLOT(on_load_voxblox_click()));
}

void gbplanner_panel::on_save_voxgraph_click() {
  voxblox_msgs::FilePath srv;
  const std::string current_time = currentDateTime();
  srv.request.file_path = voxgraph_save_path_ + "/voxgraph-" + current_time + ".ply";
  ROS_INFO("[GBPLANNER-UI] Saving Voxgraph to file: %s", srv.request.file_path.c_str());

  if (!planner_client_save_voxgraph.call(srv)) {
    ROS_ERROR("[GBPLANNER-UI] Service call failed: %s", planner_client_save_voxgraph.getService().c_str());
  } else{
    ROS_INFO("[GBPLANNER-UI] Voxgraph saved to file");
  }
}

void gbplanner_panel::on_save_voxblox_click() {
  voxblox_msgs::FilePath srv;
  const std::string current_time = currentDateTime();

  std::string file_paths[2];
  file_paths[0] = voxblox_save_path_ + "/voxblox-" + current_time + ".vxblx";
  file_paths[1] = planning_graph_save_path_ + "/latest_voxblox.vxblx";

  for(std::string fname : file_paths){
    srv.request.file_path = fname;
    ROS_INFO("[GBPLANNER-UI] Saving Voxblox to file: %s", fname.c_str());

    if (!planner_client_save_voxblox.call(srv)) {
      ROS_ERROR("[GBPLANNER-UI] Service call failed: %s", planner_client_save_voxblox.getService().c_str());
    } else{
      ROS_INFO("[GBPLANNER-UI] Voxblox saved to %s", fname.c_str());
    }
  }

  file_paths[0] = planning_graph_save_path_ + "/global_planning-" + current_time + ".graph";
  file_paths[1] = planning_graph_save_path_ + "/latest_global_planning.graph";

  planner_msgs::planner_string_trigger srv_graph;
  for(std::string fname : file_paths){
    srv_graph.request.message = fname;
    if (!planner_client_save_planning_graph.call(srv_graph)) {
      ROS_ERROR("[GBPLANNER-UI] Service call failed: %s", planner_client_save_planning_graph.getService().c_str());
    } else{
      if (srv_graph.response.success)
        ROS_INFO("[GBPLANNER-UI] Planning Graph saved to %s", fname.c_str());
      else
        ROS_ERROR("[GBPLANNER-UI] Failed to save Planning Graph");
    }
  }

}

void gbplanner_panel::on_load_voxblox_click() {
  voxblox_msgs::FilePath srv;
  srv.request.file_path = voxblox_save_path_ + "/latest_voxblox.vxblx";
  ROS_INFO("[GBPLANNER-UI] Loading Voxblox from file: %s", srv.request.file_path.c_str());

  if (!planner_client_load_voxblox_graph.call(srv)) {
    ROS_ERROR("[GBPLANNER-UI] Service call failed: %s", planner_client_load_voxblox_graph.getService().c_str());
  } else{
    ROS_INFO("[GBPLANNER-UI] Voxblox loaded from file");
  }

  planner_msgs::planner_string_trigger srv_graph;
  srv_graph.request.message = planning_graph_save_path_ + "/latest_global_planning.graph";
  if (!planner_client_load_planning_graph.call(srv_graph)) {
    ROS_ERROR("[GBPLANNER-UI] Service call failed: %s", planner_client_load_planning_graph.getService().c_str());
  } else{
    if (srv_graph.response.success)
      ROS_INFO("[GBPLANNER-UI] Planning Graph loaded from file");
    else
      ROS_ERROR("[GBPLANNER-UI] Failed to load Planning Graph");
  }
}

void gbplanner_panel::on_start_planner_click() {
  std_srvs::Trigger srv;
  if (!planner_client_start_planner.call(srv)) {
    ROS_ERROR("[GBPLANNER-UI] Service call failed: %s", planner_client_start_planner.getService().c_str());
  }
}

void gbplanner_panel::on_stop_planner_click() {
  std_srvs::Trigger srv;
  if (!planner_client_stop_planner.call(srv)) {
    ROS_ERROR("[GBPLANNER-UI] Service call failed: %s", planner_client_stop_planner.getService().c_str());
  }
}

void gbplanner_panel::on_homing_click() {
  std_srvs::Trigger srv;
  if (!planner_client_homing.call(srv)) {
    ROS_ERROR("[GBPLANNER-UI] Service call failed: %s", planner_client_homing.getService().c_str());
  }
}

void gbplanner_panel::on_init_motion_click() {
  planner_msgs::pci_initialization srv;
  if (!planner_client_init_motion.call(srv)) {
    ROS_ERROR("[GBPLANNER-UI] Service call failed: %s", planner_client_init_motion.getService().c_str());
  }
}

void gbplanner_panel::on_plan_to_waypoint_click() {
  std_srvs::Trigger srv;
  if (!planner_client_plan_to_waypoint.call(srv)) {
    ROS_ERROR("[GBPLANNER-UI] Service call failed: %s", planner_client_plan_to_waypoint.getService().c_str());
  }
}

void gbplanner_panel::on_global_planner_click() {
  // retrieve ID as a string
  std::string in_string = global_id_line_edit->text().toStdString();
  // global_id_line_edit->clear();
  int id = -1;
  if (in_string.empty())
    id = 0;
  else {
    // try to convert to an integer
    try {
      id = std::stoi(in_string);
    } catch (const std::out_of_range& exc) {
      ROS_ERROR("[GBPLANNER UI] - Invalid ID: %s", in_string.c_str());
      return;
    } catch (const std::invalid_argument& exc) {
      ROS_ERROR("[GBPLANNER UI] - Invalid ID: %s", in_string.c_str());
      return;
    }
  }
  // check bounds on integer
  if (id < 0) {
    ROS_ERROR("[GBPLANNER UI] - In valid ID, must be non-negative");
    return;
  }
  // we got an ID!!!!!!!!!
  ROS_INFO("Global Planner found ID : %i", id);

  planner_msgs::pci_global plan_srv;
  plan_srv.request.id = id;
  if (!planner_client_global_planner.call(plan_srv)) {
    ROS_ERROR("[GBPLANNER-UI] Service call failed: %s", planner_client_global_planner.getService().c_str());
  }
}
void gbplanner_panel::save(rviz::Config config) const { rviz::Panel::save(config); }
void gbplanner_panel::load(const rviz::Config& config) { rviz::Panel::load(config); }

const std::string gbplanner_panel::currentDateTime() {
    time_t     now = time(0);
    struct tm  tstruct;
    char       buf[80];
    tstruct = *localtime(&now);
    // Visit http://en.cppreference.com/w/cpp/chrono/c/strftime
    // for more information about date/time format
    strftime(buf, sizeof(buf), "%Y_%m_%d_%H-%M-%S", &tstruct);

    return buf;
}

}  // namespace gbplanner_ui

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(gbplanner_ui::gbplanner_panel, rviz::Panel)
