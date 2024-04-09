#ifndef GBPLANNER_UI_H
#define GBPLANNER_UI_H

#include <stdio.h>

#include <planner_msgs/pci_global.h>
#include <planner_msgs/pci_initialization.h>
#include <planner_msgs/planner_string_trigger.h>
#include <ros/ros.h>
#include <std_msgs/ColorRGBA.h>
#include <std_srvs/Empty.h>
#include <std_srvs/Trigger.h>

#ifndef Q_MOC_RUN
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QPainter>
#include <QPushButton>
#include <QTimer>
#include <QVBoxLayout>
#include <rviz/panel.h>
#endif

class QLineEdit;
class QPushButton;

namespace gbplanner_ui {
class gbplanner_panel : public rviz::Panel {
  Q_OBJECT
 public:
  gbplanner_panel(QWidget* parent = 0);
  virtual void load(const rviz::Config& config);
  virtual void save(rviz::Config config) const;

 public Q_SLOTS:
  void on_start_planner_click();
  void on_stop_planner_click();
  void on_homing_click();
  void on_init_motion_click();
  void on_plan_to_waypoint_click();
  void on_global_planner_click();
  void on_save_voxgraph_click();
  void on_save_voxblox_click();
  void on_load_voxblox_click();
 protected Q_SLOTS:

 protected:
  std::string uav_name_;
  std::string voxblox_save_path_;
  std::string voxgraph_save_path_;
  std::string planning_graph_save_path_;

 protected:
  QPushButton* button_start_planner;
  ros::ServiceClient planner_client_start_planner;

  QPushButton* button_stop_planner;
  ros::ServiceClient planner_client_stop_planner;

  QPushButton* button_homing;
  ros::ServiceClient planner_client_homing;

  QPushButton* button_init_motion;
  ros::ServiceClient planner_client_init_motion;

  QPushButton* button_plan_to_waypoint;
  ros::ServiceClient planner_client_plan_to_waypoint;

  QPushButton* button_global_planner;
  QLineEdit* global_id_line_edit;
  ros::ServiceClient planner_client_global_planner;

  QPushButton* button_save_voxgraph;
  ros::ServiceClient planner_client_finalize_voxgraph;
  ros::ServiceClient planner_client_save_voxgraph;

  QPushButton* button_save_voxblox;
  ros::ServiceClient planner_client_save_voxblox;
  ros::ServiceClient planner_client_save_planning_graph;

  QPushButton* button_load_voxblox;
  ros::ServiceClient planner_client_load_voxblox_graph;
  ros::ServiceClient planner_client_load_planning_graph;

  ros::NodeHandle nh;

  const std::string currentDateTime();
};

}  // namespace gbplanner_ui

#endif  // GBPLANNER_UI_H
