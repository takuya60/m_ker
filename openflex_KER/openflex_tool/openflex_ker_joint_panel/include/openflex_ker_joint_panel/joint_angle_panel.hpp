#pragma once

#include <chrono>
#include <memory>
#include <string>

#include <QLabel>
#include <QLineEdit>
#include <QDockWidget>
#include <QCheckBox>
#include <QDoubleSpinBox>
#include <QPushButton>
#include <QTableWidget>
#include <QTabWidget>
#include <QTimer>

#include <rclcpp/rclcpp.hpp>
#include <rviz_common/panel.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_srvs/srv/trigger.hpp>

namespace openflex_ker_joint_panel {

class JointAnglePanel : public rviz_common::Panel {
  Q_OBJECT

 public:
  explicit JointAnglePanel(QWidget *parent = nullptr);
  ~JointAnglePanel() override = default;

  void onInitialize() override;
  void load(const rviz_common::Config &config) override;
  void save(rviz_common::Config config) const override;

 private Q_SLOTS:
  void applyTopic();
  void pingUsb();
  void pingWifi();
  void toggleFloating();
  void refreshStatus();
  void refreshParameters();
  void applyParameters();

 private:
  void setupUi();
  void subscribe();
  void pingTransport(
    const QString &transport,
    const rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr &client,
    QPushButton *button);
  void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr message);
  void updateTable(const sensor_msgs::msg::JointState &message);
  void finishParameterOperation(bool success, const QString &message);
  QDockWidget *findDockWidget() const;

  QTabWidget *tabs_{nullptr};
  QLineEdit *topic_edit_{nullptr};
  QPushButton *apply_button_{nullptr};
  QPushButton *usb_ping_button_{nullptr};
  QPushButton *wifi_ping_button_{nullptr};
  QPushButton *float_button_{nullptr};
  QLabel *status_label_{nullptr};
  QLabel *ping_status_label_{nullptr};
  QTableWidget *left_table_{nullptr};
  QTableWidget *right_table_{nullptr};
  QCheckBox *low_pass_enabled_{nullptr};
  QCheckBox *hampel_enabled_{nullptr};
  QDoubleSpinBox *low_pass_alpha_{nullptr};
  QDoubleSpinBox *joint_velocity_{nullptr};
  QDoubleSpinBox *gripper_velocity_{nullptr};
  QDoubleSpinBox *target_timeout_{nullptr};
  QPushButton *refresh_parameters_button_{nullptr};
  QPushButton *apply_parameters_button_{nullptr};
  QLabel *parameter_status_label_{nullptr};
  QTimer *spin_timer_{nullptr};
  QTimer *status_timer_{nullptr};

  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscription_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr usb_ping_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr wifi_ping_client_;
  std::shared_ptr<rclcpp::AsyncParametersClient> driver_parameters_;
  std::shared_ptr<rclcpp::AsyncParametersClient> bridge_parameters_;
  std::string topic_{"/ker/joint_states"};
  std::chrono::steady_clock::time_point last_message_{};
  std::chrono::steady_clock::time_point rate_window_start_{};
  int messages_in_window_{0};
  double message_rate_hz_{0.0};
  int pending_parameter_operations_{0};
  bool parameter_operation_failed_{false};
  QString parameter_operation_errors_;
};

}  // namespace openflex_ker_joint_panel
