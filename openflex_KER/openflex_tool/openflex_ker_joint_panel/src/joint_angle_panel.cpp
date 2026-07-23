#include "openflex_ker_joint_panel/joint_angle_panel.hpp"

#include <algorithm>
#include <cmath>
#include <utility>
#include <vector>

#include <QGroupBox>
#include <QHeaderView>
#include <QHBoxLayout>
#include <QDockWidget>
#include <QLayout>
#include <QSizePolicy>
#include <QSplitter>
#include <QVBoxLayout>
#include <pluginlib/class_list_macros.hpp>

namespace openflex_ker_joint_panel {
namespace {
constexpr double kRadiansToDegrees = 180.0 / 3.14159265358979323846;

QTableWidget *makeJointTable(QWidget *parent) {
  auto *table = new QTableWidget(8, 3, parent);
  table->setHorizontalHeaderLabels(
    {QStringLiteral("关节"), QStringLiteral("rad"), QStringLiteral("deg")});
  table->verticalHeader()->setVisible(false);
  table->horizontalHeader()->setSectionResizeMode(0, QHeaderView::Stretch);
  table->horizontalHeader()->setSectionResizeMode(1, QHeaderView::ResizeToContents);
  table->horizontalHeader()->setSectionResizeMode(2, QHeaderView::ResizeToContents);
  table->verticalHeader()->setSectionResizeMode(QHeaderView::Stretch);
  table->setEditTriggers(QAbstractItemView::NoEditTriggers);
  table->setSelectionMode(QAbstractItemView::NoSelection);
  table->setAlternatingRowColors(true);
  table->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
  table->setMinimumHeight(80);
  for (int row = 0; row < 8; ++row) {
    table->setItem(row, 0, new QTableWidgetItem(
      row < 7 ? QStringLiteral("J%1").arg(row + 1) : QStringLiteral("夹爪")));
    table->setItem(row, 1, new QTableWidgetItem(QStringLiteral("--")));
    table->setItem(row, 2, new QTableWidgetItem(QStringLiteral("--")));
    table->item(row, 1)->setTextAlignment(Qt::AlignRight | Qt::AlignVCenter);
    table->item(row, 2)->setTextAlignment(Qt::AlignRight | Qt::AlignVCenter);
  }
  return table;
}
}  // namespace

JointAnglePanel::JointAnglePanel(QWidget *parent) : rviz_common::Panel(parent) {
  setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
  setMinimumHeight(0);
  setupUi();
}

void JointAnglePanel::setupUi() {
  auto *root = new QVBoxLayout();
  root->setSizeConstraint(QLayout::SetNoConstraint);
  root->setContentsMargins(8, 8, 8, 8);
  root->setSpacing(6);

  auto *topic_row = new QHBoxLayout();
  topic_edit_ = new QLineEdit(QString::fromStdString(topic_));
  topic_edit_->setToolTip(QStringLiteral("sensor_msgs/msg/JointState 话题"));
  apply_button_ = new QPushButton(QStringLiteral("应用"));
  topic_row->addWidget(topic_edit_);
  topic_row->addWidget(apply_button_);
  root->addLayout(topic_row);

  auto *ping_row = new QHBoxLayout();
  usb_ping_button_ = new QPushButton(QStringLiteral("USB Ping"));
  usb_ping_button_->setToolTip(QStringLiteral("通过 USB 检测 KER 并读取固件信息"));
  wifi_ping_button_ = new QPushButton(QStringLiteral("WiFi Ping"));
  wifi_ping_button_->setToolTip(QStringLiteral("通过局域网检测 KER 并读取固件信息"));
  float_button_ = new QPushButton(QStringLiteral("浮动窗口"));
  float_button_->setToolTip(QStringLiteral("浮动后可从四个方向调整面板大小"));
  ping_row->addWidget(usb_ping_button_);
  ping_row->addWidget(wifi_ping_button_);
  ping_row->addWidget(float_button_);
  root->addLayout(ping_row);

  status_label_ = new QLabel(QStringLiteral("等待 RViz 初始化"));
  status_label_->setStyleSheet(
    "QLabel { color: #374151; background: #f3f4f6; padding: 6px; border-radius: 4px; }");
  root->addWidget(status_label_);

  ping_status_label_ = new QLabel(QStringLiteral("设备信息: 未 Ping"));
  ping_status_label_->setWordWrap(true);
  ping_status_label_->setStyleSheet(
    "QLabel { color: #374151; background: #f3f4f6; padding: 6px; border-radius: 4px; }");
  root->addWidget(ping_status_label_);

  auto *right_group = new QGroupBox(QStringLiteral("右臂"));
  auto *right_layout = new QVBoxLayout();
  right_table_ = makeJointTable(right_group);
  right_layout->addWidget(right_table_);
  right_group->setLayout(right_layout);
  right_group->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);

  auto *left_group = new QGroupBox(QStringLiteral("左臂"));
  auto *left_layout = new QVBoxLayout();
  left_table_ = makeJointTable(left_group);
  left_layout->addWidget(left_table_);
  left_group->setLayout(left_layout);
  left_group->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);

  auto *arms_splitter = new QSplitter(Qt::Vertical);
  arms_splitter->setChildrenCollapsible(false);
  arms_splitter->addWidget(right_group);
  arms_splitter->addWidget(left_group);
  arms_splitter->setStretchFactor(0, 1);
  arms_splitter->setStretchFactor(1, 1);
  root->addWidget(arms_splitter, 1);
  setLayout(root);

  connect(apply_button_, &QPushButton::clicked, this, &JointAnglePanel::applyTopic);
  connect(usb_ping_button_, &QPushButton::clicked, this, &JointAnglePanel::pingUsb);
  connect(wifi_ping_button_, &QPushButton::clicked, this, &JointAnglePanel::pingWifi);
  connect(float_button_, &QPushButton::clicked, this, &JointAnglePanel::toggleFloating);
  connect(topic_edit_, &QLineEdit::returnPressed, this, &JointAnglePanel::applyTopic);
}

void JointAnglePanel::onInitialize() {
  node_ = std::make_shared<rclcpp::Node>("openflex_ker_joint_panel");
  usb_ping_client_ = node_->create_client<std_srvs::srv::Trigger>("/ker/ping_usb");
  wifi_ping_client_ = node_->create_client<std_srvs::srv::Trigger>("/ker/ping_wifi");
  subscribe();

  spin_timer_ = new QTimer(this);
  connect(spin_timer_, &QTimer::timeout, this, [this]() {
    if (node_) {
      rclcpp::spin_some(node_);
    }
  });
  spin_timer_->start(20);

  status_timer_ = new QTimer(this);
  connect(status_timer_, &QTimer::timeout, this, &JointAnglePanel::refreshStatus);
  status_timer_->start(200);

  if (auto *dock = findDockWidget()) {
    connect(dock, &QDockWidget::topLevelChanged, this, [this](bool floating) {
      float_button_->setText(floating ? QStringLiteral("停靠窗口") : QStringLiteral("浮动窗口"));
    });
  }
}

QDockWidget *JointAnglePanel::findDockWidget() const {
  QWidget *widget = parentWidget();
  while (widget != nullptr) {
    if (auto *dock = qobject_cast<QDockWidget *>(widget)) {
      return dock;
    }
    widget = widget->parentWidget();
  }
  return nullptr;
}

void JointAnglePanel::toggleFloating() {
  auto *dock = findDockWidget();
  if (dock == nullptr) {
    status_label_->setText(QStringLiteral("无法找到 RViz 停靠窗口"));
    return;
  }
  const bool floating = !dock->isFloating();
  dock->setFloating(floating);
  if (floating) {
    dock->resize(std::max(dock->width(), 520), std::max(dock->height(), 680));
  }
}

void JointAnglePanel::subscribe() {
  if (!node_) {
    return;
  }
  subscription_.reset();
  subscription_ = node_->create_subscription<sensor_msgs::msg::JointState>(
    topic_, rclcpp::SensorDataQoS(),
    [this](sensor_msgs::msg::JointState::SharedPtr message) { jointStateCallback(std::move(message)); });
  last_message_ = {};
  rate_window_start_ = std::chrono::steady_clock::now();
  messages_in_window_ = 0;
  message_rate_hz_ = 0.0;
  status_label_->setText(QStringLiteral("等待 %1").arg(QString::fromStdString(topic_)));
}

void JointAnglePanel::applyTopic() {
  const auto candidate = topic_edit_->text().trimmed();
  if (candidate.isEmpty()) {
    return;
  }
  topic_ = candidate.toStdString();
  subscribe();
}

void JointAnglePanel::pingUsb() {
  pingTransport(QStringLiteral("USB"), usb_ping_client_, usb_ping_button_);
}

void JointAnglePanel::pingWifi() {
  pingTransport(QStringLiteral("WiFi"), wifi_ping_client_, wifi_ping_button_);
}

void JointAnglePanel::pingTransport(
  const QString &transport,
  const rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr &client,
  QPushButton *button)
{
  if (!client || button == nullptr) {
    return;
  }
  if (!client->service_is_ready()) {
    ping_status_label_->setStyleSheet(
      "QLabel { color: #991b1b; background: #fee2e2; padding: 6px; border-radius: 4px; }");
    ping_status_label_->setText(
      QStringLiteral("%1 Ping 失败: ROS 服务不可用").arg(transport));
    return;
  }
  button->setEnabled(false);
  ping_status_label_->setText(QStringLiteral("正在执行 %1 Ping...").arg(transport));
  auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
  client->async_send_request(
    request, [this, transport, button](
      rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future) {
      const auto response = future.get();
      button->setEnabled(true);
      if (response->success) {
        ping_status_label_->setStyleSheet(
          "QLabel { color: #166534; background: #dcfce7; padding: 6px; border-radius: 4px; }");
        ping_status_label_->setText(
          QStringLiteral("%1 Ping 成功 | %2")
            .arg(transport, QString::fromStdString(response->message)));
      } else {
        ping_status_label_->setStyleSheet(
          "QLabel { color: #991b1b; background: #fee2e2; padding: 6px; border-radius: 4px; }");
        ping_status_label_->setText(
          QStringLiteral("%1 Ping 失败 | %2")
            .arg(transport, QString::fromStdString(response->message)));
      }
    });
}

void JointAnglePanel::jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr message) {
  if (message->name.size() != message->position.size()) {
    status_label_->setText(QStringLiteral("消息错误: name 与 position 长度不同"));
    return;
  }
  updateTable(*message);
  const auto now = std::chrono::steady_clock::now();
  last_message_ = now;
  ++messages_in_window_;
  const double elapsed = std::chrono::duration<double>(now - rate_window_start_).count();
  if (elapsed >= 1.0) {
    message_rate_hz_ = static_cast<double>(messages_in_window_) / elapsed;
    messages_in_window_ = 0;
    rate_window_start_ = now;
  }
}

void JointAnglePanel::updateTable(const sensor_msgs::msg::JointState &message) {
  for (std::size_t index = 0; index < message.name.size(); ++index) {
    const auto &name = message.name[index];
    QTableWidget *table = nullptr;
    int row = -1;
    const std::string ker_right_prefix = "ker_right_joint";
    const std::string ker_left_prefix = "ker_left_joint";
    const std::string robot_right_prefix = "openarmx_right_joint";
    const std::string robot_left_prefix = "openarmx_left_joint";
    if (name.rfind(ker_right_prefix, 0) == 0) {
      table = right_table_;
      row = std::stoi(name.substr(ker_right_prefix.size())) - 1;
    } else if (name.rfind(robot_right_prefix, 0) == 0) {
      table = right_table_;
      row = std::stoi(name.substr(robot_right_prefix.size())) - 1;
    } else if (name == "ker_right_gripper" ||
               name == "openarmx_right_finger_joint1") {
      table = right_table_;
      row = 7;
    } else if (name.rfind(ker_left_prefix, 0) == 0) {
      table = left_table_;
      row = std::stoi(name.substr(ker_left_prefix.size())) - 1;
    } else if (name.rfind(robot_left_prefix, 0) == 0) {
      table = left_table_;
      row = std::stoi(name.substr(robot_left_prefix.size())) - 1;
    } else if (name == "ker_left_gripper" ||
               name == "openarmx_left_finger_joint1") {
      table = left_table_;
      row = 7;
    }
    if (table == nullptr || row < 0 || row >= 8) {
      continue;
    }
    const double radians = message.position[index];
    table->item(row, 1)->setText(QString::number(radians, 'f', 4));
    table->item(row, 2)->setText(QString::number(radians * kRadiansToDegrees, 'f', 1));
  }
}

void JointAnglePanel::refreshStatus() {
  if (last_message_ == std::chrono::steady_clock::time_point{}) {
    return;
  }
  const double age = std::chrono::duration<double>(
    std::chrono::steady_clock::now() - last_message_).count();
  if (age > 1.0) {
    status_label_->setStyleSheet(
      "QLabel { color: #991b1b; background: #fee2e2; padding: 6px; border-radius: 4px; }");
    status_label_->setText(QStringLiteral("数据超时  %1 s").arg(age, 0, 'f', 1));
  } else {
    status_label_->setStyleSheet(
      "QLabel { color: #166534; background: #dcfce7; padding: 6px; border-radius: 4px; }");
    status_label_->setText(
      QStringLiteral("在线  %1 Hz  |  延迟 %2 ms")
        .arg(message_rate_hz_, 0, 'f', 1).arg(age * 1000.0, 0, 'f', 0));
  }
}

void JointAnglePanel::load(const rviz_common::Config &config) {
  rviz_common::Panel::load(config);
  QString stored_topic;
  if (config.mapGetString("joint_topic", &stored_topic) && !stored_topic.isEmpty()) {
    topic_ = stored_topic.toStdString();
    topic_edit_->setText(stored_topic);
    subscribe();
  }
}

void JointAnglePanel::save(rviz_common::Config config) const {
  rviz_common::Panel::save(config);
  config.mapSetValue("joint_topic", QString::fromStdString(topic_));
}

}  // namespace openflex_ker_joint_panel

PLUGINLIB_EXPORT_CLASS(openflex_ker_joint_panel::JointAnglePanel, rviz_common::Panel)
