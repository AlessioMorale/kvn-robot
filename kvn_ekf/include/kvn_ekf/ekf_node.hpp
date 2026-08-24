// Copyright (c) 2024, Alessio Morale
// Licensed under the MIT License.

#ifndef KVN_EKF__EKF_NODE_HPP_
#define KVN_EKF__EKF_NODE_HPP_

#include <Eigen/Core>
#include <Eigen/Dense>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/transform_broadcaster.h>

namespace kvn_ekf
{

/**
 * @class EKFNode
 * @brief Extended Kalman Filter for multi-sensor fusion
 *
 * Fuses:
 * - Wheel odometry (/odom) - low-drift position, drifts on yaw
 * - IMU gyroscope (/imu) - high-quality yaw rate, sensitive to bias
 *
 * Produces: Filtered /odom_ekf with lower drift, better covariance
 */
class EKFNode : public rclcpp::Node
{
public:
  explicit EKFNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  // State vector: [x, y, theta, v_x, v_y, omega_z]
  // 6D state: position (2D) + orientation (1D) + velocity (2D) + angular velocity (1D)
  Eigen::Matrix<double, 6, 1> x_;  // State estimate
  Eigen::Matrix<double, 6, 6> P_;  // Covariance (uncertainty)

  // Kalman matrices
  Eigen::Matrix<double, 6, 6> F_;  // Motion model Jacobian
  Eigen::Matrix<double, 6, 6> Q_;  // Process noise covariance
  Eigen::Matrix<double, 3, 6> H_odom_;  // Observation Jacobian (odometry)
  Eigen::Matrix<double, 1, 6> H_imu_;   // Observation Jacobian (IMU gyro)
  Eigen::Matrix<double, 3, 3> R_odom_;  // Measurement noise (odometry)
  Eigen::Matrix<double, 1, 1> R_imu_;   // Measurement noise (IMU)

  // ROS subscriptions and publishers
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscription_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_ekf_publisher_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  // Last measurement timestamps
  rclcpp::Time last_odom_time_;
  rclcpp::Time last_imu_time_;
  rclcpp::Time last_predict_time_;

  // Parameters (tunable)
  double process_noise_position_;
  double process_noise_yaw_;
  double process_noise_velocity_;
  double process_noise_angular_velocity_;
  double measurement_noise_odom_position_;
  double measurement_noise_odom_yaw_;
  double measurement_noise_imu_yaw_rate_;
  double max_measurement_gap_;
  bool use_imu_updates_;
  bool ekf_enabled_;

  // Callbacks
  void on_odometry(const nav_msgs::msg::Odometry::SharedPtr msg);
  void on_imu(const sensor_msgs::msg::Imu::SharedPtr msg);

  // EKF core operations
  void predict(double dt);
  void update_odometry(const Eigen::Matrix<double, 3, 1> & z);
  void update_imu(double z);
  void publish_filtered_odometry();

  // Helper functions
  double normalize_angle(double angle);
  void initialize_matrices();
};

}  // namespace kvn_ekf

#endif  // KVN_EKF__EKF_NODE_HPP_
