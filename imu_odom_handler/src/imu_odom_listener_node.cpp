#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "Eigen/Dense"

using std::placeholders::_1;

class ImuOdomHandler : public rclcpp::Node
{
  public:
    ImuOdomHandler(): Node("ImuOdomHandler"), previous_yaw_(0.0), previous_velo_(0.0)
    {
        set_initial_values();
        imu_subscription_ = this->create_subscription<sensor_msgs::msg::Imu>("demo/imu", 10, std::bind(&ImuOdomHandler::imu_callback, this, _1));
        odom_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>("demo/odom", 10, std::bind(&ImuOdomHandler::odom_callback, this, _1));
    }


  private:
    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        if(output_)
        {
            RCLCPP_INFO(this->get_logger(), "#######################IMU#######################");

            RCLCPP_INFO(this->get_logger(), "IMU: header frame id: '%s'", msg->header.frame_id);
            RCLCPP_INFO(this->get_logger(), "IMU: header stamp nanosec: '%d'", msg->header.stamp.nanosec);
            RCLCPP_INFO(this->get_logger(), "IMU: header stamp sec: '%d'", msg->header.stamp.sec);

            RCLCPP_INFO(this->get_logger(), "IMU: linear acc x componenet: '%f'", msg->linear_acceleration.x);
            RCLCPP_INFO(this->get_logger(), "IMU: linear acc y componenet: '%f'", msg->linear_acceleration.y);
            RCLCPP_INFO(this->get_logger(), "IMU: linear acc z componenet: '%f'", msg->linear_acceleration.z);
            for (int i = 0; i < 9; i++)
            {
                RCLCPP_INFO(this->get_logger(), "IMU: linear acc covariance: '%f'", msg->linear_acceleration_covariance[i]);
            }

            RCLCPP_INFO(this->get_logger(), "IMU: angular velocity x component: '%f'", msg->angular_velocity.x);
            RCLCPP_INFO(this->get_logger(), "IMU: angular velocity y component: '%f'", msg->angular_velocity.y);
            RCLCPP_INFO(this->get_logger(), "IMU: angular velocity z component: '%f'", msg->angular_velocity.z);
            for (int i = 0; i < 9; i++)
            {
                RCLCPP_INFO(this->get_logger(), "IMU: linear angular velocity covariance: '%f'", msg->angular_velocity_covariance[i]);
            }

            RCLCPP_INFO(this->get_logger(), "IMU: quaternion x component: '%f'", msg->orientation.x);
            RCLCPP_INFO(this->get_logger(), "IMU: quaternion y component: '%f'", msg->orientation.y);
            RCLCPP_INFO(this->get_logger(), "IMU: quaternion z component: '%f'", msg->orientation.z);
            RCLCPP_INFO(this->get_logger(), "IMU: quaternion w component: '%f'", msg->orientation.w);

            for (int i = 0; i < 9; i++)
            {
                RCLCPP_INFO(this->get_logger(), "IMU: linear orientation covariance: '%f'", msg->orientation_covariance[i]);
            }
        }

        double yaw_angle = std::atan2(-msg->linear_acceleration.y, msg->linear_acceleration.x);
        double yaw_rate = (yaw_angle - previous_yaw_) / delta_t_; 
        double acc_x = msg->linear_acceleration.x;
        double acc_y = msg->linear_acceleration.y;
        double acc = get_2d_value(acc_x, acc_y);
        double velo = acc * delta_t_ + previous_velo_;
        process_model(acc, yaw_rate, velo, yaw_angle);
        
        previous_yaw_ = yaw_angle;
        previous_velo_ = velo;
        
    }

    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        if(output_)
        {
            RCLCPP_INFO(this->get_logger(), "#######################ODOM#######################");
            RCLCPP_INFO(this->get_logger(), "ODOM: header frame id: '%s'", msg->header.frame_id);
            RCLCPP_INFO(this->get_logger(), "ODOM: header stamp nanosec: '%d'", msg->header.stamp.nanosec);
            RCLCPP_INFO(this->get_logger(), "ODOM: header stamp sec: '%d'", msg->header.stamp.sec);

            RCLCPP_INFO(this->get_logger(), "ODOM: position x component: '%f'", msg->pose.pose.position.x);
            RCLCPP_INFO(this->get_logger(), "ODOM: position y component: '%f'", msg->pose.pose.position.y);
            RCLCPP_INFO(this->get_logger(), "ODOM: position z component: '%f'", msg->pose.pose.position.z);

            RCLCPP_INFO(this->get_logger(), "ODOM: orientation x component: '%f'", msg->pose.pose.orientation.x);
            RCLCPP_INFO(this->get_logger(), "ODOM: orientation y component: '%f'", msg->pose.pose.orientation.y);
            RCLCPP_INFO(this->get_logger(), "ODOM: orientation z component: '%f'", msg->pose.pose.orientation.z);
            RCLCPP_INFO(this->get_logger(), "ODOM: orientation w component: '%f'", msg->pose.pose.orientation.w);
            
            for (int i = 0; i < 36; i++)
            {
                RCLCPP_INFO(this->get_logger(), "ODOM: pose covariance: '%f'", msg->pose.covariance[i]);
            }

            RCLCPP_INFO(this->get_logger(), "ODOM: linear twist x component: '%f'", msg->twist.twist.linear.x);
            RCLCPP_INFO(this->get_logger(), "ODOM: linear twist y component: '%f'", msg->twist.twist.linear.y);
            RCLCPP_INFO(this->get_logger(), "ODOM: linear twist z component: '%f'", msg->twist.twist.linear.z);

            RCLCPP_INFO(this->get_logger(), "ODOM: anguler twist x component: '%f'", msg->twist.twist.angular.x);
            RCLCPP_INFO(this->get_logger(), "ODOM: anguler twist y component: '%f'", msg->twist.twist.angular.y);
            RCLCPP_INFO(this->get_logger(), "ODOM: anguler twist z component: '%f'", msg->twist.twist.angular.z);
            
            for (int i = 0; i < 36; i++)
            {
                RCLCPP_INFO(this->get_logger(), "ODOM: twist covariance: '%f'", msg->twist.covariance[i]);
            }
        }
        
    }

    double get_2d_value(double x, double y) const
    {
        return std::sqrt(x*x + y*y);
    }

    const void process_model(double acc, double yaw_rate, double velo, double yaw_angle)
    {
        control_input_ << acc, yaw_rate, velo*std::cos(yaw_angle), velo*std::sin(yaw_angle);   
        state_.transpose() += delta_t_ *  control_input_.transpose();

        jacobian_fx_ << 1, 0, 0, 0,
                        0, 1, 0, 0,
                        delta_t_*std::cos(yaw_angle), -delta_t_*velo*std::sin(yaw_angle), 1, 0,
                        delta_t_*std::sin(yaw_angle), delta_t_*velo*std::cos(yaw_angle), 0, 1;
        
        //TODO: investigate the process model noise Q matrix
        covariance_ = jacobian_fx_ * covariance_ * jacobian_fx_.transpose() + 
                      jacobian_fw_ * 0 * jacobian_fw_.transpose();

    }

    void set_initial_values()
    {
        state_.setZero();
        jacobian_fx_.setZero();
        jacobian_fw_.setIdentity();
        covariance_.setZero();
        previous_yaw_ = 0.0;
        previous_velo_ = 0.0;
    }

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscription_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
    Eigen::VectorXd state_ = Eigen::VectorXd(4);
    Eigen::VectorXd control_input_ = Eigen::VectorXd(4);
    Eigen::MatrixXd jacobian_fx_ = Eigen::MatrixXd(4,4);
    Eigen::MatrixXd jacobian_fw_ = Eigen::MatrixXd(4,4);
    Eigen::MatrixXd covariance_ = Eigen::MatrixXd(4,4);
    Eigen::VectorXd measurement();
    Eigen::MatrixXd measurement_covariance();
    
    //Eigen::VectorXf initial_state_ = Eigen::VectorXf(15);
    
    double delta_t_ = 0.1;
    double previous_yaw_;
    double previous_velo_;
    bool output_ = false;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ImuOdomHandler>());
  rclcpp::shutdown();
  return 0;
}