#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <turtlesim/msg/pose.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/int16.hpp>
#include <Eigen/Dense>
#include <memory>
#include <vector>
#include <deque>
#include <chrono>

// Keyboard handling code from teleop_turtle_key.cpp
#include <signal.h>
#include <stdio.h>
#ifndef _WIN32
# include <termios.h>
# include <unistd.h>
#else
# include <windows.h>
#endif

#define KEYCODE_W 0x77
#define KEYCODE_A 0x61
#define KEYCODE_S 0x73
#define KEYCODE_D 0x64
#define KEYCODE_X 0x78
#define KEYCODE_1 0x31
#define KEYCODE_2 0x32
#define KEYCODE_3 0x33
#define KEYCODE_Q 0x71
#define KEYCODE_E 0x65
#define KEYCODE_I 0x69
#define KEYCODE_K 0x6B
#define KEYCODE_M 0x6D
#define KEYCODE_SPACE 0x20

class KeyboardReader {
public:
    KeyboardReader() {
#ifndef _WIN32
        tcgetattr(0, &cooked);
        struct termios raw;
        memcpy(&raw, &cooked, sizeof(struct termios));
        raw.c_lflag &=~ (ICANON | ECHO);
        raw.c_cc[VEOL] = 1;
        raw.c_cc[VEOF] = 2;
        tcsetattr(0, TCSANOW, &raw);
#endif
    }

    bool readOne(char * c) {
#ifndef _WIN32
        int rc = read(0, c, 1);
        if (rc < 0) {
            return false;
        }
        return true;
#else
        // Windows implementation
        for(;;) {
            HANDLE handle = GetStdHandle(STD_INPUT_HANDLE);
            INPUT_RECORD buffer;
            DWORD events;
            PeekConsoleInput(handle, &buffer, 1, &events);
            if(events > 0) {
                ReadConsoleInput(handle, &buffer, 1, &events);
                *c = buffer.Event.KeyEvent.wVirtualKeyCode;
                return true;
            }
            return false;
        }
#endif
    }

    void shutdown() {
#ifndef _WIN32
        tcsetattr(0, TCSANOW, &cooked);
#endif
    }

private:
#ifndef _WIN32
    struct termios cooked;
#endif
};

class Turtle {
public:
    Turtle(rclcpp::Node* node, const std::string& name, bool is_real = false) 
        : node_(node), name_(name), is_real_(is_real) {
        // Initialize state variables
        position_ = Eigen::Vector2d::Zero();
        theta_ = 0.0;
        l_ = 1.0;  // Initialize with a default value
        
        // Create publishers
        cmd_vel_pub_ = node_->create_publisher<geometry_msgs::msg::Twist>(
            "/" + name + "/cmd_vel", 10);
            
        if (is_real) {
            cw_pub_ = node_->create_publisher<std_msgs::msg::Bool>(
                "/robot" + name.substr(name.length()-1) + "/gpio_output_27", 10);
            pwm_pub_ = node_->create_publisher<std_msgs::msg::Int16>(
                "/robot" + name.substr(name.length()-1) + "/gpio_pwm_17", 10);
        }
        
        // Create subscriber
        pose_sub_ = node_->create_subscription<turtlesim::msg::Pose>(
            "/" + name + "/pose",
            10,
            [this](const turtlesim::msg::Pose::SharedPtr msg) {
                this->poseCallback(msg);
            });
    }

    void updateTargetTheta(double target_theta, double base_angle, double control_angular_velocity) {
        // Check if l_ is too small to avoid division by zero
        if (std::abs(l_) < 1e-6) {
            target_theta_ = target_theta + base_angle;
        } else {
            target_theta_ = target_theta + base_angle + 3.0/l_ * control_angular_velocity;
        }
        
        double angle_diff = std::fmod(target_theta_ - theta_, 2 * M_PI);
        if (angle_diff < 0) angle_diff += 2 * M_PI;
        if (angle_diff > M_PI) angle_diff -= 2 * M_PI;
        
        target_angular_velocity_ = K_p_ * angle_diff;
        
        // Log the calculated values
        RCLCPP_INFO(node_->get_logger(), "Turtle %s: l=%.3f, target_theta=%.3f, angle_diff=%.3f, angular_vel=%.3f",
            name_.c_str(), l_, target_theta_, angle_diff, target_angular_velocity_);
    }

    void setTarget(double linear_vel, double target_theta, double base_angle, double control_angular_velocity) {
        target_linear_velocity_ = linear_vel;
        
        // Check if l_ is too small to avoid division by zero
        if (std::abs(l_) < 1e-6) {
            target_theta_ = target_theta;  // Use target_theta directly without adding base_angle
        } else {
            target_theta_ = target_theta + 3.0/l_ * control_angular_velocity;  // Add only the control term
        }
        
        double angle_diff = std::fmod(target_theta_ - theta_, 2 * M_PI);
        if (angle_diff < 0) angle_diff += 2 * M_PI;
        if (angle_diff > M_PI) angle_diff -= 2 * M_PI;
        
        target_angular_velocity_ = K_p_ * angle_diff;
        
        // Log the calculated values
        RCLCPP_INFO(node_->get_logger(), "Turtle %s: l=%.3f, target_theta=%.3f, angle_diff=%.3f, angular_vel=%.3f",
            name_.c_str(), l_, target_theta_, angle_diff, target_angular_velocity_);
    }

    void poseCallback(const turtlesim::msg::Pose::SharedPtr msg) {
        position_[0] = msg->x;
        position_[1] = msg->y;
        theta_ = msg->theta + M_PI;
        
        // Log position information
        // RCLCPP_INFO(node_->get_logger(), "Turtle %s position: x=%.2f, y=%.2f, theta=%.2f", 
            // name_.c_str(), position_[0], position_[1], theta_);
    }

    void publishVelocity() {
        auto twist = std::make_unique<geometry_msgs::msg::Twist>();
        twist->linear.x = target_linear_velocity_;
        twist->linear.y = 0.0;
        twist->linear.z = 0.0;
        twist->angular.x = 0.0;
        twist->angular.y = 0.0;
        twist->angular.z = target_angular_velocity_;
        cmd_vel_pub_->publish(std::move(twist));
        RCLCPP_INFO(node_->get_logger(), "Turtle %s velocity: linear = %f, angular = %f", name_.c_str(), target_linear_velocity_, target_angular_velocity_);
    }

    void publishMotorControl(bool cw_flag, int pwm) {
        if (is_real_) {
            auto cw_msg = std::make_unique<std_msgs::msg::Bool>();
            cw_msg->data = cw_flag;
            cw_pub_->publish(std::move(cw_msg));

            auto pwm_msg = std::make_unique<std_msgs::msg::Int16>();
            pwm_msg->data = pwm;
            pwm_pub_->publish(std::move(pwm_msg));
        }
    }

    const Eigen::Vector2d& getPosition() const { return position_; }
    double getTheta() const { return theta_; }
    double getL() const { return l_; }
    void setL(double l) { l_ = l; }
    void setPoseTheta(double theta) { pose_theta_ = theta; }
    double getPoseTheta() const { return pose_theta_; }
    const std::string& getName() const { return name_; }

private:
    rclcpp::Node* node_;
    std::string name_;
    bool is_real_;
    
    Eigen::Vector2d position_;
    double theta_;
    
    double target_linear_velocity_{0.0};
    double target_angular_velocity_{0.0};
    double target_theta_{0.0};
    double K_p_{2.0};
    
    double l_{0.0};
    double pose_theta_{0.0};
    
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr cw_pub_;
    rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr pwm_pub_;
    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr pose_sub_;
};

class TurtleController : public rclcpp::Node {
public:
    TurtleController() : Node("teleop_keyboard") {
        // Create publishers first
        midpoint_pub_ = create_publisher<turtlesim::msg::Pose>("/mid_point/pose", 10);
        desired_pub_ = create_publisher<geometry_msgs::msg::Twist>("/desired/cmd_vel", 10);
        
        // Initialize control state
        control_first_ = true;
        control_second_ = false;
        control_all_ = false;
        control_linear_velocity_ = 0.0;
        control_angular_velocity_ = 0.0;
        theta_steer_ = 0.0;
        base_angle_ = 0.0;  // Initialize base_angle
        
        // Motor control
        height_count_ = 0;
        height_change_running_ = false;
        motor_status_ = "stop";
        pwm_ = 0;
        cw_flag_ = true;
        
        // Timing control
        last_update_time_ = now();
        update_interval_ = std::chrono::milliseconds(10);  // 100Hz
        
        // Create timer for periodic updates
        timer_ = create_wall_timer(
            update_interval_,
            std::bind(&TurtleController::updateTurtles, this));

        // Initialize turtles
        turtles_.push_back(std::make_unique<Turtle>(this, "turtle1", true));
        turtles_.push_back(std::make_unique<Turtle>(this, "turtle2", true));
        turtles_.push_back(std::make_unique<Turtle>(this, "turtle3"));
    }

    void processKey(char c) {
        switch(c) {
            case KEYCODE_W:
                control_linear_velocity_ = checkLinearLimitVelocity(
                    control_linear_velocity_ + LIN_VEL_STEP_SIZE);
                printVels();
                break;
            case KEYCODE_X:
                control_linear_velocity_ = checkLinearLimitVelocity(
                    control_linear_velocity_ - LIN_VEL_STEP_SIZE);
                printVels();
                break;
            case KEYCODE_A:
                control_angular_velocity_ = checkAngularLimitVelocity(
                    control_angular_velocity_ + ANG_VEL_STEP_SIZE);
                printVels();
                break;
            case KEYCODE_D:
                control_angular_velocity_ = checkAngularLimitVelocity(
                    control_angular_velocity_ - ANG_VEL_STEP_SIZE);
                printVels();
                break;
            case KEYCODE_1:
                control_first_ = true;
                control_second_ = false;
                control_all_ = false;
                RCLCPP_INFO(get_logger(), "controlling first robot");
                break;
            case KEYCODE_2:
                control_first_ = false;
                control_second_ = true;
                control_all_ = false;
                RCLCPP_INFO(get_logger(), "controlling second robot");
                break;
            case KEYCODE_3:
                control_first_ = true;
                control_second_ = true;
                control_all_ = true;
                updateMidpoint();
                break;
            case KEYCODE_Q:
                theta_steer_ += 0.01 * M_PI;
                break;
            case KEYCODE_E:
                theta_steer_ -= 0.01 * M_PI;
                break;
            case KEYCODE_SPACE:
            case KEYCODE_S:
                stopAll();
                break;
            case KEYCODE_I:
                handleMotorUp();
                break;
            case KEYCODE_K:
                handleMotorStop();
                break;
            case KEYCODE_M:
                handleMotorDown();
                break;
        }
    }

private:
    void updateTurtles() {
        if (control_all_) {
            updateMidpoint();
            updateAllControl();
        } else {
            updateSingleControl();
        }
    }

    void updateMidpoint() {
        Eigen::Vector2d midpoint = (turtles_[1]->getPosition() + turtles_[0]->getPosition()) / 2.0;
        Eigen::Vector2d diff = turtles_[1]->getPosition() - turtles_[0]->getPosition();
        double l = diff.norm();
        
        // Check if robots are too close to each other
        if (l < 1e-6) {
            RCLCPP_WARN(get_logger(), "Robots are too close to each other, using previous base_angle");
            return;
        }
        
        // Calculate base_angle directly without accumulation
        base_angle_ = std::atan2(diff[1], diff[0]);
        
        // Log the calculated values
        RCLCPP_INFO(get_logger(), "Midpoint: (%.3f, %.3f), Base angle: %.3f", 
            midpoint[0], midpoint[1], base_angle_);
        
        for (auto& turtle : turtles_) {
            Eigen::Vector2d relative_pos = turtle->getPosition() - midpoint;
            double dist = relative_pos.norm();
            
            // Check if turtle is too close to midpoint
            if (dist < 1e-6) {
                RCLCPP_WARN(get_logger(), "Turtle %s is too close to midpoint", turtle->getName().c_str());
                continue;
            }
            
            turtle->setL(dist);
            double pose_theta = std::atan2(relative_pos[1], relative_pos[0]) - base_angle_;
            turtle->setPoseTheta(pose_theta);
            
            // Log the calculated values
            RCLCPP_INFO(get_logger(), "Turtle %s: l=%.3f, pose_theta=%.3f", 
                turtle->getName().c_str(), dist, pose_theta);
        }
    }

    void updateAllControl() {
        double base_angle = std::atan2(
            turtles_[1]->getPosition()[1] - turtles_[0]->getPosition()[1],
            turtles_[1]->getPosition()[0] - turtles_[0]->getPosition()[0]
        );
        
        theta_steer_ = std::fmod(theta_steer_, 2 * M_PI);
        while (theta_steer_ < 0) {
            theta_steer_ += 2 * M_PI;
        }
        
        RCLCPP_INFO(get_logger(), "theta_steer = %f pi", theta_steer_ / M_PI);
        
        calculateTargetVelocities(control_linear_velocity_, control_angular_velocity_, base_angle);
        
        for (auto& turtle : turtles_) {
            turtle->publishVelocity();
        }
        
        publishMidpointAndDesired(control_linear_velocity_, control_angular_velocity_, base_angle);
    }

    void calculateTargetVelocities(double control_linear_velocity, double control_angular_velocity, double base_angle) {
        // Cache frequently used values
        double l = turtles_[0]->getL() + turtles_[1]->getL();
        
        // Handle the case when both velocities are zero
        if (std::abs(control_linear_velocity) < 1e-2 && std::abs(control_angular_velocity) < 1e-2) {
            for (auto& turtle : turtles_) {
                turtle->setTarget(0.0, turtle->getTheta(), 0.0, 0.0);  // Remove base_angle
            }
            return;
        }
        
        if (std::abs(control_angular_velocity) < 1e-2) {
            // Straight line motion
            for (auto& turtle : turtles_) {
                turtle->setTarget(control_linear_velocity, theta_steer_, 0.0, 0.0);  // Remove base_angle
            }
        }
        else if (std::abs(control_linear_velocity) < 1e-2 && std::abs(control_angular_velocity) > 1e-2) {
            // Pure rotation
            if (control_angular_velocity > 0) {
                turtles_[0]->setTarget(turtles_[0]->getL() * control_angular_velocity, M_PI/2, 0.0, control_angular_velocity);  // Remove base_angle
                turtles_[1]->setTarget(turtles_[1]->getL() * control_angular_velocity, 3*M_PI/2, 0.0, control_angular_velocity);  // Remove base_angle
                for (size_t i = 2; i < turtles_.size(); ++i) {
                    turtles_[i]->setTarget(
                        turtles_[i]->getL() * control_angular_velocity,
                        turtles_[i]->getPoseTheta() - M_PI/2,
                        0.0,  // Remove base_angle
                        control_angular_velocity
                    );
                }
            } else {
                turtles_[0]->setTarget(-turtles_[0]->getL() * control_angular_velocity, 3*M_PI/2, 0.0, control_angular_velocity);  // Remove base_angle
                turtles_[1]->setTarget(-turtles_[1]->getL() * control_angular_velocity, M_PI/2, 0.0, control_angular_velocity);  // Remove base_angle
                for (size_t i = 2; i < turtles_.size(); ++i) {
                    turtles_[i]->setTarget(
                        -turtles_[i]->getL() * control_angular_velocity,
                        turtles_[i]->getPoseTheta() + M_PI/2,
                        0.0,  // Remove base_angle
                        control_angular_velocity
                    );
                }
            }
        }
        else {
            RCLCPP_INFO(get_logger(), "control_angular_velocity = %f", control_angular_velocity);
            
            // Calculate rho and check for potential division by zero
            double rho;
            if (std::abs(control_angular_velocity) < 1e-2) {
                rho = std::numeric_limits<double>::infinity();
            } else {
                rho = control_linear_velocity / control_angular_velocity;
            }
            
            if ((control_angular_velocity < 0 && ((theta_steer_ >= 0 && theta_steer_ < M_PI/2) || (theta_steer_ <= 2*M_PI && theta_steer_ > 3*M_PI/2)) && control_linear_velocity > 0) ||
                (control_angular_velocity > 0 && ((theta_steer_ >= 0 && theta_steer_ < M_PI/2) || (theta_steer_ <= 2*M_PI && theta_steer_ > 3*M_PI/2)) && control_linear_velocity < 0)) {
                RCLCPP_INFO(get_logger(), "R_D");
                double psi = theta_steer_ - M_PI/2;
                double x = rho * std::cos(psi);
                double y = rho * std::sin(psi);
                
                // Check for infinite values
                if (std::isinf(rho)) {
                    for (auto& turtle : turtles_) {
                        turtle->setTarget(0.0, turtle->getTheta(), 0.0, 0.0);  // Remove base_angle
                    }
                    return;
                }
                
                double target_theta1 = std::atan2(y, x - turtles_[0]->getL()) + M_PI/2;
                double target_theta2 = std::atan2(y, x + turtles_[1]->getL()) + 3*M_PI/2;
                
                // Check for potential division by zero in sin calculation
                double sin_diff = std::sin(target_theta2 - target_theta1);
                if (std::abs(sin_diff) < 1e-6) {
                    for (auto& turtle : turtles_) {
                        turtle->setTarget(0.0, turtle->getTheta(), 0.0, 0.0);  // Remove base_angle
                    }
                    return;
                }
                
                double target_vel1 = -control_angular_velocity * l / sin_diff * std::cos(target_theta2);
                double target_vel2 = control_angular_velocity * l / sin_diff * std::cos(target_theta1);
                target_theta2 += M_PI;
                
                turtles_[0]->setTarget(target_vel1, target_theta1, 0.0, control_angular_velocity);  // Remove base_angle
                turtles_[1]->setTarget(target_vel2, target_theta2, 0.0, control_angular_velocity);  // Remove base_angle
                
                for (size_t i = 2; i < turtles_.size(); ++i) {
                    Eigen::Vector2d pos(
                        turtles_[i]->getL() * std::cos(turtles_[i]->getPoseTheta()) + x,
                        turtles_[i]->getL() * std::sin(turtles_[i]->getPoseTheta()) + y
                    );
                    double ri = pos.norm();
                    double target_thetai = std::atan2(
                        turtles_[i]->getL() * std::sin(turtles_[i]->getPoseTheta()) + y,
                        turtles_[i]->getL() * std::cos(turtles_[i]->getPoseTheta()) + x
                    ) + M_PI/2;
                    double target_veli = -ri * control_angular_velocity;
                    turtles_[i]->setTarget(target_veli, target_thetai, 0.0, control_angular_velocity);  // Remove base_angle
                }
            }
            else if ((control_angular_velocity > 0 && (theta_steer_ > M_PI/2 && theta_steer_ < 3*M_PI/2) && control_linear_velocity > 0) ||
                     (control_angular_velocity < 0 && (theta_steer_ > M_PI/2 && theta_steer_ < 3*M_PI/2) && control_linear_velocity < 0)) {
                RCLCPP_INFO(get_logger(), "L_D");
                double rho = control_linear_velocity / control_angular_velocity;
                double psi = theta_steer_ + M_PI/2;
                double x = rho * std::cos(psi);
                double y = rho * std::sin(psi);
                
                double target_theta1 = std::atan2(y, x - turtles_[0]->getL()) - M_PI/2;
                double target_theta2 = std::atan2(y, x + turtles_[1]->getL()) - M_PI/2;
                double target_vel1 = -control_angular_velocity * l / std::sin(target_theta2 - target_theta1) * std::cos(target_theta2);
                double target_vel2 = -control_angular_velocity * l / std::sin(target_theta2 - target_theta1) * std::cos(target_theta1);
                
                turtles_[0]->setTarget(target_vel1, target_theta1, 0.0, control_angular_velocity);  // Remove base_angle
                turtles_[1]->setTarget(target_vel2, target_theta2, 0.0, control_angular_velocity);  // Remove base_angle
                
                for (size_t i = 2; i < turtles_.size(); ++i) {
                    Eigen::Vector2d pos(
                        turtles_[i]->getL() * std::cos(turtles_[i]->getPoseTheta()) + x,
                        turtles_[i]->getL() * std::sin(turtles_[i]->getPoseTheta()) + y
                    );
                    double ri = pos.norm();
                    double target_thetai = std::atan2(
                        turtles_[i]->getL() * std::sin(turtles_[i]->getPoseTheta()) + y,
                        turtles_[i]->getL() * std::cos(turtles_[i]->getPoseTheta()) + x
                    ) - M_PI/2;
                    double target_veli = ri * control_angular_velocity;
                    turtles_[i]->setTarget(target_veli, target_thetai, 0.0, control_angular_velocity);  // Remove base_angle
                }
            }
            else if ((control_angular_velocity > 0 && ((theta_steer_ >= 0 && theta_steer_ < M_PI/2) || (theta_steer_ <= 2*M_PI && theta_steer_ > 3*M_PI/2)) && control_linear_velocity > 0) ||
                     (control_angular_velocity < 0 && ((theta_steer_ >= 0 && theta_steer_ < M_PI/2) || (theta_steer_ <= 2*M_PI && theta_steer_ > 3*M_PI/2)) && control_linear_velocity < 0)) {
                RCLCPP_INFO(get_logger(), "L_U");
                double rho = control_linear_velocity / control_angular_velocity;
                double psi = theta_steer_ + M_PI/2;
                double x = rho * std::cos(psi);
                double y = rho * std::sin(psi);
                
                double target_theta1 = std::atan2(y, x - turtles_[0]->getL()) - M_PI/2;
                double target_theta2 = std::atan2(y, x + turtles_[1]->getL()) - M_PI/2;
                double target_vel1 = -control_angular_velocity * l / std::sin(target_theta2 - target_theta1) * std::cos(target_theta2);
                double target_vel2 = -control_angular_velocity * l / std::sin(target_theta2 - target_theta1) * std::cos(target_theta1);
                
                turtles_[0]->setTarget(target_vel1, target_theta1, 0.0, control_angular_velocity);  // Remove base_angle
                turtles_[1]->setTarget(target_vel2, target_theta2, 0.0, control_angular_velocity);  // Remove base_angle
                
                for (size_t i = 2; i < turtles_.size(); ++i) {
                    Eigen::Vector2d pos(
                        turtles_[i]->getL() * std::cos(turtles_[i]->getPoseTheta()) + x,
                        turtles_[i]->getL() * std::sin(turtles_[i]->getPoseTheta()) + y
                    );
                    double ri = pos.norm();
                    double target_thetai = std::atan2(
                        turtles_[i]->getL() * std::sin(turtles_[i]->getPoseTheta()) + y,
                        turtles_[i]->getL() * std::cos(turtles_[i]->getPoseTheta()) + x
                    ) - M_PI/2;
                    double target_veli = ri * control_angular_velocity;
                    turtles_[i]->setTarget(target_veli, target_thetai, 0.0, control_angular_velocity);  // Remove base_angle
                }
            }
            else if ((control_angular_velocity > 0 && ((theta_steer_ > M_PI/2 && theta_steer_ < 3*M_PI/2) || (theta_steer_ > 3*M_PI/2 && theta_steer_ < 2*M_PI)) && control_linear_velocity < 0) ||
                     (control_angular_velocity < 0 && ((theta_steer_ > M_PI/2 && theta_steer_ < 3*M_PI/2) || (theta_steer_ > 3*M_PI/2 && theta_steer_ < 2*M_PI)) && control_linear_velocity > 0)) {
                RCLCPP_INFO(get_logger(), "R_U");
                double rho = -control_linear_velocity / control_angular_velocity;
                double psi = theta_steer_ - M_PI/2;
                double x = rho * std::cos(psi);
                double y = rho * std::sin(psi);
                
                double target_theta1 = std::atan2(y, x - turtles_[0]->getL()) + M_PI/2;
                double target_theta2 = std::atan2(y, x + turtles_[1]->getL()) + M_PI/2;
                double target_vel1 = -control_angular_velocity * l / std::sin(target_theta2 - target_theta1) * std::cos(target_theta2);
                double target_vel2 = -control_angular_velocity * l / std::sin(target_theta2 - target_theta1) * std::cos(target_theta1);
                
                turtles_[0]->setTarget(target_vel1, target_theta1, 0.0, control_angular_velocity);  // Remove base_angle
                turtles_[1]->setTarget(target_vel2, target_theta2, 0.0, control_angular_velocity);  // Remove base_angle
                
                for (size_t i = 2; i < turtles_.size(); ++i) {
                    Eigen::Vector2d pos(
                        turtles_[i]->getL() * std::cos(turtles_[i]->getPoseTheta()) + x,
                        turtles_[i]->getL() * std::sin(turtles_[i]->getPoseTheta()) + y
                    );
                    double ri = pos.norm();
                    double target_thetai = std::atan2(
                        turtles_[i]->getL() * std::sin(turtles_[i]->getPoseTheta()) + y,
                        turtles_[i]->getL() * std::cos(turtles_[i]->getPoseTheta()) + x
                    ) + M_PI/2;
                    double target_veli = -ri * control_angular_velocity;
                    turtles_[i]->setTarget(target_veli, target_thetai, 0.0, control_angular_velocity);  // Remove base_angle
                }
            }
        }
    }

    void updateSingleControl() {
        if (control_first_) {
            double theta = turtles_[0]->getTheta();
            if (std::isnan(theta)) {
                theta = 0.0;  // Use default value if theta is NaN
            }
            turtles_[0]->setTarget(control_linear_velocity_, theta, 0.0, control_angular_velocity_);
            turtles_[0]->publishVelocity();
            turtles_[0]->publishMotorControl(cw_flag_, pwm_);
        }
        if (control_second_) {
            double theta = turtles_[1]->getTheta();
            if (std::isnan(theta)) {
                theta = 0.0;  // Use default value if theta is NaN
            }
            turtles_[1]->setTarget(control_linear_velocity_, theta, 0.0, control_angular_velocity_);
            turtles_[1]->publishVelocity();
            turtles_[1]->publishMotorControl(cw_flag_, pwm_);
        }
    }

    void publishMidpointAndDesired(double linear_vel, double angular_vel, double base_angle) {
        // Publish midpoint pose
        auto midpoint_msg = std::make_unique<turtlesim::msg::Pose>();
        Eigen::Vector2d midpoint = (turtles_[1]->getPosition() + turtles_[0]->getPosition()) / 2.0;
        midpoint_msg->x = midpoint[0];
        midpoint_msg->y = midpoint[1];
        midpoint_msg->theta = base_angle;
        midpoint_pub_->publish(std::move(midpoint_msg));

        // Publish desired velocity
        auto desired_msg = std::make_unique<geometry_msgs::msg::Twist>();
        desired_msg->linear.x = linear_vel;
        desired_msg->angular.z = angular_vel;
        desired_pub_->publish(std::move(desired_msg));
    }

    void stopAll() {
        control_linear_velocity_ = 0.0;
        control_angular_velocity_ = 0.0;
        pwm_ = 0;
        motor_status_ = "stop";
        height_change_running_ = false;
        printVels();
    }

    void handleMotorUp() {
        if (height_count_ < 255) {
            cw_flag_ = true;
            pwm_ = 128;
            motor_status_ = "cw";
            height_change_running_ = true;
            // Start height change thread
            height_change_thread_ = std::thread(&TurtleController::changeHeight, this);
            height_change_thread_.detach();
            printVels();
        } else {
            pwm_ = 0;
            motor_status_ = "stop";
            printVels();
            RCLCPP_INFO(get_logger(), "!!!height upper limit reached!!!");
        }
    }

    void handleMotorStop() {
        pwm_ = 0;
        motor_status_ = "stop";
        height_change_running_ = false;
        printVels();
    }

    void handleMotorDown() {
        if (height_count_ > 0) {
            cw_flag_ = false;
            pwm_ = 128;
            motor_status_ = "ccw";
            height_change_running_ = true;
            // Start height change thread
            height_change_thread_ = std::thread(&TurtleController::changeHeight, this);
            height_change_thread_.detach();
            printVels();
        } else {
            pwm_ = 0;
            motor_status_ = "stop";
            printVels();
            RCLCPP_INFO(get_logger(), "!!!height lower limit reached!!!");
        }
    }

    void changeHeight() {
        while (height_change_running_) {
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
            if (motor_status_ == "cw" && height_count_ < 255) {
                height_count_++;
            } else if (motor_status_ == "ccw" && height_count_ > 0) {
                height_count_--;
            } else if (height_count_ >= 255 || height_count_ <= 0) {
                height_change_running_ = false;
                pwm_ = 0;
            }
            
            // Print progress bar
            int bar_length = 50;
            int filled_length = static_cast<int>(bar_length * height_count_ / 255);
            std::string bar = std::string(filled_length, '#') + std::string(bar_length - filled_length, '-');
            RCLCPP_INFO(get_logger(), "\rHeight Count: [%s] %d/255", bar.c_str(), height_count_);
        }
    }

    void printVels() {
        RCLCPP_INFO(get_logger(), "\ncurrently:\tlinear velocity %f\t angular velocity %f\t motor command %s",
            control_linear_velocity_,
            control_angular_velocity_,
            motor_status_.c_str());
    }

    static double checkLinearLimitVelocity(double velocity) {
        if (std::getenv("TURTLEBOT3_MODEL") == std::string("burger")) {
            return constrain(velocity, -BURGER_MAX_LIN_VEL, BURGER_MAX_LIN_VEL);
        } else {
            return constrain(velocity, -WAFFLE_MAX_LIN_VEL, WAFFLE_MAX_LIN_VEL);
        }
    }

    static double checkAngularLimitVelocity(double velocity) {
        if (std::getenv("TURTLEBOT3_MODEL") == std::string("burger")) {
            return constrain(velocity, -BURGER_MAX_ANG_VEL, BURGER_MAX_ANG_VEL);
        } else {
            return constrain(velocity, -WAFFLE_MAX_ANG_VEL, WAFFLE_MAX_ANG_VEL);
        }
    }

    static double constrain(double input_vel, double low_bound, double high_bound) {
        if (input_vel < low_bound) {
            return low_bound;
        } else if (input_vel > high_bound) {
            return high_bound;
        }
        return input_vel;
    }

    std::vector<std::unique_ptr<Turtle>> turtles_;
    rclcpp::Publisher<turtlesim::msg::Pose>::SharedPtr midpoint_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr desired_pub_;
    
    bool control_first_{true};
    bool control_second_{false};
    bool control_all_{false};
    double control_linear_velocity_{0.0};
    double control_angular_velocity_{0.0};
    double theta_steer_{0.0};
    double base_angle_{0.0};  // Add base_angle as member variable
    
    int height_count_{0};
    bool height_change_running_{false};
    std::string motor_status_{"stop"};
    int pwm_{0};
    bool cw_flag_{true};
    std::thread height_change_thread_;
    
    rclcpp::Time last_update_time_;
    std::chrono::milliseconds update_interval_;
    rclcpp::TimerBase::SharedPtr timer_;

    static constexpr double BURGER_MAX_LIN_VEL = 100.0;
    static constexpr double BURGER_MAX_ANG_VEL = 100.0;
    static constexpr double WAFFLE_MAX_LIN_VEL = 100.0;
    static constexpr double WAFFLE_MAX_ANG_VEL = 100.0;
    static constexpr double LIN_VEL_STEP_SIZE = 0.02;
    static constexpr double ANG_VEL_STEP_SIZE = 0.02;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TurtleController>();
    
    KeyboardReader input;
    char c;
    
    RCLCPP_INFO(node->get_logger(), "Control Your TurtleBot3!");
    RCLCPP_INFO(node->get_logger(), "---------------------------");
    RCLCPP_INFO(node->get_logger(), "Moving around:                     Motors:");
    RCLCPP_INFO(node->get_logger(), "        w                               i");
    RCLCPP_INFO(node->get_logger(), "   a    s    d                          k");
    RCLCPP_INFO(node->get_logger(), "        x                               m");
    RCLCPP_INFO(node->get_logger(), "w/x : increase/decrease linear velocity");
    RCLCPP_INFO(node->get_logger(), "a/d : increase/decrease angular velocity");
    RCLCPP_INFO(node->get_logger(), "i/k/m : move lift upwards/stop/downwards");
    RCLCPP_INFO(node->get_logger(), "1/2/3 : controlling 1st robot, 2nd robot, all");
    RCLCPP_INFO(node->get_logger(), "space key, s : force stop");
    RCLCPP_INFO(node->get_logger(), "CTRL-C to quit");

    // Start a separate thread for ROS2 spinning
    std::atomic<bool> running(true);
    std::thread spin_thread([&node, &running]() {
        while (running && rclcpp::ok()) {
            rclcpp::spin_some(node);
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    });

    try {
        while (rclcpp::ok()) {
            // Check for keyboard input
            if (input.readOne(&c)) {
                node->processKey(c);
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node->get_logger(), "Exception: %s", e.what());
    }

    // Cleanup
    running = false;
    if (spin_thread.joinable()) {
        spin_thread.join();
    }
    input.shutdown();
    rclcpp::shutdown();
    return 0;
} 