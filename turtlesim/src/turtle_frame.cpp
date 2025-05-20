/*
 * Copyright (c) 2009, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include "turtlesim/turtle_frame.h"

#include <QPointF>
#include <cmath>
#include <cstdlib>
#include <ctime>
#include <fstream>
#include <yaml-cpp/yaml.h>

#define DEFAULT_BG_R 0x80
#define DEFAULT_BG_G 0x80
#define DEFAULT_BG_B 0x80

namespace turtlesim
{

TurtleFrame::TurtleFrame(rclcpp::Node::SharedPtr& node_handle, QWidget* parent, Qt::WindowFlags f)
: QFrame(parent, f)
, path_image_(1600, 1200, QImage::Format_ARGB32)
, path_painter_(&path_image_)
, frame_count_(0)
, id_counter_(0)
, rect_initialized_(false)
, total_rotation_(0.0)
{
  setFixedSize(1600, 1200);
  setWindowTitle("TurtleSim");

  srand(time(NULL));
  update_timer_ = new QTimer(this);
  update_timer_->setInterval(16);
  update_timer_->start();

  connect(update_timer_, SIGNAL(timeout()), this, SLOT(onUpdate()));

  nh_ = node_handle;
  rcl_interfaces::msg::IntegerRange range;
  range.from_value = 0;
  range.step = 1;
  range.to_value = 255;
  rcl_interfaces::msg::ParameterDescriptor background_r_descriptor;
  background_r_descriptor.description = "Red channel of the background color";
  background_r_descriptor.integer_range.push_back(range);
  rcl_interfaces::msg::ParameterDescriptor background_g_descriptor;
  background_g_descriptor.description = "Green channel of the background color";
  background_g_descriptor.integer_range.push_back(range);
  rcl_interfaces::msg::ParameterDescriptor background_b_descriptor;
  background_b_descriptor.description = "Blue channel of the background color";
  background_b_descriptor.integer_range.push_back(range);
  nh_->declare_parameter("background_r", rclcpp::ParameterValue(DEFAULT_BG_R), background_r_descriptor);
  nh_->declare_parameter("background_g", rclcpp::ParameterValue(DEFAULT_BG_G), background_g_descriptor);
  nh_->declare_parameter("background_b", rclcpp::ParameterValue(DEFAULT_BG_B), background_b_descriptor);

  QVector<QString> turtles;
  turtles.append("ardent.png");
  // turtles.append("bouncy.png");
  // turtles.append("crystal.png");
  // turtles.append("dashing.png");
  // turtles.append("eloquent.png");
  // turtles.append("foxy.png");

  QString images_path = (ament_index_cpp::get_package_share_directory("turtlesim") + "/images/").c_str();
  for (int i = 0; i < turtles.size(); ++i)
  {
    QImage img;
    img.load(images_path + turtles[i]);
    turtle_images_.append(img);
  }

  meter_ = turtle_images_[0].height(); //45

  clear();

  clear_srv_ = nh_->create_service<std_srvs::srv::Empty>("clear", std::bind(&TurtleFrame::clearCallback, this, std::placeholders::_1, std::placeholders::_2));
  reset_srv_ = nh_->create_service<std_srvs::srv::Empty>("reset", std::bind(&TurtleFrame::resetCallback, this, std::placeholders::_1, std::placeholders::_2));
  spawn_srv_ = nh_->create_service<turtlesim::srv::Spawn>("spawn", std::bind(&TurtleFrame::spawnCallback, this, std::placeholders::_1, std::placeholders::_2));
  kill_srv_ = nh_->create_service<turtlesim::srv::Kill>("kill", std::bind(&TurtleFrame::killCallback, this, std::placeholders::_1, std::placeholders::_2));

  rclcpp::QoS qos(rclcpp::KeepLast(100), rmw_qos_profile_sensor_data);
  parameter_event_sub_ = nh_->create_subscription<rcl_interfaces::msg::ParameterEvent>(
    "/parameter_events", qos, std::bind(&TurtleFrame::parameterEventCallback, this, std::placeholders::_1));

  RCLCPP_INFO(nh_->get_logger(), "Starting turtlesim with node name %s", nh_->get_node_names()[0].c_str());

  width_in_meters_ = (width() - 1) / meter_;
  height_in_meters_ = (height() - 1) / meter_;
  loadTurtles();

  // spawn all available turtle types
  if(false)
  {
    for(int index = 0; index < turtles.size(); ++index)
    {
      QString name = turtles[index];
      name = name.split(".").first();
      name.replace(QString("-"), QString(""));
      spawnTurtle(name.toStdString(), 1.0f + 1.5f * (index % 7), 1.0f + 1.5f * (index / 7), static_cast<float>(PI) / 2.0f, index);
    }
  }
}

TurtleFrame::~TurtleFrame()
{
  delete update_timer_;
}

bool TurtleFrame::spawnCallback(const turtlesim::srv::Spawn::Request::SharedPtr req, turtlesim::srv::Spawn::Response::SharedPtr res)
{
  std::string name = spawnTurtle(req->name, req->x, req->y, req->theta);
  if (name.empty())
  {
    RCLCPP_ERROR(nh_->get_logger(), "A turtle named [%s] already exists", req->name.c_str());
    return false;
  }

  res->name = name;

  return true;
}

bool TurtleFrame::killCallback(const turtlesim::srv::Kill::Request::SharedPtr req, turtlesim::srv::Kill::Response::SharedPtr)
{
  M_Turtle::iterator it = turtles_.find(req->name);
  if (it == turtles_.end())
  {
    RCLCPP_ERROR(nh_->get_logger(), "Tried to kill turtle [%s], which does not exist", req->name.c_str());
    return false;
  }

  turtles_.erase(it);
  update();

  return true;
}

void TurtleFrame::parameterEventCallback(const rcl_interfaces::msg::ParameterEvent::SharedPtr event)
{
  // only consider events from this node
  if (event->node == nh_->get_fully_qualified_name())
  {
    // since parameter events for this even aren't expected frequently just always call update()
    update();
  }
}

bool TurtleFrame::hasTurtle(const std::string& name)
{
  return turtles_.find(name) != turtles_.end();
}

std::string TurtleFrame::spawnTurtle(const std::string& name, float x, float y, float midtheta)
{
  return spawnTurtle(name, x, y, midtheta, rand() % turtle_images_.size());
}

std::string TurtleFrame::spawnTurtle(const std::string& name, float x, float y, float midtheta, size_t index)
{
  std::string real_name = name;
  if (real_name.empty())
  {
    do
    {
      std::stringstream ss;
      ss << "turtle" << ++id_counter_;
      real_name = ss.str();
    } while (hasTurtle(real_name));
  }
  else
  {
    if (hasTurtle(real_name))
    {
      return "";
    }
  }

  TurtlePtr t = std::make_shared<Turtle>(nh_, real_name, turtle_images_[static_cast<int>(index)], QPointF(x, height_in_meters_ - y), midtheta);
  turtles_[real_name] = t;
  update();

  RCLCPP_INFO(nh_->get_logger(), "Spawning turtle [%s] at x=[%f], y=[%f], theta=[%f]", real_name.c_str(), x, y, midtheta);

  return real_name;
}

void TurtleFrame::clear()
{
  // make all pixels fully transparent
  path_image_.fill(qRgba(255, 255, 255, 0));
  update();
}

void TurtleFrame::onUpdate()
{
  if (!rclcpp::ok())
  {
    close();
    return;
  }

  rclcpp::spin_some(nh_);

  updateTurtles();
}

void TurtleFrame::paintEvent(QPaintEvent*)
{
  QPainter painter(this);
  height_in_meters_ = (height() - 1) / meter_;
  int r = DEFAULT_BG_R;
  int g = DEFAULT_BG_G;
  int b = DEFAULT_BG_B;
  nh_->get_parameter("background_r", r);
  nh_->get_parameter("background_g", g);
  nh_->get_parameter("background_b", b);
  QRgb background_color = qRgb(r, g, b);
  painter.fillRect(0, 0, width(), height(), background_color);

  painter.drawImage(QPoint(0, 0), path_image_);

  M_Turtle::iterator it = turtles_.begin();
  M_Turtle::iterator end = turtles_.end();
  for (; it != end; ++it)
  {
    it->second->paint(painter);
  }

  if (turtles_.size() >= 2) {
    //while displaying, remind that the axes are opposite to we think.
      auto it1 = turtles_.begin();
      auto it2 = std::next(it1);

      QPointF position1 = it1->second->getposition(); // 첫 번째 거북이 위치
      QPointF position2 = it2->second->getposition(); // 두 번째 거북이 위치

      painter.setPen(Qt::red);
      painter.drawLine(position1, position2);

      float midtheta = it1->second->getTheta();
      float midvel = it1->second->getLinVel();
      float midangvel = it1->second->getAngVel();
      float base_angle = atan2(position2.y() - position1.y(), position2.x() - position1.x());
      
      midtheta = -midtheta;
      midtheta += base_angle;
      
      // Calculate center position
      float centerX = (position1.x() + position2.x()) / 2.0;
      float centerY = (position1.y() + position2.y()) / 2.0;
      QPointF red_dot(centerX * meter_, centerY * meter_);


      // //Calculate the center of object rotation
      // enum CENTER_OF_ROTATION_CASE{
      //   THETA_NEAR_PI_2,
      //   THETA_NEAR_3PI_2,
      //   LEFT_HAND_ROTATION,
      //   RIGHT_HAND_ROTATION
      // };
      // CENTER_OF_ROTATION_CASE center_of_rotation_case;


      // if(abs(midangvel) > 0.0001)
      // {
      // }
      
      red_dots_.append(red_dot);
      painter.setPen(Qt::red);
      painter.setBrush(Qt::red);
      painter.drawEllipse(red_dot, 2, 2);

      // Initialize rectangle if not done yet
      if (!rect_initialized_ && turtles_.size() >= 2) {
          float min_x = std::numeric_limits<float>::max();
          float min_y = std::numeric_limits<float>::max();
          float max_x = std::numeric_limits<float>::lowest();
          float max_y = std::numeric_limits<float>::lowest();

          for (const auto& turtle : turtles_) {
              QPointF pos = turtle.second->getposition();
              min_x = std::min(min_x, static_cast<float>(pos.x()));
              min_y = std::min(min_y, static_cast<float>(pos.y()));
              max_x = std::max(max_x, static_cast<float>(pos.x()));
              max_y = std::max(max_y, static_cast<float>(pos.y()));
          }

          // Add some padding
          float padding = 0.1;
          initial_rect_ = QRectF(
              (min_x - padding) * meter_,
              (min_y - padding) * meter_,
              (max_x - min_x + 2 * padding) * meter_,
              (max_y - min_y + 2 * padding) * meter_
          );
          current_rect_ = initial_rect_;
          rect_initialized_ = true;
      }

      // Update rectangle position based on midpoint movement
      if (rect_initialized_) {
          
          // Calculate time step
          float dt = 0.001 * update_timer_->interval();
          
          // Calculate new position based on linear velocity
          float dx = midvel * cos(midtheta) * dt;
          float dy = midvel * sin(midtheta) * dt;
          
          // Move rectangle
          current_rect_.translate(-dx * meter_, -dy * meter_);
          
          // Update total rotation
          total_rotation_ += midangvel * dt * 180.0 / PI;  // Convert to degrees
          QPointF center = current_rect_.center();
          
          // Create transformation matrix
          QTransform transform;
          transform.translate(center.x(), center.y());
          transform.rotate(-total_rotation_);
          transform.translate(-center.x(), -center.y());
          
          // Apply transformation to rectangle corners
          QPolygonF rotated_rect;
          rotated_rect << transform.map(current_rect_.topLeft())
                      << transform.map(current_rect_.topRight())
                      << transform.map(current_rect_.bottomRight())
                      << transform.map(current_rect_.bottomLeft());
          
          // Draw rotated rectangle
          painter.setPen(QPen(Qt::blue, 2));
          painter.setBrush(Qt::NoBrush);
          painter.drawPolygon(rotated_rect);
      }

      QPointF arrow_end(red_dot.x() - 100 * midvel * cos(midtheta), red_dot.y() - 100 * midvel * sin(midtheta));
      painter.setPen(Qt::green);
      painter.drawLine(red_dot, arrow_end);

      QPolygonF arrow_head;
      float arrow_size = 10.0;
      if (midvel >= 0) {
          QPointF point1 = arrow_end + QPointF(arrow_size * cos(midtheta + M_PI / 6), arrow_size * sin(midtheta + M_PI / 6));
          QPointF point2 = arrow_end + QPointF(arrow_size * cos(midtheta - M_PI / 6), arrow_size * sin(midtheta - M_PI / 6));
          arrow_head << arrow_end << point1 << point2;
      } else {
          QPointF point1 = arrow_end - QPointF(arrow_size * cos(midtheta + M_PI / 6), arrow_size * sin(midtheta + M_PI / 6));
          QPointF point2 = arrow_end - QPointF(arrow_size * cos(midtheta - M_PI / 6), arrow_size * sin(midtheta - M_PI / 6));
          arrow_head << arrow_end << point1 << point2;
      }

      painter.setPen(Qt::green);
      painter.setBrush(Qt::green);
      painter.drawPolygon(arrow_head);
  }
  
  painter.setPen(Qt::red);
  painter.setBrush(Qt::red);
  for (const auto& dot : red_dots_) {
      painter.drawEllipse(dot, 1, 1); // 기존 점들 그리기
  }
}

void TurtleFrame::updateTurtles()
{
  if (last_turtle_update_.nanoseconds() == 0)
  {
    last_turtle_update_ = nh_->now();
    return;
  }

  bool modified = false;
  M_Turtle::iterator it = turtles_.begin();
  M_Turtle::iterator end = turtles_.end();
  for (; it != end; ++it)
  {
    modified |= it->second->update(0.001 * update_timer_->interval(), path_painter_, path_image_, width_in_meters_, height_in_meters_);
  }
  if (modified)
  {
    update();
  }

  ++frame_count_;
}


bool TurtleFrame::clearCallback(const std_srvs::srv::Empty::Request::SharedPtr, std_srvs::srv::Empty::Response::SharedPtr)
{
  RCLCPP_INFO(nh_->get_logger(), "Clearing turtlesim.");
  clear();
  return true;
}

bool TurtleFrame::resetCallback(const std_srvs::srv::Empty::Request::SharedPtr, std_srvs::srv::Empty::Response::SharedPtr)
{
  RCLCPP_INFO(nh_->get_logger(), "Resetting turtlesim.");
  turtles_.clear();
  id_counter_ = 0;
  loadTurtles();
  clear();
  return true;
}

void TurtleFrame::loadTurtles()
{
  try {
    // Get the path to the config file from the installed location
    std::string config_path = ament_index_cpp::get_package_share_directory("turtlesim") + "/config/config.yaml";
    RCLCPP_INFO(nh_->get_logger(), "Loading config from: %s", config_path.c_str());
    YAML::Node config = YAML::LoadFile(config_path);
    const YAML::Node& turtles = config["turtles"];

    for (const auto& turtle : turtles) {
      const std::string& name = turtle.first.as<std::string>();
      const YAML::Node& pos = turtle.second;
      
      double x = pos["x"].as<double>();
      double y = pos["y"].as<double>();
      double theta = pos["theta"].as<double>();
      
      // Convert pixel coordinates to meters
      x = x / meter_;
      y = y / meter_;
      
      spawnTurtle(name, x, y, theta);
    }
  } catch (const YAML::Exception& e) {
    RCLCPP_ERROR(nh_->get_logger(), "Error loading config file: %s", e.what());
    // Fallback to default positions if config file fails to load
    spawnTurtle("", width_in_meters_ / 4.0, height_in_meters_ / 2.0, 0);
    spawnTurtle("", width_in_meters_ / 20.0, height_in_meters_ / 2.0, 0);
    spawnTurtle("", width_in_meters_ / 6.667, height_in_meters_ / 2.5, 0);
  }
}

}
