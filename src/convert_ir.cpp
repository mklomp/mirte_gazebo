#include "std_msgs/msg/string.hpp"
#include <iostream>
// #include <mirte_msgs/msg/
#include <mirte_msgs/msg/intensity.hpp>
#include <mirte_msgs/msg/intensity_digital.hpp>
#include <mirte_msgs/srv/get_intensity.hpp>
#include <mirte_msgs/srv/get_intensity_digital.hpp>
// #include <ros/ros.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_srvs/srv/empty.h>
using std::placeholders::_1;
using std::placeholders::_2;

class map_ir : public rclcpp::Node {

public:
  map_ir(std::string node_name, std::string side) : Node(node_name) {

    pub_ = this->create_publisher<mirte_msgs::msg::Intensity>(
        "/mirte/intensity/" + side, 10);
    pub_dig_ = this->create_publisher<mirte_msgs::msg::IntensityDigital>(
        "/mirte/intensity/" + side + "_digital", 10);
    sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        "/mirte/camera_ir_" + side + "/image_raw", 10,
        std::bind(&map_ir::callback, this, _1));
    server_ = this->create_service<mirte_msgs::srv::GetIntensity>(
        "/mirte/get_intensity_" + side,
        std::bind(&map_ir::service_cb, this, _1, _2));
    server_dig_ = this->create_service<mirte_msgs::srv::GetIntensityDigital>(
        "/mirte/get_intensity_" + side + "_digital",
        std::bind(&map_ir::service_cb_dig, this, _1, _2));
  }

  void callback(const sensor_msgs::msg::Image::SharedPtr img) {
    mirte_msgs::msg::Intensity irInt;
    irInt.value = img->data[0];
    irInt.header = img->header;
    this->lastIntensity = irInt.value;
    pub_->publish(irInt);

    mirte_msgs::msg::IntensityDigital irIntDig;
    irIntDig.value = img->data[0] > 128;
    irIntDig.header = img->header;
    this->lastIntensityDig = irIntDig.value;
    pub_dig_->publish(irIntDig);
  }

  bool service_cb(mirte_msgs::srv::GetIntensity::Request::SharedPtr /*req*/,
                  mirte_msgs::srv::GetIntensity::Response::SharedPtr res) {

    res->data = this->lastIntensity;
    return true;
  }
  bool service_cb_dig(
      mirte_msgs::srv::GetIntensityDigital::Request::SharedPtr /*req*/,
      mirte_msgs::srv::GetIntensityDigital::Response::SharedPtr res) {
    res->data = this->lastIntensityDig;
    return true;
  }

private:
  // Create the necessary objects for subscribing and publishing
  // rclcpp::Node n_;
  rclcpp::Publisher<mirte_msgs::msg::Intensity>::SharedPtr pub_;
  rclcpp::Publisher<mirte_msgs::msg::IntensityDigital>::SharedPtr pub_dig_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;
  rclcpp::Service<mirte_msgs::srv::GetIntensity>::SharedPtr server_;
  rclcpp::Service<mirte_msgs::srv::GetIntensityDigital>::SharedPtr server_dig_;

  int lastIntensity = 0;
  bool lastIntensityDig = false;
}; // End of class SubscribeAndPublish

int main(int argc, char **argv) {
  std::string side = argv[1];
  std::string name = "map_ir_" + side;
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<map_ir>(name, side));
  rclcpp::shutdown();

  return 0;
}
