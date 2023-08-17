#include "std_msgs/String.h"
#include <iostream>
#include <mirte_msgs/Encoder.h>
#include <mirte_msgs/GetEncoder.h>

#include <algorithm>
#include <cctype>
#include <iomanip>
#include <math.h>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <sstream>
#include <string>

class map_encoder {

public:
  map_encoder(ros::NodeHandle nh, std::string side, bool bidirectional,
              int ticks) {

    pub_ = n_.advertise<mirte_msgs::Encoder>("/mirte/encoder/" + side, 10);
    pub_speed_ =
        n_.advertise<mirte_msgs::Encoder>("/mirte/encoder_speed/" + side, 10);

    sub_ =
        n_.subscribe("/mirte/joint_states", 10, &map_encoder::callback, this);
    server_ = n_.advertiseService("/mirte/get_encoder_" + side,
                                  &map_encoder::service_cb, this);
    timer_ = n_.createTimer(ros::Duration(0.1), &map_encoder::timer, this);
    this->joint_name = "joint_" + side + "_wheel";
  }

  void callback(const sensor_msgs::JointState &joints) {
    if (joints.position.size() < 2) {
      return; // sometimes receive empty message
    }
    auto names = joints.name;
    auto index = std::distance(
        names.begin(), find(names.begin(), names.end(), this->joint_name));
    if (index >= names.size()) { // not in this message
      return;
    }
    auto distance = joints.position.at(index);
    auto ticks = map_rad_to_ticks(distance);

    mirte_msgs::Encoder mess;
    mess.header = joints.header;
    this->last_header = joints.header;

    if (this->bidirectional) {
      mess.value = ticks;
    } else {
      auto diff = ticks - this->last_actual_ticks;
      this->last_actual_ticks = ticks;
      mess.value =
          this->last_published_ticks + abs(diff); // always increase the ticks
    }
    this->last_published_ticks = mess.value;
    this->pub_.publish(mess);
  }

  bool service_cb(mirte_msgs::GetEncoder::Request &req,
                  mirte_msgs::GetEncoder::Response &res) {
    res.data = this->last_published_ticks;
    return true;
  }

  void timer(const ros::TimerEvent &event) {
    static int last_tick_count = 0;
    mirte_msgs::Encoder mess;
    mess.value = this->last_published_ticks - last_tick_count;
    if (!this->bidirectional) {
      mess.value = abs(mess.value);
    }
    mess.header = this->last_header;
    pub_speed_.publish(mess);
    last_tick_count = this->last_published_ticks;
  }

  int map_rad_to_ticks(double ang) {
    return (int)(ang / (2 * M_PI) * this->ticks_per_rot);
  }

private:
  // Create the necessary objects for subscribing and publishing
  ros::NodeHandle n_;
  ros::Publisher pub_;
  ros::Publisher pub_speed_;
  ros::Subscriber sub_;
  ros::ServiceServer server_;
  ros::Timer timer_;
  std_msgs::Header last_header;

  int last_actual_ticks = 0;
  int last_published_ticks = 0;
  bool bidirectional = false;
  int ticks_per_rot = 540;
  std::string joint_name;
}; // End of class SubscribeAndPublish

bool to_bool(std::string str) {
  std::transform(str.begin(), str.end(), str.begin(), ::tolower);
  std::istringstream is(str);
  bool b;
  is >> std::boolalpha >> b;
  return b;
}

int main(int argc, char **argv) {
  bool bidirectional = false;
  std::string side = argv[1];
  int ticks = 540;
  if (argc > 2) {
    bidirectional = to_bool(argv[2]);
    if (argc > 3) {
      ticks = std::stoi(argv[3]);
    }
  }
  std::string name = "map_encoder_" + side;
  ros::init(argc, argv, name);
  ros::NodeHandle nh;

  // Create an object of class pcl_object_detector_sub_and_pub that will take
  // care of everything
  map_encoder map_encoder_object(nh, side, bidirectional, ticks);
  ros::spin();
  return 0;
}
