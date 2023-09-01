#include "std_msgs/String.h"
#include <iostream>
#include <mirte_msgs/GetIntensity.h>
#include <mirte_msgs/GetIntensityDigital.h>
#include <mirte_msgs/Intensity.h>
#include <mirte_msgs/IntensityDigital.h>

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <std_srvs/Empty.h>

class map_ir {

public:
  map_ir(ros::NodeHandle nh, std::string side) {

    pub_ = n_.advertise<mirte_msgs::Intensity>("/mirte/intensity/" + side, 10);
    pub_dig_ = n_.advertise<mirte_msgs::IntensityDigital>(
        "/mirte/intensity/" + side + "_digital", 10);
    sub_ = n_.subscribe("/mirte/camera_ir_" + side + "/image_raw", 10,
                        &map_ir::callback, this);
    server_ = n_.advertiseService("/mirte/get_intensity_" + side,
                                  &map_ir::service_cb, this);
    server_dig_ =
        n_.advertiseService("/mirte/get_intensity_" + side + "_digital",
                            &map_ir::service_cb_dig, this);
  }

  void callback(const sensor_msgs::Image &img) {
    mirte_msgs::Intensity irInt;
    irInt.value = img.data[0];
    irInt.header = img.header;
    this->lastIntensity = irInt.value;
    pub_.publish(irInt);

    mirte_msgs::IntensityDigital irIntDig;
    irIntDig.value = img.data[0] > 128;
    irIntDig.header = img.header;
    this->lastIntensityDig = irIntDig.value;
    pub_dig_.publish(irIntDig);
  }

  bool service_cb(mirte_msgs::GetIntensity::Request &req,
                  mirte_msgs::GetIntensity::Response &res) {

    res.data = this->lastIntensity;
    return true;
  }
  bool service_cb_dig(mirte_msgs::GetIntensityDigital::Request &req,
                      mirte_msgs::GetIntensityDigital::Response &res) {
    res.data = this->lastIntensityDig;
    return true;
  }

private:
  // Create the necessary objects for subscribing and publishing
  ros::NodeHandle n_;
  ros::Publisher pub_;
  ros::Publisher pub_dig_;
  ros::Subscriber sub_;
  ros::ServiceServer server_;
  ros::ServiceServer server_dig_;

  int lastIntensity = 0;
  bool lastIntensityDig = false;
}; // End of class SubscribeAndPublish

int main(int argc, char **argv) {
  std::string side = argv[1];
  std::string name = "map_ir_" + side;
  ros::init(argc, argv, name);
  ros::NodeHandle nh;

  // Create an object of class pcl_object_detector_sub_and_pub that will take
  // care of everything
  map_ir map_ir_object(nh, side);
  ros::spin();
  return 0;
}
