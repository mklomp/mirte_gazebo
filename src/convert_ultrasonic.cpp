#include <algorithm>
#include <iostream>

#include <mirte_msgs/GetDistance.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Range.h>

class map_sonar {

public:
  map_sonar(ros::NodeHandle nh, std::string side) {

    pub_ = n_.advertise<sensor_msgs::Range>("/mirte/distance/" + side, 10);

    sub_ =
        n_.subscribe("/mirte/scanSonar" + side, 10, &map_sonar::callback, this);
    server_ = n_.advertiseService("/mirte/get_distance_" + side,
                                  &map_sonar::service_cb, this);
  }

  void callback(const sensor_msgs::LaserScan &scan) {
    sensor_msgs::Range range;
    range.header = scan.header;
    range.radiation_type = sensor_msgs::Range::ULTRASOUND;
    range.field_of_view = scan.angle_max - scan.angle_min;
    range.min_range = scan.range_min;
    range.max_range = scan.range_max;
    range.range = *std::min_element(scan.ranges.begin(), scan.ranges.end());
    pub_.publish(range);
    this->lastRange = range.range;
  }
  bool service_cb(mirte_msgs::GetDistance::Request &req,
                  mirte_msgs::GetDistance::Response &res) {
    res.data = this->lastRange;
    return true;
  }

private:
  ros::NodeHandle n_;
  ros::Publisher pub_;
  ros::Subscriber sub_;
  ros::ServiceServer server_;
  float lastRange = 0;
};

int main(int argc, char **argv) {
  std::string side = argv[1];
  std::string name = "map_sonar_" + side;
  ros::init(argc, argv, name);
  ros::NodeHandle nh;
  map_sonar map_sonar_object(nh, side);
  ros::spin();
  return 0;
}
