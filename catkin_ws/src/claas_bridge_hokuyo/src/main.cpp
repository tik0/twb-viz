// #include <Constants.h>

// ROS
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
ros::Publisher publisher;
sensor_msgs::LaserScan scan_msg;

// RSB
#include <rsb/Factory.h>
#include <rsb/converter/Repository.h>
#include <rsb/converter/ProtocolBufferConverter.h>
#include <rsb/MetaData.h>
#include <rst/vision/LocatedLaserScan.pb.h>
using namespace rsb;

// Variables
static std::string topic;
static std::string scope;
static std::string frame;
static int useRsbTimeStamp;
int useParamAngle = 0;
float angleMin_deg, angleMax_deg;

void callback(rsb::EventPtr event) {
  boost::shared_ptr<rst::vision::LocatedLaserScan> update
    = boost::static_pointer_cast<rst::vision::LocatedLaserScan>(event->getData());

  // TODO The angle are crap, take these from Constants.h, if they exist in the future
  if (useParamAngle) {
    scan_msg.angle_min = angleMin_deg * M_PI / 180;
    scan_msg.angle_max = angleMax_deg * M_PI / 180;
  } else {
    scan_msg.angle_min = update->scan_angle_start();
    scan_msg.angle_max = update->scan_angle_end();
  }
  scan_msg.angle_increment = update->scan_angle_increment();
  scan_msg.time_increment = 0;
  scan_msg.scan_time = 0;
  scan_msg.range_min = update->scan_values_min();
  scan_msg.range_max = update->scan_values_max();
  if (scan_msg.ranges.size() != update->scan_values_size()) {
    scan_msg.ranges.resize(update->scan_values_size(),0);
  }
  for (int idx = 0; idx < update->scan_values_size(); ++idx) {
    scan_msg.ranges.at(idx) = update->scan_values(idx);
  }
//  if (scan_msg.intensities.size() != update->pulsewidth_size()) {
//    scan_msg.intensities.resize(update->pulsewidth_size(),0);
//  }
//  for (int idx = 0; idx < update->pulsewidth_size(); ++idx) {
//    scan_msg.intensities.at(idx) = float(update->pulsewidth(idx));
//  }
  if (useRsbTimeStamp) {
    scan_msg.header.stamp.sec = event->getMetaData().getSendTime() * 1e-6;
    scan_msg.header.stamp.nsec = event->getMetaData().getSendTime() % uint64_t(1e6);
  } else {
    scan_msg.header.stamp = ros::Time::now();
  }
  scan_msg.header.frame_id = frame;
  publisher.publish(scan_msg);
}

/**
* This programm republishes all HOKUYO messages as ROS LaserScan
*/
int main(int argc, char **argv)
{
  // ROS
  ros::init(argc, argv, "claas_bridge_hokuyo");
  ros::NodeHandle n("~");

  n.param<std::string>("scope_rsb", scope, "/lidar");
  n.param<std::string>("topic_ros", topic, "/lidar");
  n.param<std::string>("frame", frame, "world");
  n.param<float>("angle_min_deg", angleMin_deg, -135);
  n.param<float>("angle_max_deg", angleMax_deg, +135);
  n.param<int>("use_param_angle", useParamAngle, 0);
  n.param<int>("use_rsb_timestamp", useRsbTimeStamp, false);

  publisher = n.advertise<sensor_msgs::LaserScan>(topic, 1);

  // RSB
  rsb::Factory& factory = rsb::getFactory();

  boost::shared_ptr< rsb::converter::ProtocolBufferConverter<rst::vision::LocatedLaserScan > > scanConverter(new rsb::converter::ProtocolBufferConverter<rst::vision::LocatedLaserScan >());
  rsb::converter::converterRepository<std::string>()->registerConverter(scanConverter);

  rsb::ListenerPtr laseListener = factory.createListener(scope);
  laseListener->addHandler(HandlerPtr(new EventFunctionHandler(&callback)));

  ros::spin();

  return 0;
}
