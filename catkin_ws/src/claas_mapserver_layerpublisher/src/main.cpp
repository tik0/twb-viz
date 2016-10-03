#include <Constants.h>
using namespace claas::constants;
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf_conversions/tf_eigen.h>
#include "nav_msgs/GridCells.h"

#include <rsb/Factory.h>
#include <rsb/Version.h>
#include <rsb/Event.h>
#include <rsb/Handler.h>
#include <rsb/converter/Repository.h>
// #include <rsc/threading/SynchronizedQueue.h>
// #include <rsb/util/QueuePushHandler.h>
#include <rsb/converter/ProtocolBufferConverter.h>
#include <rsc/misc/SignalWaiter.h>
#include <rsc/threading/PeriodicTask.h>
#include <rsc/threading/ThreadedTaskExecutor.h>

// RST Proto types
#include <rst/geometry/Pose.pb.h>
#include <rst/geometry/Translation.pb.h>
#include <rst/geometry/Rotation.pb.h>
#include <rst/navigation/OccupancyGrid2DInt.pb.h>

using namespace rsb;
using namespace rsb::converter;

ros::Publisher publisher[mappingLayers::NUM_MAPS];
nav_msgs::GridCells msg;
void formatAndSendGridCell(boost::shared_ptr<rst::navigation::OccupancyGrid2DInt> ogm, int layerIdx) {
  geometry_msgs::Point point;
  msg.cell_height = mapping::discreteResolution;
  msg.cell_width = mapping::discreteResolution;
  // Fill this sloppy structure :(
  msg.cells.clear();
  for (int idy = 0; idy < ogm->height(); ++idy) {
    for (int idx = 0; idx < ogm->width(); ++idx) {
      // We get values from 0 .. 100
      if (ogm->map().at(idy * ogm->width() + idx) >  int(mapping::ogm::minDrawOccupancyUpdateCertainty * 100.0)) {
        point.x = (double(idx) * mapping::discreteResolution);
        point.y = (double(idy) * mapping::discreteResolution);
        point.z = double(layerIdx)/10.0;  // Overlay each layer by one centimeter
        msg.cells.push_back(point);
      }
    }
  }

  msg.header.frame_id = machine::frames::names::ROI_ORIGIN;
  msg.header.stamp = ros::Time::now();
  publisher[layerIdx].publish(msg);
}

 /**
 * This programm publishes a given maplayer in a given time
 */
int main(int argc, char **argv)
{
  ros::init(argc, argv, "claas_mapserver_layerpublisher");
  ros::NodeHandle node;
  for (int layerIdx = 0; layerIdx < mappingLayers::NUM_MAPS; ++layerIdx) {
    publisher[layerIdx] = node.advertise<nav_msgs::GridCells>(mappingLayers::ismScopes[layerIdx], 1);
  }

  // Get the RSB factory
  #if RSB_VERSION_NUMERIC<1200
    rsb::Factory& factory = rsb::Factory::getInstance();
  #else
    rsb::Factory& factory = rsb::getFactory();
  #endif

  // Prepare RSB client for OGM messages
  rsb::patterns::RemoteServerPtr remoteServer = factory.createRemoteServer(scopes::map::ogmServer::parent);
  // Register converters
  boost::shared_ptr< rsb::converter::ProtocolBufferConverter<rst::navigation::OccupancyGrid2DInt > > odomConverter(new rsb::converter::ProtocolBufferConverter<rst::navigation::OccupancyGrid2DInt >());
  rsb::converter::converterRepository<std::string>()->registerConverter(odomConverter);

  // We send once a second all the layers of the map server as grid cell
  ros::Rate r(1);
  ros::Time lastTfCheck = ros::Time::now();
  while (ros::ok()) {
    // Request the map and convert it to a GridCell
    try {
      boost::shared_ptr<rst::navigation::OccupancyGrid2DInt> result;

      for (int layerIdx = 0; layerIdx < mappingLayers::NUM_MAPS; ++layerIdx) {
        boost::shared_ptr<std::string> layerRequestName(new std::string(mappingLayers::mapRequestScopes[layerIdx]));
        result = remoteServer->call<rst::navigation::OccupancyGrid2DInt>(scopes::map::ogmServer::requests::singleLayerOgm, layerRequestName, 1);
//        ROS_INFO("The requested map has %d cells", result->map().size());
        formatAndSendGridCell(result, layerIdx);
      }
    } catch (...) {
      ROS_INFO("Map server error, just showing the vehicle without map");
    }

    ros::spinOnce();
    r.sleep();
  }

  return 0;
}
