#include <Constants.h>
using namespace claas::constants;
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf_conversions/tf_eigen.h>

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
#include <rst/claas/MachineModel.pb.h>

using namespace rsb;
using namespace rsb::converter;

ros::Time lastTfSend;

void sendPose(float x_m, float y_m, float phi_rad, std::string origFram, std::string targetFrame) {
  static tf::TransformBroadcaster br;
  tf::Transform transform;
  transform.setOrigin( tf::Vector3(x_m, y_m, 0.0) );
  tf::Quaternion q;
  q.setRPY(0, 0, phi_rad);
  transform.setRotation(q);
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), origFram,  targetFrame));
}

// Store the current machine model (odometry)
void poseCallback(rsb::EventPtr event) {
  boost::shared_ptr<rst::claas::MachineModel_Odemetrie> update
    = boost::static_pointer_cast<rst::claas::MachineModel_Odemetrie>(event->getData());
  lastTfSend = ros::Time::now();
  // send global coordinates
  // sendPose(- (update->x_kon_glo() - update->x_kon_roi()), - (update->y_kon_glo() - update->y_kon_roi()), update->phi_kon() * deg2rad, machine::frames::names::WORLD,  machine::frames::names::ROI);
  // sendPose(-update->y_kon_glo() + update->y_kon_roi(), update->x_kon_glo() - update->x_kon_roi(), update->phi_kon() * deg2rad, machine::frames::names::WORLD,  machine::frames::names::ROI);
  sendPose(update->x_kon_glo() - update->x_kon_roi(), update->y_kon_glo() - update->y_kon_roi(), 0.0, machine::frames::names::WORLD,  machine::frames::names::ROI);
  // Send local coordinates
  sendPose(update->x_kon_roi(), update->y_kon_roi(), update->phi_kon() * deg2rad, machine::frames::names::ROI,  machine::frames::names::MACHINE_ROI);
}

 /**
 * This programm publishes all static transformations on the machine
 */
int main(int argc, char **argv)
{
  ros::init(argc, argv, "claas_tf_machinemodel");
  ros::NodeHandle node;

  // Get the RSB factory
  #if RSB_VERSION_NUMERIC<1200
    rsb::Factory& factory = rsb::Factory::getInstance();
  #else
    rsb::Factory& factory = rsb::getFactory();
  #endif
  
  // Register converters
  boost::shared_ptr< rsb::converter::ProtocolBufferConverter<rst::claas::MachineModel_Odemetrie > > odomConverter(new rsb::converter::ProtocolBufferConverter<rst::claas::MachineModel_Odemetrie >());
  rsb::converter::converterRepository<std::string>()->registerConverter(odomConverter);

  // Prepare RSB listener for odometry (MachineModel)
  rsb::ListenerPtr machineModelListener = factory.createListener(scopes::odometry);
  machineModelListener->addHandler(HandlerPtr(new EventFunctionHandler(&poseCallback)));
  
  // We send once a second the initial position, if no new odometry messages
  // have been received
  ros::Rate r(2);
  ros::Time lastTfCheck = ros::Time::now();
  while (ros::ok()) {
    if ((lastTfSend.sec - lastTfCheck.sec) > 1) {
      sendPose(0.0, 0.0, 0.0, machine::frames::names::WORLD,  machine::frames::names::ROI);
      sendPose(0.0, 0.0, 0.0, machine::frames::names::ROI,  machine::frames::names::MACHINE_ROI);
    }
    lastTfCheck = ros::Time::now();
    ros::spinOnce();
    r.sleep();
  }

  return 0;
}
