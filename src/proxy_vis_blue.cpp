//ROS
#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <std_msgs/Float32MultiArray.h>
#include <sstream>
#include <tf/transform_broadcaster.h>
#include <Eigen/Geometry>

using namespace std;
using namespace Eigen;

static visualization_msgs::MarkerArray robot_markers;

void updatePose(double q[7]) {

  //Joint 1
  Quaterniond r1(AngleAxisd(q[0],Vector3d::UnitZ())*AngleAxisd(M_PI,Vector3d::UnitZ()));
  robot_markers.markers[1].pose.position.x = 0;
  robot_markers.markers[1].pose.position.y = 0;
  robot_markers.markers[1].pose.position.z = 0.11;

  robot_markers.markers[1].pose.orientation.x = r1.x();
  robot_markers.markers[1].pose.orientation.y = r1.y();
  robot_markers.markers[1].pose.orientation.z = r1.z();
  robot_markers.markers[1].pose.orientation.w = r1.w();

  //Joint 2
  Quaterniond r2(r1*AngleAxisd(q[1],Vector3d::UnitY()));
  Vector3d p2(r2*Vector3d(0.0,0.0,0.2005));
  r2 = r2*AngleAxisd(M_PI,Vector3d::UnitX());
 
  robot_markers.markers[2].pose.position.x = p2[0];
  robot_markers.markers[2].pose.position.y = p2[1];
  robot_markers.markers[2].pose.position.z = p2[2]+0.11-0.008+0.208;

  robot_markers.markers[2].pose.orientation.x = r2.x();
  robot_markers.markers[2].pose.orientation.y = r2.y();
  robot_markers.markers[2].pose.orientation.z = r2.z();
  robot_markers.markers[2].pose.orientation.w = r2.w();

  //Joint 3
  Quaterniond r3(r1*AngleAxisd(q[1],Vector3d::UnitY())*AngleAxisd(q[2],Vector3d::UnitZ())*AngleAxisd(M_PI,Vector3d::UnitZ()));
 
  robot_markers.markers[3].pose.position.x = p2[0];
  robot_markers.markers[3].pose.position.y = p2[1];
  robot_markers.markers[3].pose.position.z = p2[2]+0.11-0.008+0.208;

  robot_markers.markers[3].pose.orientation.x = r3.x();
  robot_markers.markers[3].pose.orientation.y = r3.y();
  robot_markers.markers[3].pose.orientation.z = r3.z();
  robot_markers.markers[3].pose.orientation.w = r3.w();

  //Joint 4
  Quaterniond r4(r3*AngleAxisd(q[3],Vector3d::UnitY()));
  Vector3d p4a(r1*AngleAxisd(q[1],Vector3d::UnitY())*Vector3d(0.0,0.0,0.2005*2));
  Vector3d p4b(r4*Vector3d(0.0,0.0,0.2005));
  r4 = r4*AngleAxisd(M_PI,Vector3d::UnitX());
 
  robot_markers.markers[4].pose.position.x = p4a[0]+p4b[0];
  robot_markers.markers[4].pose.position.y = p4a[1]+p4b[1];
  robot_markers.markers[4].pose.position.z = p4a[2]+p4b[2]+0.11-0.008+0.208;

  robot_markers.markers[4].pose.orientation.x = r4.x();
  robot_markers.markers[4].pose.orientation.y = r4.y();
  robot_markers.markers[4].pose.orientation.z = r4.z();
  robot_markers.markers[4].pose.orientation.w = r4.w();

  //Joint 5
  Quaterniond r5(r4*AngleAxisd(-q[4],Vector3d::UnitZ())*AngleAxisd(M_PI,Vector3d::UnitX())*AngleAxisd(M_PI,Vector3d::UnitZ()));
 
  robot_markers.markers[5].pose.position.x = p4a[0]+p4b[0];
  robot_markers.markers[5].pose.position.y = p4a[1]+p4b[1];
  robot_markers.markers[5].pose.position.z = p4a[2]+p4b[2]+0.11-0.008+0.208;

  robot_markers.markers[5].pose.orientation.x = r5.x();
  robot_markers.markers[5].pose.orientation.y = r5.y();
  robot_markers.markers[5].pose.orientation.z = r5.z();
  robot_markers.markers[5].pose.orientation.w = r5.w();

  //Joint 6
  Quaterniond r6(r5*AngleAxisd(q[5],Vector3d::UnitY()));
  Vector3d p6(r3*AngleAxisd(q[3],Vector3d::UnitY())*Vector3d(0.0,0.0,0.2008+0.19));
 
  robot_markers.markers[6].pose.position.x = p4a[0]+p6[0];
  robot_markers.markers[6].pose.position.y = p4a[1]+p6[1];
  robot_markers.markers[6].pose.position.z = p4a[2]+p6[2]+0.11-0.008+0.208;

  robot_markers.markers[6].pose.orientation.x = r6.x();
  robot_markers.markers[6].pose.orientation.y = r6.y();
  robot_markers.markers[6].pose.orientation.z = r6.z();
  robot_markers.markers[6].pose.orientation.w = r6.w();

  //Joint 7
  Quaterniond r7(r6*AngleAxisd(q[6],Vector3d::UnitZ()));
  Vector3d p7(r7*Vector3d(0.0,0.0,0.078));
 
  robot_markers.markers[7].pose.position.x = p4a[0]+p6[0]+p7[0];
  robot_markers.markers[7].pose.position.y = p4a[1]+p6[1]+p7[1];
  robot_markers.markers[7].pose.position.z = p4a[2]+p6[2]+p7[2]+0.11-0.008+0.208;

  robot_markers.markers[7].pose.orientation.x = r7.x();
  robot_markers.markers[7].pose.orientation.y = r7.y();
  robot_markers.markers[7].pose.orientation.z = r7.z();
  robot_markers.markers[7].pose.orientation.w = r7.w();
}

void qCallback(const std_msgs::Float32MultiArray::ConstPtr& msg) {
  double q[7];
  for (size_t ii(0); ii < 7; ++ii) {
    q[ii] = msg->data[ii];
  }
  updatePose(q);
}

int main(int argc, char ** argv) {

  char* def_in_address = "/visualize_proxy_blue";
  if (argc > 2) {
    def_in_address = argv[1];
  }

  char* def_out_address = "proxy_marker_blue";
  if (argc > 3) {
    def_out_address = argv[2];
  }

  ros::init(argc, argv, "proxy_display_blue");
  ros::NodeHandle node("~");

  ros::Publisher robot_pub = node.advertise<visualization_msgs::MarkerArray>( def_out_address, 1 );
  ros::Subscriber joint_sub = node.subscribe(def_in_address,1,qCallback);

  string meshes[8];
  meshes[0] = "package://kuka_vis/meshes/lwr/arm_base.dae";
  meshes[1] = "package://kuka_vis/meshes/lwr/arm_segment_a.dae";
  meshes[2] = "package://kuka_vis/meshes/lwr/arm_segment_b.dae";
  meshes[3] = "package://kuka_vis/meshes/lwr/arm_segment_a.dae";
  meshes[4] = "package://kuka_vis/meshes/lwr/arm_segment_b.dae";
  meshes[5] = "package://kuka_vis/meshes/lwr/arm_segment_last.dae";
  meshes[6] = "package://kuka_vis/meshes/lwr/arm_wrist.dae";
  meshes[7] = "package://kuka_vis/meshes/lwr/link_7.STL";

  for (size_t ii(0); ii < 8; ++ii) {
    robot_markers.markers.push_back(visualization_msgs::Marker());
    robot_markers.markers[ii].header.frame_id = "/base_link";
    robot_markers.markers[ii].header.stamp = ros::Time();
    robot_markers.markers[ii].ns = "rviz_display";
    robot_markers.markers[ii].id = ii;
    robot_markers.markers[ii].type = visualization_msgs::Marker::MESH_RESOURCE;
    robot_markers.markers[ii].action = visualization_msgs::Marker::ADD;
    robot_markers.markers[ii].scale.x = 1.0;
    robot_markers.markers[ii].scale.y = 1.0;
    robot_markers.markers[ii].scale.z = 1.0;
    robot_markers.markers[ii].color.a = 0.5;
    robot_markers.markers[ii].color.r = 0.0;
    robot_markers.markers[ii].color.g = 0.0;
    robot_markers.markers[ii].color.b = 0.9;
    robot_markers.markers[ii].mesh_resource = meshes[ii];
    robot_markers.markers[ii].mesh_use_embedded_materials = true;
  }

  //Base never moves
  Quaterniond base(AngleAxisd(M_PI,Vector3d::UnitZ()));
  robot_markers.markers[0].pose.position.x = 0;
  robot_markers.markers[0].pose.position.y = 0;
  robot_markers.markers[0].pose.position.z = 0;
  robot_markers.markers[0].pose.orientation.x = base.x();
  robot_markers.markers[0].pose.orientation.y = base.y();
  robot_markers.markers[0].pose.orientation.z = base.z();
  robot_markers.markers[0].pose.orientation.w = base.w();

  double q[7];
  for (size_t ii(0); ii < 7; ++ii) {
    q[ii] = 0;
  }
  //Initial configuration assumed to zero
  updatePose(q);

  tf::TransformBroadcaster br;
  tf::Transform transform;
  transform.setOrigin( tf::Vector3(0.0, 0.0, 0.0) );
  transform.setRotation( tf::Quaternion(0.0, 0.0, 0.0, 1.0) );


  ros::Rate r(60);
  while (ros::ok()) { 
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "base_link"));
    robot_pub.publish(robot_markers);

    ros::spinOnce();
    r.sleep();
  }
  ros::shutdown();
}
