#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/types.h>
#include <errno.h>
#include <fcntl.h>
#include <rtdm/rtdm.h>

static ros::Subscriber joint_sub;
static int m_local_port, m_socket;
static struct sockaddr m_remote_addr;
static std_msgs::Float32MultiArray m_cmd_data;

void qCallback(const std_msgs::Float32MultiArray::ConstPtr& msg) {
  for (size_t ii(0); ii < 7; ++ii) {
    m_cmd_data.data[ii] = msg->data[ii];
  }

  if (0 > rt_dev_sendto(m_socket, (void*) &m_cmd_data, sizeof(m_cmd_data), 0,
		(sockaddr*) &m_remote_addr, sizeof(m_remote_addr)))
	ROS_WARN("Error in datagram");
}

int main(int argc, char ** argv) {

  char* def_node = "vis_rt_udp_client";
  char* def_in = "/visualize_lwr";
  if (argc > 2) {
    def_node = argv[1];
  }
  if (argc == 3) {
    def_in = argv[2];
  }

  ros::init(argc, argv, def_node);
  ros::NodeHandle node("~");

  m_cmd_data.data.resize(7);

  m_socket = rt_dev_socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
  rt_dev_setsockopt(m_socket, SOL_SOCKET, SO_REUSEADDR, 0, 0);

  m_local_port = 11111;
  struct sockaddr_in local_addr;
  bzero((char *) &local_addr, sizeof(local_addr));
  local_addr.sin_family = AF_INET;
  local_addr.sin_addr.s_addr = INADDR_ANY;
  local_addr.sin_port = htons(m_local_port);

  if (rt_dev_bind(m_socket, (sockaddr*) &local_addr, sizeof(sockaddr_in)) < 0) {
	ROS_WARN("Binding of port failed with errno %d",errno);
	return false;
  }

  joint_sub = node.subscribe(def_in,1,qCallback);

  ros::spin();

  ros::shutdown();
}
