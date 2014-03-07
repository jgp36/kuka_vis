#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/types.h>
#include <errno.h>
#include <fcntl.h>

int main(int argc, char ** argv) {

  char* def_node = "vis_udp_sever";
  char* def_out = "/visualize_lwr";
  if (argc > 2) {
    def_node = argv[1];
  }
  if (argc == 3) {
    def_out = argv[2];
  }

  int m_local_port, m_socket;
  struct sockaddr m_remote_addr;
  std_msgs::Float32MultiArray m_msr_data;
  socklen_t m_sock_addr_len;
  m_local_port = 22222;

  ros::init(argc, argv, def_node);
  ros::NodeHandle node("~");

  ros::Publisher robot_pub = node.advertise<std_msgs::Float32MultiArray>( def_out, 1 );

  if (m_socket != 0)
 	close(m_socket);
  m_socket = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
  setsockopt(m_socket, SOL_SOCKET, SO_REUSEADDR, 0, 0);

  struct sockaddr_in local_addr;
  bzero((char *) &local_addr, sizeof(local_addr));
  local_addr.sin_family = AF_INET;
  local_addr.sin_addr.s_addr = INADDR_ANY;
  local_addr.sin_port = htons(m_local_port);

  if (bind(m_socket, (sockaddr*) &local_addr, sizeof(sockaddr_in)) < 0) {
 	ROS_WARN("Binding of port failed with errno %d",errno);
  }
  
  ros::Rate r(1000);
  while (ros::ok()) { 
    int n = recvfrom(m_socket, (void*) &m_msr_data, sizeof(m_msr_data), 0,
			(sockaddr*) &m_remote_addr, &m_sock_addr_len); 
    robot_pub.publish(m_msr_data);

    ros::spinOnce();
    r.sleep();
  }
  ros::shutdown();
}
