#ifndef RT_UDP_CLIENT_COMPONENT_HPP_
#define RT_UDP_CLIENT_COMPONENT_HPP_

#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <rtt/Logger.hpp>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <std_msgs/Float32MultiArray.h>

using namespace RTT;

class rt_udp_client_component: public RTT::TaskContext {
public:
	rt_udp_client_component(const std::string& name);
	~rt_udp_client_component();

	virtual bool configureHook();
	virtual bool startHook();

	virtual void updateHook();
	virtual void stopHook();
	virtual void cleanupHook();

private:
	InputPort<std_msgs::Float32MultiArray> jpos_port;
	std_msgs::Float32MultiArray m_cmd_data;
	double jpos_out[7];

	int m_local_port, m_socket, m_remote_port;
	struct sockaddr_in m_remote_addr;
	socklen_t m_sock_addr_len;

};


#endif

