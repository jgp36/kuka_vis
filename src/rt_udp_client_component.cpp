#include <rtt/Component.hpp>
#include <rtt/Logger.hpp>

#include "rt_udp_client_component.hpp"

#include <netinet/in.h>
#include <sys/types.h>
#include <errno.h>
#include <fcntl.h>
#include <arpa/inet.h>
#include <rtt/os/TimeService.hpp>

#include <rtdm/rtdm.h>

using namespace RTT;

    rt_udp_client_component::rt_udp_client_component(const std::string& name) :
        TaskContext(name, PreOperational),
        m_socket(0)
    {
	this->addProperty("udp_port", m_local_port);
	this->addPort("jpos", jpos_port);

}

rt_udp_client_component::~rt_udp_client_component() {
}

bool rt_udp_client_component::configureHook() {
	
	m_socket = rt_dev_socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
	rt_dev_setsockopt(m_socket, SOL_SOCKET, SO_REUSEADDR, 0, 0);

	struct sockaddr_in local_addr;
	bzero((char *) &local_addr, sizeof(local_addr));
	local_addr.sin_family = AF_INET;
	local_addr.sin_addr.s_addr = INADDR_ANY;
	local_addr.sin_port = htons(m_local_port);
  
        bzero((char *) &m_remote_addr, sizeof(m_remote_addr));
  	m_remote_addr.sin_family = AF_INET;
  	m_remote_addr.sin_port = htons(m_local_port);
  	m_remote_addr.sin_addr.s_addr = inet_addr("192.168.0.101");

	if (rt_dev_bind(m_socket, (sockaddr*) &local_addr, sizeof(sockaddr_in)) < 0) {
		log(Error) << "Binding of port failed with errno " << errno << endlog();
		return false;
	}
	return true;
}

bool rt_udp_client_component::startHook() {
	return true;
}

void rt_udp_client_component::updateHook() {
	//Read:
	if (jpos_port.read(m_cmd_data) == NewData) {
	  for(size_t ii(0); ii < 7; ++ii) {
	    jpos_out[ii] = m_cmd_data.data[ii];
	  }

          int m = rt_dev_sendto(m_socket, (void*) &jpos_out, sizeof(jpos_out), 0,
 		(sockaddr*) &m_remote_addr, sizeof(m_remote_addr));
	  if (m < 0) {
	    std::cout << "Error in datagram: " << m << std::endl;
	  }
        }
}

void rt_udp_client_component::stopHook() {
}

void rt_udp_client_component::cleanupHook() {
	close(m_socket);
}


ORO_CREATE_COMPONENT(rt_udp_client_component)

