#include <functional>

#include <ros/ros.h>
#include <boost/asio/buffer.hpp>
#include <boost/asio/io_service.hpp>
#include <boost/asio/ip/udp.hpp>

// http://wiki.ros.org/roscpp/Overview/MessagesSerializationAndAdaptingTypes
#define UDPBUFFERSIZE 65535

namespace dtransmit {

typedef int PORT;

struct Socket {
  boost::asio::ip::udp::socket *socket;

  std::function<void(const boost::system::error_code &, std::size_t)> readHandler;

  uint8_t recvBuffer[UDPBUFFERSIZE];

  boost::asio::ip::udp::endpoint remoteEndpoint;


  Socket() {}

  Socket(boost::asio::io_service &service, PORT port) {

    socket = new boost::asio::ip::udp::socket(service);

    boost::asio::ip::udp::endpoint rx_endpoint_(boost::asio::ip::udp::v4(), port);
    boost::system::error_code error;
    socket->open(rx_endpoint_.protocol(), error);
    if (error) {
      ROS_ERROR("Can't open recv socket");
    } else {
      socket->set_option(boost::asio::ip::udp::socket::reuse_address(true));
      socket->bind(rx_endpoint_, error);
      if (error) {
        ROS_ERROR("Can't bind recv socket");
      }
    }
  }

  ~Socket() {}
};

}
