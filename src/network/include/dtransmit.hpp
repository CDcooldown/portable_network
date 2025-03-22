#include <algorithm>
#include <ifaddrs.h>

#include <ros/ros.h>



#include <boost/asio/buffer.hpp>
#include <boost/asio/io_service.hpp>
#include <boost/asio/ip/udp.hpp>

#include <functional>
#include <iostream>
#include <memory>
#include <string>
#include <thread>
#include <vector>
#include <utility>
#include <map>

#include "recv_socket.hpp"

using namespace std;

typedef int PORT;

namespace dtransmit {

/**
 * @brief Transmitting ROS messages and other information over UDP.
 */
class DTransmit {

 public:
  /**
   * @brief DTransmit constructor.
   *
   * @param address - udp broadcast address
   */
  explicit DTransmit(const std::string &address = "",
                     const bool &use_local_loop = false);
  /**
   * @brief DTransmit destructor
   */
  ~DTransmit();

  /**
   * @brief Add listener for receiving ROS messages.
   *
   * @tparam ROSMSG - ROS message type
   * @param port - listening port
   * @param callback - callback function when receiving messages
   */
  void addRawRecv(PORT port, std::function<void(void *, std::size_t)> callback);

  /**
   * @brief Add listener for receiving raw messages from specific remote
   * endpoint.
   *
   * @param port - listening port
   * @param remoteEndpoint - given remote endpoint address
   * @param callback - callback function when receiving messages
   */
  void addRawRecvFiltered(PORT port, std::string remoteEndpoint,
                          std::function<void(void *, std::size_t)> callback);

  /**
   * @brief Send raw messages.
   *
   * @param port - gievn port
   * @param buffer - buffer to send
   * @param size - size of buffer
   */
  void sendRaw(PORT port, const void *buffer, std::size_t size);

  /**
   * @brief Start service of asio.
   */
  void startService();

 private:
  /**
   * @brief Start receiving.
   *
   * @tparam ReadHandler - type of handler
   * @param port - port for receiving messages
   * @param handler - handler for reading Sockets
   */
  template<typename ReadHandler>
  void startRecv(PORT port, ReadHandler handler);
  /**
   * @brief Create Socket for sending messages.
   *
   * @param addr - broadcast address for sending messages
   * @param port - port for sending messages
   */
  void createSendSocket(const std::string &addr, const PORT &port);
  /**
   * @brief Send buffer.
   *
   * @param boost::asio::ip::udp::socket - Socket
   * @param buffer - buffer to send
   * @param size - size of buffer
   */
  void sendBuffer(boost::asio::ip::udp::socket *, const void *buffer,
                  std::size_t size);

  /**
   * @brief Retrieve for all interfaces the broadcast address
   */
  void retrieveBroadcastAddress();

  //! Broadcast addresses of all interfaces
  std::vector<std::string> broadcast_addresses_;
  //! IO service
  boost::asio::io_service service_;

  //! Thread instance
  std::thread thread_;
  //! Map of ports and corresponding Socket for receiving messages
  std::map<PORT, Socket> recv_Sockets_;
  //! Map of ports and Sockets for sending messages
  std::map<std::pair<std::string, PORT>, boost::asio::ip::udp::socket *> send_sockets_;
};

template<typename ReadHandler>
void DTransmit::startRecv(PORT port, ReadHandler handler) {
  recv_Sockets_[port].socket->async_receive_from(
      boost::asio::buffer(
          boost::asio::mutable_buffer((void *) &recv_Sockets_[port].recvBuffer,
                                      sizeof(recv_Sockets_[port].recvBuffer))),
      recv_Sockets_[port].remoteEndpoint, handler);
}

}  // namespace dtransmit

using namespace boost::asio;

namespace dtransmit {
DTransmit::DTransmit(const std::string &address, const bool &use_local_loop)
    : service_() {
  // add broadcast address
  if (!address.empty()) {
    broadcast_addresses_.push_back(address);
  } else {
    retrieveBroadcastAddress();
  }
  // remove local loop 127.0.0.1 from broadcast address
  if (!use_local_loop) {
    broadcast_addresses_.erase(
        std::remove_if(broadcast_addresses_.begin(),
                       broadcast_addresses_.end(),
                       [](const std::string &addr) {
                         return addr == "127.0.0.1" || addr.find("172.17.") != std::string::npos;
                       }),
        broadcast_addresses_.end());
  }
  // pirnt info
  for (const auto &addr:broadcast_addresses_) {
    ROS_INFO("New DTransmit on %s", addr.c_str());
  }
}

DTransmit::~DTransmit() {
  service_.stop();
  for (auto &p : send_sockets_) {
    ip::udp::socket *s = p.second;
    if (s) {
      if (s->is_open()) s->close();
      delete s;
    }
  }

  for (auto &p : recv_Sockets_) {
    ip::udp::socket *s = p.second.socket;
    if (s) {
      if (s->is_open()) s->close();

      delete s;
    }
  }

  service_.stop();
  if (thread_.joinable()) thread_.join();
}

void DTransmit::startService() {
  thread_ = std::thread([&]() {
    service_.reset();
    service_.run();
  });
}

void DTransmit::createSendSocket(const std::string &addr, const PORT &port) {
  using namespace boost::asio;

  ip::udp::endpoint broadcastEndpoint(
  ip::address::from_string(addr), port);
  send_sockets_[std::make_pair(addr, port)] = new ip::udp::socket(service_, ip::udp::v4());
  send_sockets_[std::make_pair(addr, port)]->set_option(socket_base::broadcast(true));

  boost::system::error_code ec;
  send_sockets_[std::make_pair(addr, port)]->connect(broadcastEndpoint, ec);
  if (ec) {
    ROS_ERROR("DTransmit create sendRos socket error: %s",
              ec.message().c_str());
  }
  ROS_DEBUG("DTransmit create send socket on %s:%d", addr.c_str(), port);
}

void DTransmit::sendBuffer(boost::asio::ip::udp::socket *socket,
                           const void *buffer, std::size_t size) {
  boost::system::error_code ec;
  socket->send(boost::asio::buffer(buffer, size), 0, ec);

  if (ec) {
    ROS_WARN("DTransmit can't send Ros buffer to %s:%d : %s",
             socket->local_endpoint().address().to_string().c_str(),
             socket->local_endpoint().port(),
             ec.message().c_str());
  }
}

void DTransmit::addRawRecv(PORT port,
                           std::function<void(void *, std::size_t)> callback) {

  if (recv_Sockets_.count(port)) {
    ROS_ERROR("Error in addRawRecv: port %d exist!", port);
    return;
  }
  recv_Sockets_[port] = Socket(service_, port);

  recv_Sockets_[port].readHandler = [=](const boost::system::error_code &error,
                                    std::size_t bytesRecved) {
    if (error) {
      ROS_ERROR("Error in RawRecv: %s", error.message().c_str());
    } else {
      callback(recv_Sockets_[port].recvBuffer, bytesRecved);
    }
  
    startRecv(port, recv_Sockets_[port].readHandler);
  };
  startRecv(port, recv_Sockets_[port].readHandler);
}

void DTransmit::sendRaw(PORT port, const void *buffer, std::size_t size) {

  for (const auto &addr:broadcast_addresses_) {
    if (!send_sockets_.count(std::make_pair(addr, port))) {
      createSendSocket(addr, port);
    }
    sendBuffer(send_sockets_[std::make_pair(addr, port)], buffer, size);
    ROS_DEBUG("DTransmit send raw data on %s:%d", addr.c_str(), port);
  }
}

void DTransmit::retrieveBroadcastAddress() {
  struct ifaddrs *ifap;
  if (getifaddrs(&ifap) == -1) {
    ROS_ERROR("Failed to getting broadcast address: %s", strerror(errno));
    return;
  }

  while (ifap != nullptr) {
    struct sockaddr *addr = ifap->ifa_broadaddr;
    if (addr != nullptr && addr->sa_family == AF_INET) {
      char str[INET_ADDRSTRLEN];
      inet_ntop(AF_INET, &(((struct sockaddr_in *) addr)->sin_addr), str, INET_ADDRSTRLEN);
      broadcast_addresses_.emplace_back(str);
      ROS_INFO("New DTransmit on %s", str);
    }
    ifap = ifap->ifa_next;
  }

}
}
