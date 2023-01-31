#include <boost/asio.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <iostream>
#include <time.h>
#include <unistd.h>

int main(int argc, char **argv) {
  boost::asio::io_service io_service;
  boost::asio::ip::udp::socket socket(io_service);
  boost::asio::ip::udp::endpoint remote_endpoint;

  socket.open(boost::asio::ip::udp::v4());
  // remote_endpoint = boost::asio::ip::udp::endpoint(
  //     boost::asio::ip::address::from_string("100.102.137.21"), 8888);
  remote_endpoint = boost::asio::ip::udp::endpoint(
      boost::asio::ip::address::from_string("127.0.0.1"), 8888);

  struct timespec time = {0, 0};

  while (1) {
    // 标准时间
    // std::string message = boost::posix_time::to_simple_string(
    //     boost::posix_time::second_clock::local_time());

    // 时间戳
    clock_gettime(CLOCK_REALTIME, &time);
    std::string message =
        std::to_string(time.tv_sec + time.tv_nsec / 1000000000.0);

    // 发送数据
    socket.send_to(boost::asio::buffer(message), remote_endpoint);
    std::cout << message << std::endl;

    // 接收数据
    // std::vector<char> v(100, 0);
    // boost::asio::ip::udp::endpoint recv_ep;
    // socket.receive_from(boost::asio::buffer(v), recv_ep);
    // std::cout << "recv from " << recv_ep.address() << " " << &v[0] <<
    // std::endl;

    usleep(1000000);
  }

  return 0;
}