#include <boost/asio.hpp>
#include <iostream>
#include <time.h>

int main(int argc, char **argv) {
  boost::asio::io_service io;
  // 接收端口8888
  boost::asio::ip::udp::socket sock(
      io, boost::asio::ip::udp::endpoint(boost::asio::ip::udp::v4(), 8888));

  struct timespec time = {0, 0};

  while (1) {

    std::vector<char> v(100, 0);
    boost::asio::ip::udp::endpoint ep;
    boost::system::error_code ec;
    sock.receive_from(boost::asio::buffer(v), ep, 0, ec);

    clock_gettime(CLOCK_REALTIME, &time);
    std::string time_ =
        std::to_string(time.tv_sec + time.tv_nsec / 1000000000.0);

    // std::cout << &v[0] << " " << time_ << std::endl;
    // 发送和接收时间差
    std::cout << 1000 * atof(time_.c_str()) - atof(&v[0]) << "ms" << std::endl;

    if (ec && ec != boost::asio::error::message_size) {
      throw std::system_error(ec);
    }

    // std::cout << "send to " << ep.address() << std::endl;
    sock.send_to(boost::asio::buffer("received!"), ep);
  }
}
