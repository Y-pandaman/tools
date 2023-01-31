#ifndef UDP_TO_CAN_HPP
#define UDP_TO_CAN_HPP

#include "loguru.hpp"
#include <boost/asio.hpp>
#include <iostream>
#include <bitset>
#include <thread>
#include <unistd.h>

#define BUFFER_SIZE 8 * 1024

struct can_msgs {
    uint32_t id;
    uint8_t data_len;
    uint8_t data[8];
    bool is_rtr;
    bool is_extended;
};

union Uint32_bytes {
    uint32_t data;
    uint8_t bytes[4];
};

class UdpToCan {
public:
    UdpToCan(int local_port);
    ~UdpToCan();

    void start(std::string remote_ip, int remote_port);
    void receiveData();
    void messageAnalysis(char *data, size_t len);
    void convertUdp2Can(char *data, can_msgs *can_message);
    bool isAllZeroValues(const can_msgs *can_frame);
    void canAnalysis(const can_msgs *can_frame);
    void canToUdp(const can_msgs can_frame);
private:
    boost::asio::io_service io_;
    boost::asio::ip::udp::socket *sockSend;
    boost::asio::ip::udp::endpoint ep_;
    boost::system::error_code ec_;
    boost::asio::ip::udp::endpoint remote_ep_;
    char buf_[BUFFER_SIZE];
    int local_port_;
    int remote_port_;
    std::string remote_ip_;
};

#endif