#include "udp_to_can.hpp"

UdpToCan::UdpToCan(int local_port) {
    local_port_ = local_port;
}

void UdpToCan::start(std::string remote_ip, int remote_port) {
    remote_ip_   = remote_ip;
    remote_port_ = remote_port;
    remote_ep_   = boost::asio::ip::udp::endpoint(
        boost::asio::ip::address::from_string(remote_ip_), remote_port_);
    sockSend = new boost::asio::ip::udp::socket(io_);
    sockSend->open(boost::asio::ip::udp::v4());

    can_msgs msg;
    msg.id       = 0x11223344;
    msg.data_len = 8;
    for(int i = 0; i < msg.data_len; i++) {
        msg.data[i] = { 0xFF };
    }
    msg.is_rtr      = false;
    msg.is_extended = false;

    std::thread t(&UdpToCan::canToUdp, this, msg);
    t.detach();
}

/**
 * @brief 获取udp数据
 *
 * @param null
 */
void UdpToCan::receiveData() {
    LOG_F(INFO, "receive data from port %d", local_port_);
    memset(buf_, '\0', sizeof(buf_));  // 清空buf
    boost::asio::ip::udp::socket sockRecv(
        io_, boost::asio::ip::udp::endpoint(boost::asio::ip::udp::v4(),
                                            local_port_));
    sockRecv.receive_from(boost::asio::buffer(buf_), ep_, 0, ec_);
    // LOG_F(INFO, "buf info: ", buf_);

    if(ec_ && ec_ != boost::asio::error::message_size) {
        throw std::system_error(ec_);
    }

    messageAnalysis(buf_, sizeof(buf_));
}

/**
 * @brief 解析udp数据
 *
 * @param buf udp缓冲区数据
 * @param len 数据长度
 */
void UdpToCan::messageAnalysis(char *buf, size_t len) {
    size_t can_frame_size = 0;
    if(len % 13 == 0) {
        can_frame_size = len / 13;
    }
    else {
        LOG_F(WARNING, "buf data length: %ld", len);
        return;
    }
    LOG_F(INFO, "can frame size: %ld，data long： %ld", can_frame_size, len);

    for(size_t i = 0; i < can_frame_size; i++) {
        buf += i * 13;
        // LOG_F(INFO, "buf[0]: %x", buf[0]);
        // std::bitset<8> b(buf[1]);
        // std::cout << b << std::endl;   // 二进制表示
        if((buf[0] & 0x30) == 0x00) {  // 四五RESVD位必须为0
            can_msgs canFrame;
            convertUdp2Can(buf, &canFrame);
            canAnalysis(&canFrame);
        }
        else {
            LOG_F(WARNING, "received data, but not can frame!");
        }
    }
}

/**
 * @brief 分解can帧
 *
 * @param data udp数据
 * @param can_message  can帧
 */
void UdpToCan::convertUdp2Can(char *data, can_msgs *can_message) {
    can_message->data_len = data[0] & 0x0f;  // &0x0f取低四位

    Uint32_bytes id{};
    id.bytes[0]     = data[4];
    id.bytes[1]     = data[3];
    id.bytes[2]     = data[2];
    id.bytes[3]     = data[1];
    can_message->id = id.data;

    can_message->is_rtr      = data[0] & 0x40;  // 数据帧|远程帧
    can_message->is_extended = data[0] & 0x80;  // 扩展帧

    for(uint8_t i = 0; i < can_message->data_len; i++) {
        can_message->data[i] = data[i + 5];
    }
}

/**
 * @brief 检测can帧是否有数据
 *
 * @param can_frame can帧
 * @return true
 * @return false
 */
bool UdpToCan::isAllZeroValues(const can_msgs *can_frame) {
    for(uint8_t i = 0; i < can_frame->data_len; i++) {
        if(can_frame->data[i] != 0)
            return false;
    }
    LOG_F(WARNING, "received a can frame 0x%x, but ALL ZERO! not process",
          can_frame->id);
    return true;
}

/**
 * @brief 解析can帧
 *
 * @param can_frame can帧
 */
void UdpToCan::canAnalysis(const can_msgs *can_frame) {
    if(isAllZeroValues(can_frame)) {
        return;
    }
    LOG_F(INFO, "received can frame from CANET: 0x%x", can_frame->id);

    switch(can_frame->id) {
    case 0x11223344:
        LOG_F(INFO, "success");
        LOG_F(INFO, "0x11223344 data: 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x",
              can_frame->data[0], can_frame->data[1], can_frame->data[2],
              can_frame->data[3], can_frame->data[4], can_frame->data[5],
              can_frame->data[6], can_frame->data[7]);
        break;
    case 0x181:
        LOG_F(INFO, "0x181 data: 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x",
              can_frame->data[0], can_frame->data[1], can_frame->data[2],
              can_frame->data[3], can_frame->data[4], can_frame->data[5],
              can_frame->data[6], can_frame->data[7]);
        break;
    case 0x281:
        LOG_F(INFO, "0x281 data: 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x",
              can_frame->data[0], can_frame->data[1], can_frame->data[2],
              can_frame->data[3], can_frame->data[4], can_frame->data[5],
              can_frame->data[6], can_frame->data[7]);
        break;
    case 0x381:
        LOG_F(INFO, "0x381 data: 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x",
              can_frame->data[0], can_frame->data[1], can_frame->data[2],
              can_frame->data[3], can_frame->data[4], can_frame->data[5],
              can_frame->data[6], can_frame->data[7]);
        break;

    default:
        break;
    }
}

/**
 * @brief 将can数据通过udp发送
 *
 * @param can_frame can帧数据
 */
void UdpToCan::canToUdp(const can_msgs can_frame) {
    uint8_t info         = can_frame.data_len;
    uint8_t sendData[13] = { 0 };

    if(can_frame.is_rtr) {
        info |= 0x40;
    }

    if(can_frame.is_extended) {
        info |= 0x80;
    }
    sendData[0] = info;

    Uint32_bytes id;
    id.data     = can_frame.id;
    sendData[1] = id.bytes[3];
    sendData[2] = id.bytes[2];
    sendData[3] = id.bytes[1];
    sendData[4] = id.bytes[0];

    for(uint8_t i = 0; i < can_frame.data_len; i++) {
        sendData[i + 5] = can_frame.data[i];
    }

    while(1) {
        LOG_F(INFO, "send id: 0x%x", can_frame.id);
        sockSend->send_to(boost::asio::buffer(sendData), remote_ep_);
        usleep(1000000);
    }
}

UdpToCan::~UdpToCan() {}

/**
 * @brief 解析udp数据
 *
 * @param buf udp缓冲区数据
 * @param len 数据长度
 */
void UdpToCan::messageAnalysis(const void *buf_data, size_t len) {
    uint8_t buf_[len] = { 0 };
    memcpy(&buf_, buf_data, len);

    size_t can_frame_size = 0;
    if(len % 13 == 0) {
        can_frame_size = len / 13;
    }
    else {
        LOG_F(WARNING, "buf length: %ld", len);
        return;
    }
    LOG_F(INFO, "can_size: %ld，long： %ld", can_frame_size, len);

    uint8_t *buf = buf_;
    for(size_t i = 0; i < can_frame_size; i++) {
        buf += i * 13;
        LOG_F(INFO, "buf[0]: %x", buf[0]);
        std::bitset<8> b(buf[1]);
        std::cout << b << std::endl;   // 二进制表示
        if((buf[0] & 0x30) == 0x00) {  // 四五RESVD位必须为0
            can_msgs canFrame;
            convertUdp2Can(buf, &canFrame);
            canAnalysis(&canFrame);
        }
        else {
            LOG_F(WARNING, "received data, but not can frame!");
        }
    }
}

/**
 * @brief 分解can帧
 *
 * @param data udp数据
 * @param can_message  can帧
 */
void UdpToCan::convertUdp2Can(const uint8_t *data, can_msgs *can_message) {
    can_message->data_len = data[0] & 0x0f;  // &0x0f取低四位

    Uint32_bytes id{};
    id.bytes[0]     = data[4];
    id.bytes[1]     = data[3];
    id.bytes[2]     = data[2];
    id.bytes[3]     = data[1];
    can_message->id = id.data;

    can_message->is_rtr      = data[0] & 0x40;  // 数据帧|远程帧
    can_message->is_extended = data[0] & 0x80;  // 扩展帧

    for(uint8_t i = 0; i < can_message->data_len; i++) {
        can_message->data[i] = data[i + 5];
    }
}