#include "udp_to_can.hpp"

int main(int argc, char const *argv[]) {
    UdpToCan udpToCan(8889);
    udpToCan.start("192.168.89.92", 8888);
    // udpToCan.start("127.0.0.1", 8888);


    while(1) {
        udpToCan.receiveData();
    }
    return 0;
}
