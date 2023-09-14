#include "single_kalman.h"

int main(int argc, char **argv) {
    SingleKalmanFilter singleKalmanFilter(1, 1, 1, 0, 1);
    singleKalmanFilter.setProcessNoise(0.01);
    singleKalmanFilter.setMeasurementNoise(15.4);

    double data[10] = {1,2,3,4,5,6,7,8,9,10}
    for(int i = 0; i < 10; i ++){
        double single_filter = singleKalmanFilter.filter(data[i], 0);
        std::cout << single_filter << std::endl;
    }
    return 0;
}