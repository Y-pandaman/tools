#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>

static const std::string OPENCV_WINDOW = "Image window";
int g_fps = 30.0;

cv::VideoWriter
    writer("/home/bdca/workspace/test_ws/src/ros_video_save/data/write.avi",
           CV_FOURCC('M', 'J', 'P', 'G'), g_fps, cv::Size(1280, 720));

void imageCb(const sensor_msgs::ImageConstPtr &msg) {
    cv_bridge::CvImagePtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    } catch (cv_bridge::Exception &e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    writer << cv_ptr->image;
    std::cout << "video is saving to "
                 "/home/bdca/workspace/test_ws/src/ros_video_save/data/data/"
                 "write.avi ..."
              << std::endl;
    // Draw an example circle on the video stream
    if (cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60)
        cv::circle(cv_ptr->image, cv::Point(50, 50), 10, CV_RGB(255, 0, 0));

    // Update GUI Window
    cv::imshow(OPENCV_WINDOW, cv_ptr->image);
    cv::waitKey(3);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "image_converter");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    image_transport::Subscriber image_sub;
    image_sub = it.subscribe("/usb_cam/image_raw", 1, imageCb);

    // ImageConverter ic;
    ros::Rate loop_rate(30);
    while (ros::ok()) {

        loop_rate.sleep();
        ros::spinOnce();
    }
    return 0;
}
