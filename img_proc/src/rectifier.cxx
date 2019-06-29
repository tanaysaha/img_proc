#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

cv::Mat image;

ros::Publisher img_pub;

std::vector<double> cam_mat; //= {414.1573058656413, 0, 379.5051916771028, 0, 413.8961901608803, 260.064272132189, 0, 0, 1};
std::vector<double> dist_coeffs; //= {-0.2667497715010867, 0.05824595745054147, 0.001097211085107507, -0.001283170962460823, 0};

// void paramCallback(const sensor_msgs::CameraInfo cam_info){
//     for(int i=0;i<5;i++)
//         dist_coeffs.push_back(cam_info.D.at(i));
//     for(int i=0;i<9;i++)
//         cam_mat.push_back(cam_info.K[i]);
// }

void imgCallback(const sensor_msgs::ImageConstPtr& img){
    if(cam_mat.empty())
        return;
    
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
    }
    
    image = cv_ptr->image;

    int tempIdx=0;
    cv::Mat intrinsic = cv::Mat_<double>(3, 3);
    cv::Mat dist_coeff_ = cv::Mat_<double>(1,5);
    
    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            intrinsic.at<double>(i, j) = cam_mat.at(tempIdx++);
        }
    }

    for(int i=0;i<dist_coeffs.size();i++)
    {
        dist_coeff_.at<double>(i) = dist_coeffs[i];
    }

    cv::Mat img_;
    cv::undistort(image, img_, intrinsic, dist_coeff_);
    
    cv_bridge::CvImage rectified_img;
    rectified_img.encoding = sensor_msgs::image_encodings::BGR8;
    rectified_img.header.stamp = ros::Time::now();
    rectified_img.image = img_;
    img_pub.publish(rectified_img.toImageMsg());
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "rectifier");
    ros::NodeHandle nh;
    ros::Rate rate(20);
    img_pub = nh.advertise<sensor_msgs::Image>("rectified_image", 1);
    // ros::Subscriber param_sub = nh.subscribe("usb_cam/camera_info", 1, &paramCallback);
    nh.getParam("hdetect/distortion_coefficients/data", dist_coeffs);
    nh.getParam("hdetect/camera_matrix/data", cam_mat);
    ros::Subscriber raw_img_sub = nh.subscribe("usb_cam/image_raw", 1, &imgCallback);
    while(ros::ok())
    {
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}