#include <stdlib.h>
#include <stdio.h>
#include <std_msgs/String.h>
#include <tough_perception_common/MultisenseImage.h>
#include <tough_perception_common/MultisenseImageInterface.h>
#include <tough_perception_common/PointCloudHelper.h>
#include <opencv2/core.hpp>
#include <opencv/highgui.h>

void colorSegmentation(cv::Mat &inImage, const int hsv [6], cv::Mat &outImage)
{   
    cv::Mat hsvImage;
    cv::cvtColor(inImage, hsvImage, CV_RGB2HSV);
    cv::inRange(inImage, cv::Scalar(hsv[0],hsv[1],hsv[2]), cv::Scalar(hsv[3],hsv[4],hsv[5]), outImage);
}

void showImage(cv::Mat &image,std::string name)
{
    cv::namedWindow(name,cv::WINDOW_AUTOSIZE);
    cv::imshow(name,image);
    ROS_INFO("Press ESC or q to continue");
    while(cv::waitKey(1)!=27 && cv::waitKey(1)!='q');
    ROS_INFO("closing window");
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "test_multisense_image");
    ros::NodeHandle nh;
    bool status;

    tough_perception::MultisenseImageInterfacePtr imageHandler;
    imageHandler = tough_perception::MultisenseImageInterface::getMultisenseImageInterface(nh); //nh is NodeHandle

    tough_perception::StereoPointCloudColor::Ptr organized_cloud(new tough_perception::StereoPointCloudColor);
    cv::Mat color;
    cv::Mat_<float> disp;
    cv::Mat_<double> Q;
    cv::Mat image;
    //tough_perception::MultisenseCameraModel Q;

    bool valid_Q = false;
    bool new_color = false;
    bool new_disp = false;

        
    while(ros::ok())
    {
        //std::cout<<"In\n";

        if (imageHandler->getImage(color))
        {
            new_color = true;
        }

        if (imageHandler->getDisparity(disp))
        {
            new_disp = true;
        }

        if (new_color && new_disp)
        {

            std::cout << disp.cols << " x " << disp.rows << std::endl;    
            tough_perception::PointCloudHelper::generateOrganizedRGBDCloud(disp, color, Q, organized_cloud);

            //ROS_INFO_STREAM("Organized cloud size: "<< organized_cloud->size());
            //pcl::PCLPointCloud2 output;
            //pcl::toPCLPointCloud2(*organized_cloud, output);
            int hsv[] = {110, 50, 50, 130, 255, 255};
            colorSegmentation(color, hsv, image);

            if(new_color)
            showImage(color,"RGB Image");

            new_disp= new_color = false;
        }
        ros::spinOnce();
    }

}