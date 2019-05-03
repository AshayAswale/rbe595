#include <stdlib.h>
#include <stdio.h>
#include <std_msgs/String.h>

#include <tough_perception_common/MultisenseImageInterface.h>
#include <tough_perception_common/PointCloudHelper.h>
#include <geometry_msgs/PointStamped.h>

#include <opencv2/core.hpp>
#include <opencv/highgui.h>
#include <pcl-1.7/pcl/common/centroid.h>
#include <pcl-1.7/pcl/point_types.h>
#include <pcl-1.7/pcl/common/common.h>
#include <pcl-1.7/pcl/filters/passthrough.h>

ros::Publisher* point_pub;

void colorSegmentation(cv::Mat& inImage, const int hsv[6], cv::Mat& outImage)
{
  cv::Mat hsvImage;
  cv::cvtColor(inImage, hsvImage, CV_BGR2HSV);
  cv::inRange(hsvImage, cv::Scalar(hsv[0], hsv[1], hsv[2]), cv::Scalar(hsv[3], hsv[4], hsv[5]), outImage);
}

void showImage(cv::Mat& image, std::string name)
{
  cv::namedWindow(name, cv::WINDOW_AUTOSIZE);
  cv::imshow(name, image);
  cv::waitKey(0);
  // ROS_INFO("Press ESC or q to continue");
  // while(cv::waitKey(1)!=27 && cv::waitKey(1)!='q');
  // ROS_INFO("closing window");
}

void findMaxContour(cv::Mat image, cv::Rect& roi, cv::Mat& colorImage, cv::Mat& filledContour)
{
  cv::Mat canny_output;
  std::vector<std::vector<cv::Point>> contours;
  std::vector<cv::Vec4i> hierarchy;
  std::vector<cv::Point> hull;
  cv::Canny(image, canny_output, 100, 200, 3);

  cv::findContours(canny_output, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
  double largestArea = 0;
  int largestContourIndex = 0;
  for (int i = 0; i < contours.size(); i++)
  {
    double area = cv::contourArea(contours[i]);
    if (area > largestArea)
    {
      largestArea = area;
      largestContourIndex = i;
    }
  }
  roi = cv::boundingRect(contours[largestContourIndex]);
  cv::Scalar color = cv::Scalar(0, 0, 255);
  cv::rectangle(colorImage, roi, color, 3);

  cv::fillPoly(canny_output, contours, cv::Scalar(255, 255, 255));
  canny_output.copyTo(filledContour);
  // showImage(canny_output, "Filled contour");
}
/*
void detectPlanes (  pcl::PointCloud<pcl::PointXYZRGB> currentDetectionCloud)
{
      pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
      pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
      pcl::SACSegmentation<pcl::PointXYZ> seg;
      seg.setModelType(pcl::SACMODEL_PLANE);
      seg.setMethodType(pcl::SAC_RANSAC);
      seg.setDistanceThreshold(0.01);

      seg.setInputCloud(cloud);
      seg.segment(*inliers, *coefficients);

      if (inliers->indices.size() == 0)
      {
        PCL_ERROR("Could not estimate a planar model for the given dataset.");
        return (-1);

      }

}
*/
void getValveLocation(cv::Mat& filledContour, tough_perception::StereoPointCloudColor::Ptr& pointCloud)
{
  // std::vector<cv::Point> indices;
  cv::Mat indices;
  cv::findNonZero(filledContour, indices);

  pcl::PointCloud<pcl::PointXYZRGB> currentDetectionCloud;

  for (int k = 0; k < indices.total(); k++)
  {
    pcl::PointXYZRGB temp_pclPoint = pointCloud->at(indices.at<cv::Point>(k).x, indices.at<cv::Point>(k).y);

    if (temp_pclPoint.z > -2.0)
    {
      currentDetectionCloud.push_back(pcl::PointXYZRGB(temp_pclPoint));
    }
  }
  /*
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>(currentDetectionCloud));
    // pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);

    pcl::PassThrough<pcl::PointXYZ> pass;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered;

    pcl::copyPointCloud(*cloud, *cloud_xyz);

    pass.setInputCloud(cloud_xyz);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(1.0, 5.0);
    pass.filter(*cloud_filtered);
  */
  Eigen::Vector4f cloudCentroid;
  pcl::compute3DCentroid(currentDetectionCloud, cloudCentroid);
  ROS_INFO_STREAM("x " << cloudCentroid(0) << " y " << cloudCentroid(1) << " z " << cloudCentroid(2));

  geometry_msgs::Point geom_point;
  geom_point.x = cloudCentroid(0);
  geom_point.y = cloudCentroid(1);
  geom_point.z = cloudCentroid(2);

  point_pub->publish(geom_point);
  // pcl::PointXYZRGB min_pt, max_pt;

  // pcl::getMinMax3D(currentDetectionCloud, min_pt, max_pt);
  // std::cout << "\nMin point - " << min_pt.x << "," << min_pt.y << "," << min_pt.z << "\n";
  // std::cout << "\nMax point - " << max_pt.x << "," << max_pt.y << "," << max_pt.z << "\n";
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "test_multisense_image");
  ros::NodeHandle nh;
  bool status;

  ros::Publisher publish_point = nh.advertise<geometry_msgs::Point>("/valve_center", 100);
  point_pub = &publish_point;

  tough_perception::MultisenseImageInterfacePtr imageHandler;
  imageHandler = tough_perception::MultisenseImageInterface::getMultisenseImageInterface(nh);  // nh is NodeHandle

  ROS_INFO_STREAM("[Height]" << imageHandler->getHeight() << " [width]" << imageHandler->getWidth());

  cv::Mat colorImage;
  cv::Mat dispImage;
  cv::Mat valveSegmentedImage;
  tough_perception::StereoPointCloudColor::Ptr pointCloud(new tough_perception::StereoPointCloudColor);
  cv::Rect roi;
  cv::Mat filledContour;  // to use with fillPoly
  status = imageHandler->getStereoData(dispImage, colorImage, pointCloud);
  ROS_INFO("image status %s", status ? "true" : "false");

  if (status)
    showImage(colorImage, "BGR Image");

  int hsv[] = { 110, 50, 50, 130, 255, 255 };  // to segment BLUE
  colorSegmentation(colorImage, hsv, valveSegmentedImage);

  cv::GaussianBlur(valveSegmentedImage, valveSegmentedImage, cv::Size(9, 9), 2, 2);
  findMaxContour(valveSegmentedImage, roi, colorImage, filledContour);
  showImage(valveSegmentedImage, "Valve Segmented Image");
  showImage(colorImage, "Valve Detection");
  showImage(filledContour, "Filled contour");

  getValveLocation(filledContour, pointCloud);

  return 0;
}