#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud2.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "tracking.h"

int detect_x  = -1;
int detect_y = -1;

cf_instance cf;

ros::Publisher imageThPublisher;

void pointsCb(const sensor_msgs::PointCloud2ConstPtr& msg)
{
  if (cf.found) {
    int pos = cf.x*msg->point_step + cf.y*msg->row_step;
    float x = *((float*)&msg->data[pos + msg->fields[0].offset]);
    float y = *((float*)&msg->data[pos + msg->fields[1].offset]);
    float z = *((float*)&msg->data[pos + msg->fields[2].offset]);

    ROS_ERROR("Crazyflie detected at : %f %f (%f, %f, %f)", cf.x, cf.y, x, y, z);
  }
}

void imageCb(const sensor_msgs::ImageConstPtr& msg)
{
  cv_bridge::CvImagePtr cv_ptr;
  try
  {
    cv_ptr = cv_bridge::toCvCopy(msg, "16UC1");
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  cv::Mat ir_image = cv_ptr->image;
  cv::Mat th_display(ir_image.size(), CV_8UC3, cv::Scalar(255, 255, 255));

  cv::namedWindow("TH");
  detect_cfs(&ir_image, &th_display, &cf);

  cv_bridge::CvImage cv_image;
  cv_image.image = th_display;
  cv_image.encoding = "rgb8";
  imageThPublisher.publish(cv_image.toImageMsg());
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "detector");
  ros::NodeHandle n;

  ros::Subscriber pointSub = n.subscribe("/kinect2/sd/points", 1, pointsCb);
  ros::Subscriber imageSub = n.subscribe("/kinect2/sd/image_ir", 1, imageCb);

  imageThPublisher = n.advertise<sensor_msgs::Image>("/detector/threshold", 1);

  ros::spin();
  return 0;
}
