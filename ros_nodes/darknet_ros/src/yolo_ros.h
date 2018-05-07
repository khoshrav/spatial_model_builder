#ifndef YOLO_ROS
#define YOLO_ROS

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/objdetect/objdetect.hpp>
// include pcl stuff
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/common/transforms.h>
#include <pcl/common/eigen.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>

#include "cuda_runtime.h"
#include "curand.h"
#include "cublas_v2.h"

IplImage* get_Ipl_image();
/*
typedef struct {
  float x, y, w, h;
  int num, Class;
} ROS_box;
*/
typedef struct {
  float x, y, w, h, prob;
  int Class;
} PredBox;
// class yoloObjectDetector
// {
// std::string *class_labels;
// int num_classes;
// };
#endif
