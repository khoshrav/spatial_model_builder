#include "yolo_ros.h"
#include <fstream>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <geometry_msgs/Point.h>
#include <tf/transform_listener.h>
#include <vector>
#include <iostream>
#include <pthread.h>
#include <std_msgs/Int8.h>
#include <math.h>
#include <darknet_ros/bbox_array.h>
#include <darknet_ros/bbox.h>

#include <geometry_msgs/PointStamped.h>

extern "C" {
  #include "box.h"
}

// initialize YOLO functions that are called in this script
//ROS_box *run_yolo();
PredBox *run_yolo();
void load_net(char *cfgfile, char *weightfile, float thresh, float hier);
int get_obj_count();

// define demo_yolo inputs
char *cfg;
char *weights;
char *labels;
float thresh = 0.5;
std::string *class_labels;
int num_classes;
// const std::string class_labels[] = { "aeroplane", "bicycle", "bird", "boat", "bottle", "bus", "car", "cat",
// 		     	             "chair", "cow", "dining table", "dog", "horse", "motorbike", "person",
// 		                     "potted plant", "sheep", "sofa", "train", "tv monitor" };
// const int num_classes = sizeof(class_labels)/sizeof(class_labels[0]);

cv::Mat input_image;

// define parameters
const std::string CAMERA_TOPIC_NAME = "/camera/image_raw";
const std::string CAMERA_INFO_TOPIC = "/camera/camera_info";
const std::string CAMERA_POINTS_TOPIC = "/camera/points";
// const std::string CAMERA_WIDTH_PARAM = "/usb_cam/image_width";
// const std::string CAMERA_HEIGHT_PARAM = "/usb_cam/image_height";
const std::string OPENCV_WINDOW = "YOLO object detection";
int FRAME_W;
int FRAME_H;
int FRAME_AREA;
int FRAME_COUNT = 0;

// define a function that will replace CvVideoCapture.
// This function is called in yolo_kernels and allows YOLO to receive the ROS image
// message as an IplImage
IplImage* get_Ipl_image()
{
   IplImage* ROS_img = new IplImage(input_image);
   return ROS_img;
}

class yoloObjectDetector
{
   ros::NodeHandle _nh;
   image_transport::ImageTransport _it;
   image_transport::Subscriber _image_sub;
   ros::Subscriber _points_sub;
   ros::Subscriber _camera_info_sub;
   ros::Publisher _found_object_pub;
   ros::Publisher _bboxes_pub;
   tf::TransformListener tf_listener;
   //std::vector< std::vector<PredBox> > _class_bboxes;
   //std::vector<int> _class_obj_count;
   std::vector<cv::Scalar> _bbox_colors;
   darknet_ros::bbox_array _bbox_results_msg;
   PredBox* _boxes;
   pcl::PointCloud<pcl::PointXYZ> point_cloud;

public:
   yoloObjectDetector() : _it(_nh), _bbox_colors(num_classes), point_cloud(640, 480, pcl::PointXYZ())
   {
     std::ifstream label_file(labels);
     label_file.unsetf(std::ios_base::skipws);

     // count the newlines with an algorithm specialized for counting:
     unsigned line_count = std::count(
         std::istream_iterator<char>(label_file),
         std::istream_iterator<char>(),
         '\n');
         label_file.clear();
         label_file.seekg(0, std::ios::beg);
        class_labels = new std::string[line_count];
        num_classes = line_count;
        int a = 0;
        while(!label_file.eof())
        {

          std::string temp;
          getline(label_file,temp);
          if(!temp.empty())
          {
            class_labels[a] = temp;
            a++;
          }
          }
        ROS_INFO("Got labels!");
        int incr = floor(255/num_classes);
      for (int i = 0; i < num_classes; i++)
      {
         _bbox_colors.push_back(cv::Scalar(255 - incr*i, 0 + incr*i, 255 - incr*i));
      }
      _camera_info_sub = _nh.subscribe<sensor_msgs::CameraInfo>(CAMERA_INFO_TOPIC, 1,
                                                              &yoloObjectDetector::cameraInfoCallback, this);
      _image_sub = _it.subscribe(CAMERA_TOPIC_NAME, 1,
	                       &yoloObjectDetector::cameraCallback,this);
     _points_sub =_nh.subscribe(CAMERA_POINTS_TOPIC, 1,
                         &yoloObjectDetector::pointsCallback, this);
      _found_object_pub = _nh.advertise<std_msgs::Int8>("found_object", 1);
      _bboxes_pub = _nh.advertise<darknet_ros::bbox_array>("YOLO_bboxes", 1);

      cv::namedWindow(OPENCV_WINDOW, cv::WINDOW_NORMAL);
   }

   ~yoloObjectDetector()
   {
      cv::destroyWindow(OPENCV_WINDOW);
      delete[] class_labels;
   }

private:
   void drawBBoxes(cv::Mat &input_frame, std::vector<PredBox> &class_boxes, int &class_obj_count,
		   cv::Scalar &bbox_color, const std::string &class_label)
   {
      darknet_ros::bbox bbox_result;

      for (int i = 0; i < class_obj_count; i++)
      {
         int xmin = (class_boxes[i].x - class_boxes[i].w/2)*FRAME_W;
         int ymin = (class_boxes[i].y - class_boxes[i].h/2)*FRAME_H;
         int xmax = (class_boxes[i].x + class_boxes[i].w/2)*FRAME_W;
         int ymax = (class_boxes[i].y + class_boxes[i].h/2)*FRAME_H;
         bbox_result.Class = class_label;
         bbox_result.prob = class_boxes[i].prob;
         bbox_result.frame = "/head_base_link";
         double x_center = static_cast<double>(xmin) + (static_cast<double>(xmax-xmin)/2);
         double y_center = static_cast<double>(ymin) + (static_cast<double>(ymax-ymin)/2);

         pcl::PointXYZ point = point_cloud.at(x_center, y_center);
         try
       {
           geometry_msgs::PointStamped point_src;
           point_src.header.frame_id = "kinect_openni_frame";

           //we'll just use the most recent transform available for our simple example
           point_src.header.stamp = ros::Time(0);

           //just an arbitrary point in space
           point_src.point.x = point.x;
           point_src.point.y = point.y;
           point_src.point.z = point.z;
           geometry_msgs::PointStamped point_des;
           tf_listener.waitForTransform("world",
              point_src.header.frame_id,
              point_src.header.stamp,
              ros::Duration(3.0));
           tf_listener.transformPoint("world", point_src, point_des);
           bbox_result.x = point_des.point.x;
           bbox_result.y = point_des.point.y;
           bbox_result.z = point_des.point.z;
           _bbox_results_msg.bboxes.push_back(bbox_result);
         }
         catch(tf::TransformException& ex){
           ROS_ERROR("Received an exception trying to transform a point from \"kinect_openni_frame\" to \"head_base_link\": %s", ex.what());
         }


         // draw bounding box of first object found
         cv::Point topLeftCorner = cv::Point(xmin, ymin);
         cv::Point botRightCorner = cv::Point(xmax, ymax);
	 cv::rectangle(input_frame, topLeftCorner, botRightCorner, bbox_color, 2);
         cv::putText(input_frame, class_label, cv::Point(xmin, ymax+15), cv::FONT_HERSHEY_PLAIN,
		 1.0, bbox_color, 2.0);
      }
   }

   void runYOLO(cv::Mat &full_frame)
   {
      cv::Mat input_frame = full_frame.clone();
      std::vector< std::vector<PredBox> > class_bboxes(num_classes);
      std::vector<int> class_obj_count(num_classes, 0);

      // run yolo and get bounding boxes for objects
      _boxes = run_yolo();

      // get the number of bounding boxes found
      int num = get_obj_count(); //_boxes[0].num;

      // if at least one bbox found, draw box
      if (num > 0  && num <= 100)
      {
	 std::cout << "# Objects: " << num << std::endl;

	 // split bounding boxes by class
         for (int i = 0; i < num; i++)
         {
            for (int j = 0; j < num_classes; j++)
            {
               if (_boxes[i].Class == j)
               {
                  class_bboxes[j].push_back(_boxes[i]);
                  class_obj_count[j]++;
               }
            }
         }

	 // send message that an object has been detected
         std_msgs::Int8 msg;
         msg.data = 1;
         _found_object_pub.publish(msg);

         for (int i = 0; i < num_classes; i++)
         {
            if (class_obj_count[i] > 0) drawBBoxes(input_frame, class_bboxes[i],
					      class_obj_count[i], _bbox_colors[i], class_labels[i]);
         }
         _bboxes_pub.publish(_bbox_results_msg);
         _bbox_results_msg.bboxes.clear();
      }
      else
      {
          std_msgs::Int8 msg;
          msg.data = 0;
          _found_object_pub.publish(msg);
      }

      //for (int i = 0; i < num_classes; i++)
      //{
      //   _class_bboxes[i].clear();
      //   _class_obj_count[i] = 0;
      //}

      cv::imshow(OPENCV_WINDOW, input_frame);
      cv::waitKey(3);
   }
   void cameraInfoCallback(const sensor_msgs::CameraInfoConstPtr& msg)
   {
      FRAME_W =  msg->width;
      FRAME_H = msg->height;
   }
   void cameraCallback(const sensor_msgs::ImageConstPtr& msg)
   {
      std::cout << "usb image received" << std::endl;

      cv_bridge::CvImagePtr cam_image;

      try
      {
         cam_image = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
      }
      catch (cv_bridge::Exception& e)
      {
         ROS_ERROR("cv_bridge exception: %s", e.what());
	 return;
      }

      if (cam_image)
      {
         input_image = cam_image->image.clone();

	 if (FRAME_COUNT == 0) {
            runYOLO(cam_image->image);
         }
	 //FRAME_COUNT++;
	 if (FRAME_COUNT == 1) FRAME_COUNT = 0;
      }
      return;
   }

   void pointsCallback(const sensor_msgs::PointCloud2ConstPtr& msg)
   {
      pcl::fromROSMsg(*msg, point_cloud);
   }
};

int main(int argc, char** argv)
{
   ros::init(argc, argv, "ROS_interface");

   // ros::param::get(CAMERA_WIDTH_PARAM, FRAME_W);
   // ros::param::get(CAMERA_HEIGHT_PARAM, FRAME_H);
   ros::NodeHandle nh;
   std::string configFilePath;
   if (!nh.getParam("/darknet_ros/darknet_config", configFilePath))
   {

     configFilePath = "/home/khoshrav/three_dof_head/darknet_ros/config/yolo.cfg";
     ROS_WARN("Could not retrieve 'darknet_config' parameter, setting to default value: '%s' \n",
              configFilePath.c_str());

   }
   cfg = new char[configFilePath.size() + 1];
    std::copy(configFilePath.begin(), configFilePath.end(), cfg);
    cfg[configFilePath.size()] = '\0'; // don't forget the terminating 0

    std::string weightFilePath;
    if (!nh.getParam("/darknet_ros/darknet_weights", weightFilePath))
    {

      weightFilePath = "/home/khoshrav/three_dof_head/darknet_ros/config/yolo.weights";
      ROS_WARN("Could not retrieve 'darknet_config' parameter, setting to default value: '%s' \n",
               weightFilePath.c_str());

    }
    weights = new char[weightFilePath.size() + 1];
     std::copy(weightFilePath.begin(), weightFilePath.end(), weights);
     weights[weightFilePath.size()] = '\0'; // don't forget the terminating 0

  std::string labelFilePath;
  if (!nh.getParam("/darknet_ros/darknet_labels", labelFilePath))
  {

   labelFilePath = "/home/khoshrav/three_dof_head/darknet_ros/config/coco.names";
   ROS_WARN("Could not retrieve 'darknet_config' parameter, setting to default value: '%s' \n",
            labelFilePath.c_str());

  }
   labels = new char[labelFilePath.size() + 1];
   std::copy(labelFilePath.begin(), labelFilePath.end(), labels);
   labels[labelFilePath.size()] = '\0'; // don't forget the terminating 0

   load_net(cfg, weights, thresh, 0.5);
   yoloObjectDetector yod;
   ros::spin();
   delete[] cfg;
   delete[] weights;
   delete[] labels;
   return 0;
}
