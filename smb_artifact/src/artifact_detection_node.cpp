#include "ros/ros.h"
#include "std_msgs/String.h"
#include "object_detection_msgs/ObjectDetectionInfoArray.h"

#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <fstream>

class ArtifactDetector {
public:
  ArtifactDetector(ros::NodeHandle n, ros::NodeHandle np) : nh_(n), nh_priv_(np) {
    sub_ = n.subscribe("/object_detector/detection_info", 1000, &ArtifactDetector::detectionCallback, this);
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(tf_buffer_);


    std::string filename="object_positions_log.csv";
    // filename = nh_.getParam("artifact_file", filename);
    nh_priv_.param<std::string>("artifact_file", filename, filename);
    std::cout << "writing to file: " << filename << "\n";
    log_file.open(filename);
    if (!log_file.is_open()) {
        // ROS_ERROR("Can't open log log_file: %s" , filename);
        std::cout << "cant open log file: " << filename << "\n";
    }
  } 

  ~ArtifactDetector() {
    log_file.close();
  }

void detectionCallback(const object_detection_msgs::ObjectDetectionInfoArray::ConstPtr& msg)
{
  // ROS_INFO("I heard a detection");

  // std_msgs/Header header
  // ObjectDetectionInfo[] info

  // string class_id
  // int32 id 
  // # position in 3D
  // geometry_msgs/Point position

  for (size_t i = 0; i < msg->info.size(); ++i)
  {
    auto info = msg->info[i];
    // ROS_INFO("Element[%lu]: %f", i, msg->info[i]);
    std::cout << info.class_id << '\n';
    std::string s = info.class_id;
    // ROS_INFO("Element[%lu]: %s, %i, (%f,%f,%f)", i, s, info.id, info.position.x, info.position.y, info.position.z);

    // auto transform = tf_buffer_.lookupTransform("world_graph_msf", msg->header.frame_id, msg->stamp, ros::Duration(0.5));

    // tf_listener_.waitForTransform("world_graph_msf", msg->header.frame_id, msg->stamp, ros::Duration(3.0));
        
    // // Perform the transformation
    // geometry_msgs::PoseStamped transformed_point;
    // tf_listener_.transformPose("world_graph_msf", *info->position, transformed_point);

    geometry_msgs::PointStamped transformed_point;
    geometry_msgs::PointStamped point;
    point.header = msg->header;
    point.point = info.position;

    try
    {
      // Lookup the transform from source_frame to target_frame
      geometry_msgs::TransformStamped transform_stamped = tf_buffer_.lookupTransform("world_graph_msf", msg->header.frame_id, ros::Time(0));

      // Transform the pose
      tf2::doTransform(point, transformed_point, transform_stamped);

      ROS_INFO("Transformed Pose: (%.2f, %.2f, %.2f) in frame %s",
               transformed_point.point.x, transformed_point.point.y, transformed_point.point.z,
               transformed_point.header.frame_id.c_str());

      log_file << info.id << ',' << transformed_point.point.x<< ',' << transformed_point.point.y << ','<< transformed_point.point.z << '\n';
    }
    catch (tf2::TransformException &ex)
    {
      ROS_ERROR("Transform error: %s", ex.what());
    }
  }

}


private:
  ros::NodeHandle nh_;
  ros::NodeHandle nh_priv_;
  ros::Subscriber sub_;
  tf2_ros::Buffer tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
  std::ofstream log_file;
};


int main(int argc, char **argv)
{
  ros::init(argc, argv, "artifact_detection_node");
  ros::NodeHandle n;
  ros::NodeHandle np("~");

  ArtifactDetector detector(n, np);

  ros::spin();

  return 0;
}
