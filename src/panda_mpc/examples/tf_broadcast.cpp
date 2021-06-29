#include <ros/ros.h>
#include <tf2_msgs/TFMessage.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2/LinearMath/Quaternion.h>


class CameraWorldTf
{
private:
  double x_, y_, z_, roll_, pitch_, yaw_;
  ros::NodeHandle nh_;
  ros::Subscriber tf_listener_;

public:

  CameraWorldTf(ros::NodeHandle& n);

  void tfCallBack(const tf2_msgs::TFMessage& tf_msgs);
};


CameraWorldTf::CameraWorldTf(ros::NodeHandle& n): nh_(n){

  tf_listener_ = nh_.subscribe("tf",10,&CameraWorldTf::tfCallBack,this);
}

void CameraWorldTf::tfCallBack(const tf2_msgs::TFMessage &tf_msgs){

  static tf2_ros::TransformBroadcaster br;
  geometry_msgs::TransformStamped transformStamped;
  transformStamped.header.stamp = ros::Time::now();
  transformStamped.header.frame_id = "panda_link0";
  transformStamped.child_frame_id = "camera_rgb_optical_frame";
  transformStamped.transform.translation.x = 0.4;
  transformStamped.transform.translation.y = 1.;
  transformStamped.transform.translation.z = 0.4;

  tf2::Quaternion q;
  q.setRPY(-1.8,0 , -2.61);
  transformStamped.transform.rotation.x = q.x();
  transformStamped.transform.rotation.y = q.y();
  transformStamped.transform.rotation.z = q.z();
  transformStamped.transform.rotation.w = q.w();

  br.sendTransform(transformStamped);

}

int main(int argc, char** argv){
  ros::init(argc,argv, "tf_camera_robot");
  ros::NodeHandle n;

  CameraWorldTf camera_world_tf(n);
  ros::spin();
  return 0;
}
