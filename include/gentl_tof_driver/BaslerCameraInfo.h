#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/SetCameraInfo.h>
#include <sensor_msgs/distortion_models.h>


class BaslerCameraInfo{
private:
    ros::ServiceServer service_;
    bool default_camera_info_;

    sensor_msgs::CameraInfoPtr getDefaultCameraInfo(int width, int height, double f);
    sensor_msgs::CameraInfoPtr getSavedCameraInfo();

public:
  BaslerCameraInfo();
  bool setCameraInfo(sensor_msgs::SetCameraInfo::Request &req, sensor_msgs::SetCameraInfo::Response &res);
  sensor_msgs::CameraInfoPtr getCameraInfo(int width, int height, double f);
};
