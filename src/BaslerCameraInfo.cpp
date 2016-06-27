#include <gentl_tof_driver/BaslerCameraInfo.h>


BaslerCameraInfo::BaslerCameraInfo()
{
  ros::NodeHandle n;
  service_ = n.advertiseService("/camera/set_camera_info", &BaslerCameraInfo::setCameraInfo, this);
  default_camera_info_ = true;
  ROS_INFO("use default camera info!");
}

bool BaslerCameraInfo::setCameraInfo(sensor_msgs::SetCameraInfo::Request &req, sensor_msgs::SetCameraInfo::Response &res)
{
    res.success = true;
    res.status_message = std::string("test");

    return true;
}

sensor_msgs::CameraInfoPtr BaslerCameraInfo::getCameraInfo(int width, int height, double f)
{
    if(default_camera_info_)
        return getDefaultCameraInfo(width, height,f);
    else
        return getSavedCameraInfo();


}

sensor_msgs::CameraInfoPtr BaslerCameraInfo::getDefaultCameraInfo(int width, int height, double f)
{
    sensor_msgs::CameraInfoPtr info = boost::make_shared<sensor_msgs::CameraInfo>();

    info->width  = width;
    info->height = height;

    // No distortion
    info->D.resize(5, 0.0);
    info->distortion_model = sensor_msgs::distortion_models::PLUMB_BOB;

    // Simple camera matrix: square pixels (fx = fy), principal point at center
    info->K.assign(0.0);
    info->K[0] = info->K[4] = f;
    info->K[2] = (width / 2) - 0.5;

    // Aspect ratio for the camera center on Kinect (and other devices?) is 4/3
    // This formula keeps the principal point the same in VGA and SXGA modes
    info->K[5] = (width * (3./8.)) - 0.5;
    info->K[8] = 1.0;

    // No separate rectified image plane, so R = I
    info->R.assign(0.0);
    info->R[0] = info->R[4] = info->R[8] = 1.0;

    // Then P=K(I|0) = (K|0)
    info->P.assign(0.0);
    info->P[0]  = info->P[5] = f; // fx, fy
    info->P[2]  = info->K[2];     // cx
    info->P[6]  = info->K[5];     // cy
    info->P[10] = 1.0;

    return info;
}

sensor_msgs::CameraInfoPtr BaslerCameraInfo::getSavedCameraInfo()
{
    sensor_msgs::CameraInfoPtr info = boost::make_shared<sensor_msgs::CameraInfo>();

     info->width  = 640;
     info->height = 460;

     // distortion
     info->D.resize(5, 0.0);
     info->D[0] = -0.528319;
     info->D[1] = 0.326522;
     info->D[2] = -0.014408;
     info->D[3] = 0.014202;
     info->D[4] = 0.0;
     info->distortion_model = sensor_msgs::distortion_models::PLUMB_BOB;

     // camera matrix
     info->K.assign(0.0);
     info->K[0] = 560.474024;
     info->K[4] = 558.132830;
     info->K[2] = 302.414381;
     info->K[4] = 558.132830;
     info->K[5] = 239.108094;
     info->K[8] = 1.0;

     // No separate rectified image plane, so R = I
     info->R.assign(0.0);
     info->R[0] = info->R[4] = info->R[8] = 1.0;

     // projection matrix
     info->P.assign(0.0);
     info->P[0]  = 460.870453;
     info->P[2]  = 311.568259;
     info->P[5]  = 498.076355;
     info->P[6]  = 231.496420;
     info->P[10] = 1.0;

     return info;

}
