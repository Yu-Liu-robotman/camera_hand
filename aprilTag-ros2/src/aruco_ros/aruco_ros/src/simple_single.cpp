/*****************************
 Copyright 2011 Rafael Muñoz Salinas. All rights reserved.

 Redistribution and use in source and binary forms, with or without modification, are
 permitted provided that the following conditions are met:

 1. Redistributions of source code must retain the above copyright notice, this list of
 conditions and the following disclaimer.

 2. Redistributions in binary form must reproduce the above copyright notice, this list
 of conditions and the following disclaimer in the documentation and/or other materials
 provided with the distribution.

 THIS SOFTWARE IS PROVIDED BY Rafael Muñoz Salinas ''AS IS'' AND ANY EXPRESS OR IMPLIED
 WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND
 FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL Rafael Muñoz Salinas OR
 CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
 ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

 The views and conclusions contained in the software and documentation are those of the
 authors and should not be interpreted as representing official policies, either expressed
 or implied, of Rafael Muñoz Salinas.
 ********************************/
/**
 * @file simple_single.cpp
 * @author Bence Magyar
 * @date June 2012
 * @version 0.1
 * @brief ROS version of the example named "simple" in the ArUco software package.
 */

#include <iostream>

#include "aruco/aruco.h"
#include "aruco/cvdrawingutils.h"
#include "aruco_ros/aruco_ros_utils.hpp"

#if __has_include("cv_bridge/cv_bridge.hpp")
#include "cv_bridge/cv_bridge.hpp"
#else
#include "cv_bridge/cv_bridge.h"
#endif
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/vector3_stamped.hpp"
#include "image_transport/image_transport.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rcpputils/asserts.hpp"
#include "sensor_msgs/image_encodings.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "visualization_msgs/msg/marker.hpp"

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core.hpp>
#include <cmath>

class ArucoSimple : public rclcpp::Node
{
private:
  rclcpp::Node::SharedPtr subNode;
  cv::Mat inImage;
  aruco::CameraParameters camParam;
  tf2::Stamped<tf2::Transform> rightToLeft;
  bool useRectifiedImages;
  aruco::MarkerDetector mDetector;
  std::vector<aruco::Marker> markers;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr cam_info_sub;
  bool cam_info_received;
  image_transport::Publisher image_pub;
  image_transport::Publisher debug_pub;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub;
  rclcpp::Publisher<geometry_msgs::msg::TransformStamped>::SharedPtr transform_pub;
  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr position_pub;
  // rviz visualization marker
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr pixel_pub;
  std::string marker_frame;
  std::string camera_frame;
  std::string reference_frame;
  // 刘雨设置
  std::string marker_dictionary;
  float error_correction_rate;
  bool pose_filter_initialized = false;
  geometry_msgs::msg::Pose prev_filtered_pose;
  double alpha = 0.2; // 0.01~0.3 常用范围；越小越平滑（响应越慢）

  double marker_size;
  int marker_id;

  std::unique_ptr<image_transport::ImageTransport> it_;
  image_transport::Subscriber image_sub;

  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

public:
  ArucoSimple()
  : Node("aruco_single"), cam_info_received(false)
  {
  }
  tf2::Quaternion quat_slerp(const tf2::Quaternion &q1, const tf2::Quaternion &q2, double t)
  {
      // 计算点积
      double x1 = q1.x(), y1 = q1.y(), z1 = q1.z(), w1 = q1.w();
      double x2 = q2.x(), y2 = q2.y(), z2 = q2.z(), w2 = q2.w();
      double dot = x1*x2 + y1*y2 + z1*z2 + w1*w2;

      // 如果 dot < 0，取补（以保证较短弧）
      tf2::Quaternion q2copy = q2;
      if (dot < 0.0) {
          dot = -dot;
          q2copy = tf2::Quaternion(-x2, -y2, -z2, -w2);
      }

      const double DOT_THRESHOLD = 0.9995;
      if (dot > DOT_THRESHOLD) {
          // 角度很小 -> 线性插值并归一化（避免除以很小的 sin）
          tf2::Quaternion result(
              (1.0 - t) * x1 + t * q2copy.x(),
              (1.0 - t) * y1 + t * q2copy.y(),
              (1.0 - t) * z1 + t * q2copy.z(),
              (1.0 - t) * w1 + t * q2copy.w()
          );
          result.normalize();
          return result;
      }

      // 标准 SLERP
      double theta_0 = std::acos(dot);        // angle between inputs
      double sin_theta_0 = std::sin(theta_0);
      double theta = theta_0 * t;             // angle for interpolation
      double sin_theta = std::sin(theta);

      double s0 = std::cos(theta) - dot * sin_theta / sin_theta_0;
      double s1 = sin_theta / sin_theta_0;

      tf2::Quaternion result(
          s0 * x1 + s1 * q2copy.x(),
          s0 * y1 + s1 * q2copy.y(),
          s0 * z1 + s1 * q2copy.z(),
          s0 * w1 + s1 * q2copy.w()
      );
      result.normalize();
      return result;
  }

  // 对 pose 做一次滤波并返回过滤后的 pose（调用位置处使用）
  geometry_msgs::msg::Pose apply_pose_filter(const geometry_msgs::msg::Pose &new_pose)
  {
      if (!pose_filter_initialized) {
          prev_filtered_pose = new_pose;
          pose_filter_initialized = true;
          return prev_filtered_pose;
      }

      geometry_msgs::msg::Pose out = prev_filtered_pose;

      // 平滑位置 (EMA)
      out.position.x = (1.0 - alpha) * prev_filtered_pose.position.x + alpha * new_pose.position.x;
      out.position.y = (1.0 - alpha) * prev_filtered_pose.position.y + alpha * new_pose.position.y;
      out.position.z = (1.0 - alpha) * prev_filtered_pose.position.z + alpha * new_pose.position.z;

      // 平滑方向 (SLERP)
      tf2::Quaternion q_prev, q_new, q_f;
      tf2::fromMsg(prev_filtered_pose.orientation, q_prev);
      tf2::fromMsg(new_pose.orientation, q_new);

      q_prev.normalize();
      q_new.normalize();

      q_f = quat_slerp(q_prev, q_new, alpha);
      out.orientation = tf2::toMsg(q_f);

      // 更新状态
      prev_filtered_pose = out;
      return out;
  }

  bool setup()
  {
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    subNode = this->create_sub_node(this->get_name());

    it_ = std::make_unique<image_transport::ImageTransport>(shared_from_this());
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);
    if (this->has_parameter("corner_refinement")) {
      RCLCPP_WARN(
        this->get_logger(),
        "Corner refinement options have been removed in ArUco 3.0.0, "
        "corner_refinement ROS parameter is deprecated");
    }

    aruco::MarkerDetector::Params params = mDetector.getParameters();
    std::string thresh_method;
    // params.setThresholdMethod(aruco::MarkerDetector::THRES_ADAPTIVE, -1, 5, 3);

    switch (params.thresMethod) {
      case aruco::MarkerDetector::ThresMethod::THRES_ADAPTIVE:
        thresh_method = "THRESH_ADAPTIVE";
        break;
      case aruco::MarkerDetector::ThresMethod::THRES_AUTO_FIXED:
        thresh_method = "THRESH_AUTO_FIXED";
        break;
      default:
        thresh_method = "UNKNOWN";
        break;
    }
    
    // Print parameters of ArUco marker detector:
    RCLCPP_INFO_STREAM(this->get_logger(), "Threshold method: " << thresh_method);

    // Declare node parameters
    this->declare_parameter<double>("marker_size", 0.05);
    this->declare_parameter<int>("marker_id", 300);
    this->declare_parameter<std::string>("reference_frame", "");
    this->declare_parameter<std::string>("camera_frame", "");
    this->declare_parameter<std::string>("marker_frame", "");
    this->declare_parameter<bool>("image_is_rectified", true);
    this->declare_parameter<float>("min_marker_size", 0.01);
    this->declare_parameter<std::string>("detection_mode", "");
    this->declare_parameter<std::string>("marker_dictionary", "ARUCO_MIP_36h12"); //ARUCO 
    this->declare_parameter<float>("error_correction_rate", 0.1f);

    float min_marker_size;  // percentage of image area
    this->get_parameter_or<float>("min_marker_size", min_marker_size, 0.01);

    std::string detection_mode;
    this->get_parameter_or<std::string>("detection_mode", detection_mode, "DM_FAST");
    if (detection_mode == "DM_FAST") {
      mDetector.setDetectionMode(aruco::DM_FAST, min_marker_size);
    } else if (detection_mode == "DM_VIDEO_FAST") {
      mDetector.setDetectionMode(aruco::DM_VIDEO_FAST, min_marker_size);
    } else {
      // Aruco version 2 mode
      mDetector.setDetectionMode(aruco::DM_NORMAL, min_marker_size);
    }
    // mDetector.setDictionary("ARUCO_MIP_36h12", 0.5f);

    RCLCPP_INFO_STREAM(
      this->get_logger(), "Marker size min: " << min_marker_size << " of image area");
    RCLCPP_INFO_STREAM(this->get_logger(), "Detection mode: " << detection_mode);

    image_sub = it_->subscribe("/image", 1, &ArucoSimple::image_callback, this);
    cam_info_sub = this->create_subscription<sensor_msgs::msg::CameraInfo>(
      "/camera_info", 1, std::bind(
        &ArucoSimple::cam_info_callback, this,
        std::placeholders::_1));

    this->get_parameter_or<int>("marker_id", marker_id, 300);
    image_pub = it_->advertise(std::string(this->get_name())  + "/ID" + std::to_string(marker_id)+ std::string("/result"), 1);  // 
    debug_pub = it_->advertise(std::string(this->get_name())  + "/ID" + std::to_string(marker_id)+ std::string("/debug"), 1);
    pose_pub = subNode->create_publisher<geometry_msgs::msg::PoseStamped>("ID" + std::to_string(marker_id) + "/pose", 100);
    transform_pub =
      subNode->create_publisher<geometry_msgs::msg::TransformStamped>("ID" + std::to_string(marker_id)+"/transform", 100);
    position_pub = subNode->create_publisher<geometry_msgs::msg::Vector3Stamped>("ID" + std::to_string(marker_id)+"/position", 100);
    marker_pub = subNode->create_publisher<visualization_msgs::msg::Marker>("ID" + std::to_string(marker_id)+"/marker", 10);
    pixel_pub = subNode->create_publisher<geometry_msgs::msg::PointStamped>("ID" + std::to_string(marker_id)+"/pixel", 10);

    this->get_parameter_or<double>("marker_size", marker_size, 0.05);
    
    this->get_parameter_or<std::string>("reference_frame", reference_frame, "");
    this->get_parameter_or<std::string>("camera_frame", camera_frame, "");
    this->get_parameter_or<std::string>("marker_frame", marker_frame, "");
    this->get_parameter_or<bool>("image_is_rectified", useRectifiedImages, true);
    
    // *刘雨设置
    this->get_parameter_or<std::string>("marker_dictionary", marker_dictionary, "ARUCO_MIP_36h12");
    this->get_parameter_or<float>("error_correction_rate", error_correction_rate, 0.1f);

    mDetector.setDictionary(marker_dictionary, error_correction_rate);
    RCLCPP_INFO_STREAM(this->get_logger(), "Using dictionary: " << marker_dictionary);
    RCLCPP_INFO_STREAM(this->get_logger(), "Error correction rate: " << error_correction_rate);    
    //  刘雨设置*

    rcpputils::assert_true(
      camera_frame != "" && marker_frame != "",
      "Found the camera frame or the marker_frame to be empty!. camera_frame : " +
      camera_frame + " and marker_frame : " + marker_frame);

    if (reference_frame.empty()) {
      reference_frame = camera_frame;
    }

    RCLCPP_INFO(
      this->get_logger(), "ArUco node started with marker size of %f m and marker id to track: %d",
      marker_size, marker_id);
    RCLCPP_INFO(
      this->get_logger(), "ArUco node will publish pose to TF with %s as parent and %s as child.",
      reference_frame.c_str(), marker_frame.c_str());

    // dyn_rec_server.setCallback(boost::bind(&ArucoSimple::reconf_callback, this, _1, _2));
    RCLCPP_INFO(this->get_logger(), "Setup of aruco_simple node is successful!");
    return true;
  }

  bool getTransform(
    const std::string & refFrame, const std::string & childFrame,
    geometry_msgs::msg::TransformStamped & transform)
  {
    std::string errMsg;

    if (!tf_buffer_->canTransform(
        refFrame, childFrame, tf2::TimePointZero,
        tf2::durationFromSec(0.5), &errMsg))
    {
      RCLCPP_ERROR_STREAM(this->get_logger(), "Unable to get pose from TF: " << errMsg);
      return false;
    } else {
      try {
        transform = tf_buffer_->lookupTransform(
          refFrame, childFrame, tf2::TimePointZero, tf2::durationFromSec(
            0.5));
      } catch (const tf2::TransformException & e) {
        RCLCPP_ERROR_STREAM(
          this->get_logger(),
          "Error in lookupTransform of " << childFrame << " in " << refFrame << " : " << e.what());
        return false;
      }
    }
    return true;
  }
  cv::Mat sharpenAndAdjustContrast(const cv::Mat& image, int contrast_value = 10) {
      cv::Mat result;
      
      // 锐化
      cv::Mat kernel = (cv::Mat_<float>(3,3) << 
          -1, -1, -1,
          -1,  9, -1,
          -1, -1, -1);
      cv::filter2D(image, result, image.depth(), kernel);
      
      // 对比度调整
      double alpha = contrast_value / 127.0 + 1.0;
      double beta = 128.0 * (1.0 - alpha);
      result.convertTo(result, -1, alpha, beta);
      
      return result;
  }
  void image_callback(const sensor_msgs::msg::Image::ConstSharedPtr & msg)
  {
    if ((image_pub.getNumSubscribers() == 0) &&
      (debug_pub.getNumSubscribers() == 0) &&
      (pose_pub->get_subscription_count() == 0) &&
      (transform_pub->get_subscription_count() == 0) &&
      (position_pub->get_subscription_count() == 0) &&
      (marker_pub->get_subscription_count() == 0) &&
      (pixel_pub->get_subscription_count() == 0))
    {
      RCLCPP_DEBUG(this->get_logger(), "No subscribers, not looking for ArUco markers");
      return;
    }
    if (cam_info_received) {
      builtin_interfaces::msg::Time curr_stamp = msg->header.stamp;
      cv_bridge::CvImagePtr cv_ptr;
      try {
        cv_ptr = cv_bridge::toCvCopy(*msg, sensor_msgs::image_encodings::RGB8);
        inImage = cv_ptr->image;
        // // 图像锐化
        // inImage = sharpenAndAdjustContrast(inImage);
        // detection results will go into "markers"
        markers.clear();
        // ok, let's detect
        mDetector.detect(inImage, markers, camParam, marker_size, false);
        // for each marker, draw info and its boundaries in the image
        for (std::size_t i = 0; i < markers.size(); ++i) {
          // only publishing the selected marker
          if (markers[i].id == marker_id) {
            tf2::Transform transform = aruco_ros::arucoMarker2Tf2(markers[i]);
            tf2::Stamped<tf2::Transform> cameraToReference;
            cameraToReference.setIdentity();

            if (reference_frame != camera_frame) {
              geometry_msgs::msg::TransformStamped transform_stamped;
              getTransform(reference_frame, camera_frame, transform_stamped);
              tf2::fromMsg(transform_stamped, cameraToReference);
            }

            transform = static_cast<tf2::Transform>(cameraToReference) *
              static_cast<tf2::Transform>(rightToLeft) *
              transform;

            geometry_msgs::msg::TransformStamped stampedTransform;
            stampedTransform.header.frame_id = reference_frame;
            stampedTransform.header.stamp = curr_stamp;
            stampedTransform.child_frame_id = marker_frame;
            tf2::toMsg(transform, stampedTransform.transform);
            tf_broadcaster_->sendTransform(stampedTransform);
            geometry_msgs::msg::PoseStamped poseMsg;
            poseMsg.header = stampedTransform.header;
            tf2::toMsg(transform, poseMsg.pose);
            poseMsg.header.frame_id = reference_frame;
            poseMsg.header.stamp = curr_stamp;

            // 对 pose 做轻微滤波
            // geometry_msgs::msg::Pose filtered_pose = apply_pose_filter(poseMsg.pose);
            // poseMsg.pose = filtered_pose;
            pose_pub->publish(poseMsg);

            transform_pub->publish(stampedTransform);

            geometry_msgs::msg::Vector3Stamped positionMsg;
            positionMsg.header = stampedTransform.header;
            positionMsg.vector = stampedTransform.transform.translation;
            position_pub->publish(positionMsg);

            geometry_msgs::msg::PointStamped pixelMsg;
            pixelMsg.header = stampedTransform.header;
            pixelMsg.point.x = markers[i].getCenter().x;
            pixelMsg.point.y = markers[i].getCenter().y;
            pixelMsg.point.z = 0;
            pixel_pub->publish(pixelMsg);

            // publish rviz marker representing the ArUco marker patch
            visualization_msgs::msg::Marker visMarker;
            visMarker.header = stampedTransform.header;
            visMarker.id = 1;
            visMarker.type = visualization_msgs::msg::Marker::CUBE;
            visMarker.action = visualization_msgs::msg::Marker::ADD;
            visMarker.pose = poseMsg.pose;
            visMarker.scale.x = marker_size;
            visMarker.scale.y = marker_size;
            visMarker.scale.z = 0.001;
            visMarker.color.r = 1.0;
            visMarker.color.g = 0;
            visMarker.color.b = 0;
            visMarker.color.a = 1.0;
            visMarker.lifetime = builtin_interfaces::msg::Duration();
            visMarker.lifetime.sec = 3;
            marker_pub->publish(visMarker);
          }
          // but drawing all the detected markers
          markers[i].draw(inImage, cv::Scalar(0, 0, 255), 2);
        }

        // draw a 3d cube in each marker if there is 3d info
        if (camParam.isValid() && marker_size > 0) {
          for (std::size_t i = 0; i < markers.size(); ++i) {
            aruco::CvDrawingUtils::draw3dAxis(inImage, markers[i], camParam);
          }
        }

        if (image_pub.getNumSubscribers() > 0) {
          // show input with augmented information
          cv_bridge::CvImage out_msg;
          out_msg.header.stamp = curr_stamp;
          out_msg.encoding = sensor_msgs::image_encodings::RGB8;
          out_msg.image = inImage;
          image_pub.publish(out_msg.toImageMsg());
        }

        if (debug_pub.getNumSubscribers() > 0) {
          // show also the internal image resulting from the threshold operation
          cv_bridge::CvImage debug_msg;
          debug_msg.header.stamp = curr_stamp;
          debug_msg.encoding = sensor_msgs::image_encodings::MONO8;
          debug_msg.image = mDetector.getThresholdedImage();
          debug_pub.publish(debug_msg.toImageMsg());
        }
      } catch (cv_bridge::Exception & e) {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
      }
    }
  }

  // wait for one camerainfo, then shut down that subscriber
  void cam_info_callback(const sensor_msgs::msg::CameraInfo & msg)
  {
    if (!cam_info_received) {
      camParam = aruco_ros::rosCameraInfo2ArucoCamParams(msg, useRectifiedImages);

      // handle cartesian offset between stereo pairs
      // see the sensor_msgs/CameraInfo documentation for details
      rightToLeft.setIdentity();
      rightToLeft.setOrigin(tf2::Vector3(-msg.p[3] / msg.p[0], -msg.p[7] / msg.p[5], 0.0));

      cam_info_received = true;
    }
  }

//  void reconf_callback(aruco_ros::ArucoThresholdConfig & config, uint32_t level)
//  {
//    mDetector.setDetectionMode(
//      aruco::DetectionMode(config.detection_mode),
//      config.min_image_size);
//    if (config.normalizeImage) {
//      RCLCPP_WARN("normalizeImageIllumination is unimplemented!");
//    }
//  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  std::shared_ptr<ArucoSimple> aruco_simple = std::make_shared<ArucoSimple>();
  aruco_simple->setup();
  rclcpp::spin(aruco_simple);
  rclcpp::shutdown();
}
