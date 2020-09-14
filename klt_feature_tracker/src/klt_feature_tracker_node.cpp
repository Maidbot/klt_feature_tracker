#include "rclcpp/rclcpp.hpp"
#include <cv_bridge/cv_bridge.h>

#include "klt_feature_tracker_msgs/srv/track_features.hpp"
#include "klt_feature_tracker/klt_feature_tracker.h"


void track(const std::shared_ptr<klt_feature_tracker_msgs::srv::TrackFeatures::Request> request,
           std::shared_ptr<klt_feature_tracker_msgs::srv::TrackFeatures::Response> response)
{
  cv_bridge::CvImagePtr left_image;
  cv_bridge::CvImagePtr right_image;

  try {
      left_image = cv_bridge::toCvCopy(request->left_image, "mono8");
  } catch (cv_bridge::Exception& e) {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),
                   "Error while converting left image to OpenCV: %s", e.what());
  }
  try {
      right_image = cv_bridge::toCvCopy(request->right_image, "mono8");
  } catch (cv_bridge::Exception& e) {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),
                   "Error while converting right image to OpenCV: %s", e.what());
  }

  std::vector<cv::Point2f> features_l;
  std::vector<cv::Point2f> features_r;

  response->features_l.resize(request->update_vect.size()*2);
  response->features_r.resize(request->update_vect.size()*2);

  trackFeatures(left_image->image, right_image->image, features_l, features_r, request->update_vect, request->stereo);

  for (unsigned int i = 0; i < features_l.size(); i++) {
      response->features_l[2*i + 0] = features_l[i].x;
      response->features_l[2*i + 1] = features_l[i].y;

      response->features_r[2*i + 0] = features_r[i].x;
      response->features_r[2*i + 1] = features_r[i].y;
  }
  response->update_vect = request->update_vect;
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("klt_feature_tracker");

  rclcpp::Service<klt_feature_tracker_msgs::srv::TrackFeatures>::SharedPtr service =
    node->create_service<klt_feature_tracker_msgs::srv::TrackFeatures>("track_features", &track);

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Ready to track features.");

  rclcpp::spin(node);
  rclcpp::shutdown();
}