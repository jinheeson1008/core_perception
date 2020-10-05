/*
 * Copyright 2020 Autoware Foundation. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 * Authors: Hatem Darweesh
 *
 */

#ifndef TRAFFICLIGHT_RECOGNIZER_FEAT_PROJ_KML_CORE_H
#define TRAFFICLIGHT_RECOGNIZER_FEAT_PROJ_KML_CORE_H

#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <Eigen/Eigen>
#include <autoware_msgs/AdjustXY.h>
#include <autoware_msgs/Signals.h>
#include <sensor_msgs/CameraInfo.h>
#include <std_msgs/String.h>
#include <visualization_msgs/MarkerArray.h>
#include <op_planner/RoadNetwork.h>

#include <string>
#include <vector>

namespace trafficlight_recognizer
{
class FeatProjKML
{
  friend class FeatProjKmlTestClass;

public:
  FeatProjKML();
  void MainLoop();

private:
  ros::NodeHandle nh;

  ros::Subscriber camera_info_subscriber_;
  ros::Subscriber adjustXY_subscriber_;
  ros::Subscriber sub_map_file_name;

  ros::Publisher roi_sign_pub_;

  std::string camera_frame_;
  std::string camera_info_topic_;

	PlannerHNS::MAP_SOURCE_TYPE m_MapType = PlannerHNS::MAP_KML_FILE;
	std::string m_MapPath;
	PlannerHNS::RoadNetwork m_Map;
	bool bMap = false;
	double m_SignalLampDefaultRaius;

  int adjust_proj_x_ = 0;
  int adjust_proj_y_ = 0;

  Eigen::Vector3f position_;
  Eigen::Quaternionf orientation_;
  float fx_ = 0.0;
  float fy_ = 0.0;
  float image_width_ = 0.0;
  float image_height_ = 0.0;
  float cx_ = 0.0;
  float cy_ = 0.0;

  tf::StampedTransform camera_to_map_tf_;
  tf::StampedTransform map_to_camera_tf_;

  void kmlMapFileNameCallback(const std_msgs::String& file_name);
  void adjustXYCallback(const autoware_msgs::AdjustXY& config_msg);
  void cameraInfoCallback(const sensor_msgs::CameraInfo& camInfoMsg);

  void getTransform(const std::string from_frame, const std::string to_frame, Eigen::Quaternionf* ori,
                    Eigen::Vector3f* pos, tf::StampedTransform* tf);
  Eigen::Vector3f transform(const Eigen::Vector3f& psrc, const tf::StampedTransform& tfsource);
  bool project2(const Eigen::Vector3f& pt, int* u, int* v, const bool useOpenGLCoord = false);
  void ReadCommonParams();
  void CollectAndPublishSigns();
  double GetSignalAngleInCameraSystem(double hang, double vang);
  bool isRange(const double lower, const double upper, const double val);
  void LoadMap(const std::string& file_name);

};
}  // namespace trafficlight_recognizer

#endif  // TRAFFICLIGHT_RECOGNIZER_FEAT_PROJ_KML_CORE_H
