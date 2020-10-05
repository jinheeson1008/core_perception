/*
 * Copyright 2015 sujiwo
 * 
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * 
 *    http://www.apache.org/licenses/LICENSE-2.0
 * 
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/**
 * Authors: sujiwo (original author), Hatem Darweesh (modified to handle kml, May 2020)
 */

#include "trafficlight_recognizer/feat_proj_kml/feat_proj_kml_core.h"

#include <algorithm>
#include <cstdio>
#include <iostream>

#include <op_planner/MappingHelpers.h>
#include <op_planner/KmlMapLoader.h>

namespace trafficlight_recognizer
{
	FeatProjKML::FeatProjKML()
	{
		m_MapType = PlannerHNS::MAP_KML_FILE;
		bMap = false;
		m_SignalLampDefaultRaius = 0.5;

		ReadCommonParams();

		ros::NodeHandle _nh;
		_nh.param<std::string>("/feature_proj_kml/camera_frame", camera_frame_, "camera");
		_nh.param<std::string>("/feature_proj_kml/camera_info_topic", camera_info_topic_, "/camera_info");
		_nh.getParam("/feature_proj_kml/x_adjust", adjust_proj_x_);
		_nh.getParam("/feature_proj_kml/y_adjust", adjust_proj_y_);
		_nh.getParam("/feature_proj_kml/signal_default_radius", m_SignalLampDefaultRaius);

		//std::cout << "X: " << adjust_proj_x_ << ", Y: " << adjust_proj_y_ << ", camera_frame: " << camera_frame_ << ", camera_info: " << camera_info_topic_ <<  std::endl;


		if(camera_frame_.empty() || camera_frame_.size() < 2)
		{
			camera_frame_ = "camera";
		}

		camera_info_subscriber_ = nh.subscribe(camera_info_topic_, 10, &FeatProjKML::cameraInfoCallback, this);
		adjustXY_subscriber_ = nh.subscribe("config/adjust_xy", 10, &FeatProjKML::adjustXYCallback, this);
		roi_sign_pub_ = nh.advertise<autoware_msgs::Signals>("roi_signal", 10);

		if(m_MapType == PlannerHNS::MAP_KML_FILE_NAME)
		{
			sub_map_file_name = nh.subscribe("/assure_kml_map_file_name", 1, &FeatProjKML::kmlMapFileNameCallback, this);
		}
	}

	void FeatProjKML::ReadCommonParams()
	{
		ros::NodeHandle _nh("~");
		int iSource = 0;
		_nh.getParam("/op_common_params/mapSource" , iSource);
		_nh.getParam("/op_common_params/mapFileName" , m_MapPath);

		if(iSource == 0)
		{
			m_MapType = PlannerHNS::MAP_AUTOWARE;
			std::cout << "Map source should be set to KML in op_common_params, otherwise use VectorMap or Lanelet2 options: " << m_MapPath << ", " << m_MapType << std::endl;
		}
		else if (iSource == 1)
		{
			m_MapType = PlannerHNS::MAP_FOLDER;
			std::cout << "Map source should be set to KML in op_common_params, otherwise use VectorMap or Lanelet2 options: " << m_MapPath << ", " << m_MapType << std::endl;
		}
		else if(iSource == 2)
		{
			m_MapType = PlannerHNS::MAP_KML_FILE;
		}
		else if(iSource == 3)
		{
			m_MapType = PlannerHNS::MAP_LANELET_2;
			std::cout << "Map source should be set to KML in op_common_params, otherwise use VectorMap or Lanelet2 options: " << m_MapPath << ", " << m_MapType << std::endl;
		}
		else if(iSource == 4)
		{
			m_MapType = PlannerHNS::MAP_KML_FILE_NAME;
		}
		//std::cout << "Read op_common Params From feat_proj_kml, start reading map from: " << m_MapPath << ", " << m_MapType << std::endl;
	}

	void FeatProjKML::kmlMapFileNameCallback(const std_msgs::String& file_name)
	{
		LoadMap(file_name.data);
	}

	// @brief get transformation between given frames
	void FeatProjKML::getTransform(const std::string from_frame, const std::string to_frame, Eigen::Quaternionf* ori,
	                                    Eigen::Vector3f* pos, tf::StampedTransform* tf)
	{
	  if (ori == nullptr || pos == nullptr || tf == nullptr)
	  {
	    ROS_ERROR_STREAM(__FUNCTION__ << ": ori, pos, or tf is null pointer!");
	    return;
	  }

	  static tf::TransformListener listener;

	  ros::Time now = ros::Time();
	  listener.waitForTransform(from_frame, to_frame, now, ros::Duration(10.0));
	  listener.lookupTransform(from_frame, to_frame, now, *tf);

	  tf::Vector3& p = tf->getOrigin();
	  tf::Quaternion o = tf->getRotation();

	  pos->x() = p.x();
	  pos->y() = p.y();
	  pos->z() = p.z();
	  ori->w() = o.w();
	  ori->x() = o.x();
	  ori->y() = o.y();
	  ori->z() = o.z();
	}

	void FeatProjKML::adjustXYCallback(const autoware_msgs::AdjustXY& config_msg)
	{
	  adjust_proj_x_ = config_msg.x;
	  adjust_proj_y_ = config_msg.y;
	}

	void FeatProjKML::cameraInfoCallback(const sensor_msgs::CameraInfo& camInfoMsg)
	{
	  fx_ = static_cast<float>(camInfoMsg.P[0]);
	  fy_ = static_cast<float>(camInfoMsg.P[5]);
	  image_width_ = camInfoMsg.width;
	  image_height_ = camInfoMsg.height;
	  cx_ = static_cast<float>(camInfoMsg.P[2]);
	  cy_ = static_cast<float>(camInfoMsg.P[6]);
	}

	void FeatProjKML::CollectAndPublishSigns()
	{
	  autoware_msgs::Signals signalsInFrame;
	  for (const auto& signal_map : m_Map.trafficLights)
	  {
		//std::cout << "Light: " << signal_map.id << " (" << signal_map.pose.pos.x <<", " << signal_map.pose.pos.y  << ")" << std::endl;
	    Eigen::Vector3f signalcenter(signal_map.pose.pos.x, signal_map.pose.pos.y, signal_map.pose.pos.z);
	    Eigen::Vector3f signalcenterx(signal_map.pose.pos.x, signal_map.pose.pos.y, signal_map.pose.pos.z + m_SignalLampDefaultRaius);

	    int u=0, v=0;
	    if (project2(signalcenter, &u, &v, false) == true)
	    {
	      int radius;
	      int ux, vx;
	      project2(signalcenterx, &ux, &vx, false);
	      //radius = static_cast<int>(hypot(u-ux, v-vx));
	      radius = static_cast<int>((Eigen::Vector2f(ux - u, vx - v)).norm());
	      //std::cout << ">>> Proj2:(ux,vx): " << ux << ", " << vx << std::endl;
	      //std::cout << ">>> (u,v,r): " << u << ", " << v << ", " << radius << std::endl;

	      autoware_msgs::ExtractedPosition sign;
	      sign.signalId = signal_map.id;

	      sign.u = u + adjust_proj_x_;  // shift project position by configuration value from runtime manager
	      sign.v = v + adjust_proj_y_;  // shift project position by configuration value from runtime manager

	      sign.radius = radius;
	      sign.x = signalcenter.x();
	      sign.y = signalcenter.y();
	      sign.z = signalcenter.z();
	      sign.hang = signal_map.horizontal_angle;  // hang is expressed in [0, 360] degree
	      sign.type = 0;
	      sign.linkId = signal_map.groupID;
	      sign.plId = signal_map.stopLineID;

	      if (signal_map.lightType == PlannerHNS::RED_LIGHT)
	      {
			  sign.type = 1;
	      }
			else if (signal_map.lightType == PlannerHNS::YELLOW_LIGHT)
			{
			  sign.type = 3;
			}
			else if (signal_map.lightType == PlannerHNS::GREEN_LIGHT)
			{
			  sign.type = 2;
			}
			else if (signal_map.lightType == PlannerHNS::CROSS_RED)
			{
			  sign.type = 4;
			}
			else if (signal_map.lightType == PlannerHNS::CROSS_GREEN)
			{
			  sign.type = 5;
			}
			else if (signal_map.lightType == PlannerHNS::LEFT_GREEN)
			{
			  sign.type = 6;
			}
			else if (signal_map.lightType == PlannerHNS::FORWARD_GREEN)
			{
			  sign.type = 7;
			}
			else if (signal_map.lightType == PlannerHNS::RIGHT_GREEN)
			{
			  sign.type = 8;
			}
			else
			{
				sign.type = 9;
			}

	      //convert the horizontal angle back to original vector map orientation
	      //tl.horizontal_angle = -vector_data.at(iv).Hang-180.0;

	      double original_hang = -signal_map.horizontal_angle - 180.0;

	      double signal_angle = GetSignalAngleInCameraSystem(original_hang + 180.0f, signal_map.vertical_angle + 180.0f);
	     // std::cout << ">>> Angles (hang,vang,signal): " << original_hang + 180.0f << ", " << signal_map.vertical_angle + 180.0f << ", " << signal_angle << std::endl;
	      // signal_angle will be zero if signal faces to x-axis
	      // Target signal should be face to -50 <= z-axis (= 90 degree) <= +50
	      if (isRange(-50, 50, signal_angle - 90))
	      {
	        signalsInFrame.Signals.push_back(sign);
	      }
	    }
	  }
	  signalsInFrame.header.stamp = ros::Time::now();
	  roi_sign_pub_.publish(signalsInFrame);

	 // std::cout << "There are " << signalsInFrame.Signals.size() << " signals in range" << std::endl;
	}

	/*
	 * Project a point from world coordinate to image plane
	 */
	bool FeatProjKML::project2(const Eigen::Vector3f& pt, int* u, int* v, bool useOpenGLCoord)
	{
		  if (u == nullptr || v == nullptr)
		  {
		    return false;
		  }

	  float nearPlane = 1.0;
	  float farPlane = 200.0;
	  Eigen::Vector3f _pt = transform(pt, camera_to_map_tf_);
	  float _u = _pt.x() * fx_ / _pt.z() + cx_;
	  float _v = _pt.y() * fy_ / _pt.z() + cy_;

	  *u = static_cast<int>(_u);
	  *v = static_cast<int>(_v);
	  if (*u < 0 || image_width_ < *u || *v < 0 || image_height_ < *v || _pt.z() < nearPlane || farPlane < _pt.z())
	  {
	    *u = -1, *v = -1;
	    return false;
	  }

	  if (useOpenGLCoord)
	  {
	    *v = image_height_ - *v;
	  }

	  return true;
	}

	Eigen::Vector3f FeatProjKML::transform(const Eigen::Vector3f& psrc, const tf::StampedTransform& tfsource)
	{
	  tf::Vector3 pt3(psrc.x(), psrc.y(), psrc.z());
	  tf::Vector3 pt3s = tfsource * pt3;
	  return Eigen::Vector3f(pt3s.x(), pt3s.y(), pt3s.z());
	}

	double FeatProjKML::GetSignalAngleInCameraSystem(double hang, double vang)
	{
	  double signal_pitch_in_map = (vang - 90.0)*DEG2RAD;
	  double signal_yaw_in_map =  (-hang + 90.0)*DEG2RAD;

	  tf::Quaternion signal_orientation_in_map_system;
	  signal_orientation_in_map_system.setRPY(0, signal_pitch_in_map, signal_yaw_in_map);

	  tf::Quaternion signal_orientation_in_cam_system = camera_to_map_tf_ * signal_orientation_in_map_system;
	  double signal_roll_in_cam;
	  double signal_pitch_in_cam;
	  double signal_yaw_in_cam;
	  tf::Matrix3x3(signal_orientation_in_cam_system).getRPY(signal_roll_in_cam, signal_pitch_in_cam, signal_yaw_in_cam);

	  return signal_pitch_in_cam*RAD2DEG;  // holizontal angle of camera is represented by pitch
	}

	bool FeatProjKML::isRange(const double lower, const double upper, const double val)
	{
	  if (lower <= upper)
	  {
	    if (lower < val && val < upper)
	    {
	      return true;
	    }
	  }
	  else
	  {
	    if (val < upper || lower < val)
	    {
	      return true;
	    }
	  }

	  return false;
	}

	void FeatProjKML::LoadMap(const std::string& file_name)
	{
		PlannerHNS::KmlMapLoader kml_loader;
		kml_loader.LoadKML(file_name, m_Map);
		PlannerHNS::MappingHelpers::ConvertVelocityToMeterPerSecond(m_Map);
		if(m_Map.trafficLights.size() > 0)
		{
			bMap = true;
			std::cout << " ******* KML Map is loaded successfully from feat_proj, KML File. Contains traffic lights: " << m_Map.trafficLights.size() << std::endl;
		}
		else
		{
			std::cout << " ******* KML Map loaded in feat_proj does not Contains traffic lights: " << m_Map.trafficLights.size() << std::endl;
		}
	}

	void FeatProjKML::MainLoop()
	{
		ros::Rate loop_rate(50);
		Eigen::Vector3f prev_position(0, 0, 0);
		Eigen::Quaternionf prev_orientation(0, 0, 0, 0);

		while (ros::ok())
		{
			if(!bMap)
			{
				ReadCommonParams();
				if(m_MapType == PlannerHNS::MAP_KML_FILE)
				{
					LoadMap(m_MapPath);
				}
			}

			try
			{
			  getTransform(camera_frame_, "map", &orientation_, &position_, &camera_to_map_tf_);
			  getTransform("map", camera_frame_, &orientation_, &position_, &map_to_camera_tf_);
			}
			catch (tf::TransformException& exc)
			{
			   ROS_WARN_STREAM(exc.what());
			}

			if(bMap)
			{
				//if (prev_orientation.vec() != orientation_.vec() && prev_position != position_)
				{
					CollectAndPublishSigns();
				}

				prev_orientation = orientation_;
				prev_position = position_;
			}

			ros::spinOnce();
			loop_rate.sleep();
		}
	}
}
