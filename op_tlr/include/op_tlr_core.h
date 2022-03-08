/*
 * op_darknet_detector_core.h
 *
 *  Created on: Mar 17, 2021
 *      Author: Hatem Darweesh
 */

#ifndef OP_DARKNET_DETECTOR_CORE_H_
#define OP_DARKNET_DETECTOR_CORE_H_

#include <ros/ros.h>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <std_msgs/UInt64.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/image_encodings.h>
#include <autoware_msgs/ProjectionMatrix.h>
#include <autoware_msgs/DetectedObjectArray.h>
#include <autoware_msgs/Signals.h>
#include <autoware_msgs/TrafficLight.h>
#include <autoware_msgs/LaneArray.h>

#include "DarknetDetector.h"

namespace op_tlr_darknet_4_ns
{

#define TRAFFIC_STATE_MAX_MEMORY 30//we need at least 3 frames to confirm a light state
#define STUCK_TIME_OUT 120 //seconds

//#define TRAFFIC_LIGHT_RED 0
//#define TRAFFIC_LIGHT_GREEN 1
//#define TRAFFIC_LIGHT_UNKNOWN 2


class TlrDetector
{
public:
    TlrDetector();
    ~TlrDetector();
    
    void MainLoop();
    
private:
    void ReadParams();            
    void ImageCallback(const sensor_msgs::ImageConstPtr& msg);
    void CameraInfoCallback(const sensor_msgs::CameraInfo& camInfoMsg);
    void GlobalPlannerPathCallback(const autoware_msgs::LaneArrayConstPtr& msg);
    void CurrentPoseCallback(const geometry_msgs::PoseStampedConstPtr& msg);

    void LoadMap(const std::string& file_name);
    void ExtractTrafficLightInformation();
    autoware_msgs::ExtractedPosition ConstructTrafficSignal(const PlannerHNS::TrafficLight& tl);
    int EstimateTrafficLightState(const std::vector<DetectedObjClass>& objects);

private:
    ros::Subscriber camera_info_sub;
    ros::Subscriber image_sub;
    ros::Subscriber globalPlannerPaths_sub;
    ros::Subscriber current_pose_sub;
    ros::Publisher objects_pub;
    ros::Publisher roi_sign_pub;
    ros::Publisher signalState_pub;
    
    DarknetParams m_DeepParams;
    DarknetDetector m_YoloTlrDetector;
    std::string m_Camera_frame_str;
    std::string m_Camera_info_str;
    std::string m_Image_src_str;

	PlannerHNS::MAP_SOURCE_TYPE m_MapType = PlannerHNS::MAP_KML_FILE;
	std::string m_MapPath;
	PlannerHNS::RoadNetwork m_Map;
	bool bMap = false;
	std::vector<std::vector<PlannerHNS::WayPoint> > m_GlobalPaths;
	std::vector<std::vector<PlannerHNS::WayPoint> > m_PrevGlobalPaths;
	std::vector<std::vector<PlannerHNS::WayPoint> > m_GlobalPathsToUse;
	std::vector<PlannerHNS::WayPoint> m_temp_path;
	double m_PathDensity = 0.5;
	double m_PlanningHorizon = 50;
	double m_GiveUpDistance = 0;
	PlannerHNS::WayPoint m_CurrentPos;
	PlannerHNS::WayPoint m_PrevPos;
	bool bNewCurrentPos = false;
	bool bCloseLights = false;
	int m_PrevLightState = 2;
	std::vector<int> m_PrevStates;
	std::vector<int> m_prev_index;
	timespec m_TimeOutTimer;
	timespec m_StopStateTimer;
	double m_DistanceToClosestStopLine = 0;
	bool m_bStopping;
	bool m_bSenFakeGreen;


};


}

#endif  //OP_DARKNET_DETECTOR
