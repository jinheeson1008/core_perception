/*
 * Copyright 2018-2019 Autoware Foundation. All rights reserved.
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
 */

#ifndef KF_CONTOUR_TRACKER_CORE
#define KF_CONTOUR_TRACKER_CORE

// ROS includes
#include <ros/ros.h>

#include "vector_map_msgs/PointArray.h"
#include "vector_map_msgs/LaneArray.h"
#include "vector_map_msgs/NodeArray.h"
#include "vector_map_msgs/StopLineArray.h"
#include "vector_map_msgs/DTLaneArray.h"
#include "vector_map_msgs/LineArray.h"
#include "vector_map_msgs/AreaArray.h"
#include "vector_map_msgs/SignalArray.h"
#include "vector_map_msgs/StopLine.h"
#include "vector_map_msgs/VectorArray.h"

#include "op_planner/RoadNetwork.h"
#include "op_planner/PlannerCommonDef.h"
#include "op_planner/MatrixOperations.h"
#include "SimpleTracker.h"
#include "PolygonGenerator.h"

#include <std_msgs/String.h>
#include <tf/transform_listener.h>
#include <autoware_can_msgs/CANInfo.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TwistStamped.h>
#include <jsk_recognition_msgs/BoundingBoxArray.h>
#include <autoware_msgs/CloudClusterArray.h>
#include <autoware_msgs/DetectedObjectArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/MarkerArray.h>
#include <autoware_lanelet2_msgs/MapBin.h>



namespace ContourTrackerNS
{

enum MAP_FILTER_TYPE
{
	FILTER_DISABLE,
	FILTER_BOUNDARY,
	FILTER_CENTERLINES
	};

class PerceptionParams
{
public:

	double 	VehicleWidth;
	double 	VehicleLength;
	double 	DetectionRadius;
	double 	MinObjSize;
	double 	MaxObjSize;
	double  nQuarters;
	double 	PolygonRes;
	TRACKING_TYPE	trackingType; // 0 association only , 1 simple tracking, 2 contour based tracking
	bool bEnableSimulation;
	bool bEnableStepByStep;
	bool bEnableLogging;
	bool bEnableTTC;
	bool bEnableLaneChange;
	bool bEnableBenchmark;
	bool bEnableInternalVisualization;
	bool bUseDetectionHulls;
	MAP_FILTER_TYPE filterType;
	double centerlineFilterDistance;

	PerceptionParams()
	{
		VehicleWidth =0;
		VehicleLength =0;
		DetectionRadius =0;
		MinObjSize =0;
		MaxObjSize =0;
		nQuarters = 0;
		PolygonRes = 0;
		trackingType = SIMPLE_TRACKER;
		bEnableStepByStep = false;
		bEnableSimulation = false;
		bEnableLogging = false;
		bEnableTTC = false;
		bEnableLaneChange = false;
		bEnableBenchmark = false;
		bEnableInternalVisualization = false;
		bUseDetectionHulls = false;
		filterType = FILTER_DISABLE;
		centerlineFilterDistance = 1.5;
	}
};

class ContourTracker
{
protected:
	std::vector<PlannerHNS::DetectedObject> m_OriginalClusters;
	autoware_msgs::DetectedObjectArray m_OutPutResults;
	bool bNewClusters;
	PlannerHNS::WayPoint m_CurrentPos;
	PlannerHNS::VehicleState m_VehicleStatus;
	int m_VelocitySource;
	bool bNewCurrentPos;
	PerceptionParams m_Params;
	SimpleTracker m_ObstacleTracking;

	//Visualization Section
	int m_nDummyObjPerRep;
	int m_nDetectedObjRepresentations;
	std::vector<visualization_msgs::MarkerArray> m_DetectedPolygonsDummy;
	std::vector<visualization_msgs::MarkerArray> m_DetectedPolygonsActual;
	visualization_msgs::MarkerArray m_DetectedPolygonsAllMarkers;
	visualization_msgs::MarkerArray m_DetectionCircles;

	std::vector<visualization_msgs::MarkerArray> m_MatchingInfoDummy;
	std::vector<visualization_msgs::MarkerArray> m_MatchingInfoActual;


	visualization_msgs::MarkerArray m_TTC_Path;
	visualization_msgs::Marker m_TTC_Info;

	std::vector<std::string>    m_LogData;
	PlannerHNS::MAP_SOURCE_TYPE m_MapType;
	std::string m_MapPath;
	PlannerHNS::RoadNetwork m_Map;
	bool bMap;
	bool bCommonParams;
	std::string m_ExperimentFolderName;

	std::vector<PlannerHNS::Lane*> m_ClosestLanesList;

	int m_nOriginalPoints;
	int m_nContourPoints;
	double m_FilteringTime;
	double m_PolyEstimationTime;
	double m_tracking_time;
	double m_dt;
	struct timespec  m_loop_timer;
	int frame_count_;
	std::string kitti_data_dir_;
	std::string result_file_path_;
	//std::string pointcloud_frame ;
	std::string target_tracking_frame;
	std::string source_data_frame;
	tf::TransformListener tf_listener;
	tf::StampedTransform m_local2global;
	std_msgs::Header m_InputHeader;

	//ROS subscribers
	ros::NodeHandle nh;

	//define publishers
	ros::Publisher pub_AllTrackedObjects;

	ros::Publisher pub_DetectedPolygonsRviz;
	ros::Publisher pub_TrackedObstaclesRviz;
	ros::Publisher pub_TTC_PathRviz;

	// define subscribers.
	ros::Subscriber sub_cloud_clusters;
	ros::Subscriber sub_current_pose ;
	ros::Subscriber sub_detected_objects;
	ros::Subscriber sub_current_velocity;
	ros::Subscriber sub_robot_odom;
	ros::Subscriber sub_can_info;
	ros::Subscriber sub_map_file_name;


	// Callback function for subscriber.
	void callbackGetCloudClusters(const autoware_msgs::CloudClusterArrayConstPtr &msg);
	void callbackGetCurrentPose(const geometry_msgs::PoseStampedConstPtr& msg);
	void callbackGetDetectedObjects(const autoware_msgs::DetectedObjectArrayConstPtr& msg);
	void callbackGetVehicleStatus(const geometry_msgs::TwistStampedConstPtr& msg);
	void callbackGetCanInfo(const autoware_can_msgs::CANInfoConstPtr & msg);
	void callbackGetRobotOdom(const nav_msgs::OdometryConstPtr& msg);

	//Helper Functions
	void VisualizeLocalTracking();
	void ImportCloudClusters(const autoware_msgs::CloudClusterArray& msg, std::vector<PlannerHNS::DetectedObject>& originalClusters);
	void ImportDetectedObjects(const autoware_msgs::DetectedObjectArray& msg, std::vector<PlannerHNS::DetectedObject>& originalClusters);
	bool FilterByMap(const PlannerHNS::DetectedObject& obj, const PlannerHNS::WayPoint& currState, PlannerHNS::RoadNetwork& map);
	bool FilterBySize(const PlannerHNS::DetectedObject& obj, const PlannerHNS::WayPoint& currState);
	void CalculateTTC(const std::vector<PlannerHNS::DetectedObject>& objs, const PlannerHNS::WayPoint& currState, PlannerHNS::RoadNetwork& map);
	void GetFrontTrajectories(std::vector<PlannerHNS::Lane*>& lanes, const PlannerHNS::WayPoint& currState, const double& max_distance, std::vector<std::vector<PlannerHNS::WayPoint> >& trajectories);
	void transformPoseToGlobal(const std::string& src_frame, const std::string& dst_frame, const tf::StampedTransform& local2global, const autoware_msgs::CloudClusterArray& input, autoware_msgs::CloudClusterArray& transformed_input);
	void ReadNodeParams();
	void ReadCommonParams();
	void Log();
	void PostProcess();
	void dumpResultText(autoware_msgs::DetectedObjectArray& detected_objects);

public:
  ContourTracker();
  ~ContourTracker();
  void MainLoop();

	//Mapping Section
	UtilityHNS::MapRaw m_MapRaw;
  	ros::Subscriber sub_bin_map;
	ros::Subscriber sub_lanes;
	ros::Subscriber sub_points;
	ros::Subscriber sub_dt_lanes;
	ros::Subscriber sub_intersect;
	ros::Subscriber sup_area;
	ros::Subscriber sub_lines;
	ros::Subscriber sub_stop_line;
	ros::Subscriber sub_signals;
	ros::Subscriber sub_vectors;
	ros::Subscriber sub_curbs;
	ros::Subscriber sub_edges;
	ros::Subscriber sub_way_areas;
	ros::Subscriber sub_cross_walk;
	ros::Subscriber sub_nodes;

	void callbackGetLanelet2(const autoware_lanelet2_msgs::MapBin& msg);
	void callbackGetVMLanes(const vector_map_msgs::LaneArray& msg);
	void callbackGetVMPoints(const vector_map_msgs::PointArray& msg);
	void callbackGetVMdtLanes(const vector_map_msgs::DTLaneArray& msg);
	void callbackGetVMIntersections(const vector_map_msgs::CrossRoadArray& msg);
	void callbackGetVMAreas(const vector_map_msgs::AreaArray& msg);
	void callbackGetVMLines(const vector_map_msgs::LineArray& msg);
	void callbackGetVMStopLines(const vector_map_msgs::StopLineArray& msg);
	void callbackGetVMSignal(const vector_map_msgs::SignalArray& msg);
	void callbackGetVMVectors(const vector_map_msgs::VectorArray& msg);
	void callbackGetVMCurbs(const vector_map_msgs::CurbArray& msg);
	void callbackGetVMRoadEdges(const vector_map_msgs::RoadEdgeArray& msg);
	void callbackGetVMWayAreas(const vector_map_msgs::WayAreaArray& msg);
	void callbackGetVMCrossWalks(const vector_map_msgs::CrossWalkArray& msg);
	void callbackGetVMNodes(const vector_map_msgs::NodeArray& msg);
	void kmlMapFileNameCallback(const std_msgs::String& file_name);
	void LoadKmlMap(const std::string& file_name);
	void LoadMap();
};

}

#endif  // KF_CONTOUR_TRACKER_CORE
