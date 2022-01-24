

#include "op_tlr_core.h"
#include "op_planner/KmlMapLoader.h"
#include "op_ros_helpers/op_ROSHelpers.h"
#include "op_planner/PlanningHelpers.h"
#include "op_utility/UtilityH.h"
extern "C"{
#include "image_opencv.h"
}

namespace op_tlr_darknet_4_ns
{
    
TlrDetector::TlrDetector()
{
    ReadParams();

    m_YoloTlrDetector.Init(m_DeepParams);

	ros::NodeHandle _nh("~");

    image_sub = _nh.subscribe(m_Image_src_str, 1, &TlrDetector::ImageCallback, this);
    camera_info_sub = _nh.subscribe(m_Camera_info_str, 1, &TlrDetector::CameraInfoCallback, this);
    globalPlannerPaths_sub = _nh.subscribe("/lane_waypoints_array", 1, &TlrDetector::GlobalPlannerPathCallback, this);
    current_pose_sub = _nh.subscribe("/current_pose", 1,	&TlrDetector::CurrentPoseCallback, this);

	objects_pub = _nh.advertise<autoware_msgs::DetectedObjectArray>("/detection/image_detector/objects", 1);
	roi_sign_pub = _nh.advertise<autoware_msgs::Signals>("/roi_signal", 1);
	signalState_pub = _nh.advertise<autoware_msgs::TrafficLight>("/light_color", 1);

	UtilityHNS::UtilityH::GetTickCount(m_StopStateTimer);
}

TlrDetector::~TlrDetector()
{
}

void TlrDetector::ReadParams()
{
	ros::NodeHandle _nh("~");
	_nh.getParam("yolo_config_file", m_DeepParams.config_file);
	_nh.getParam("yolo_weights_file", m_DeepParams.weights_file);
	_nh.getParam("detection_threshold", m_DeepParams.detect_threshold);
	_nh.param<std::string>("camera_frame", m_Camera_frame_str, "camera");
	_nh.param<std::string>("camera_info_topic", m_Camera_info_str, "/camera_info");
	_nh.param<std::string>("image_src", m_Image_src_str, "/image_raw");


	if(m_Camera_frame_str.empty() || m_Camera_frame_str.size() < 2)
	{
		m_Camera_frame_str = "camera";
	}

	int iSource = 0;
	_nh.getParam("/op_common_params/mapSource" , iSource);
	_nh.getParam("/op_common_params/mapFileName" , m_MapPath);
	_nh.getParam("/op_common_params/pathDensity" , m_PathDensity);
	_nh.getParam("/op_common_params/horizonDistance" , m_PlanningHorizon);
	_nh.getParam("/op_common_params/giveUpDistance" , m_GiveUpDistance);



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
}

void TlrDetector::CurrentPoseCallback(const geometry_msgs::PoseStampedConstPtr& msg)
{
	m_CurrentPos.pos = PlannerHNS::GPSPoint(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z, tf::getYaw(msg->pose.orientation));
	bNewCurrentPos = true;

	if(UtilityHNS::UtilityH::GetTimeDiffNow(m_StopStateTimer) > 10) //Check distance every two seconds, we should move at leaset
	{
		double d = hypot(m_PrevPos.pos.y - m_CurrentPos.pos.y, m_PrevPos.pos.x - m_CurrentPos.pos.x);
		if(d > 5)
		{
			m_bStopping = false;
		}
		else
		{
			m_bStopping = true;
		}

		m_PrevPos = m_CurrentPos;
		UtilityHNS::UtilityH::GetTickCount(m_StopStateTimer);
	}

}

void TlrDetector::CameraInfoCallback(const sensor_msgs::CameraInfo& camInfoMsg)
{
	float fx_ = static_cast<float>(camInfoMsg.P[0]);
	float fy_ = static_cast<float>(camInfoMsg.P[5]);
	float image_width_ = camInfoMsg.width;
	float image_height_ = camInfoMsg.height;
	float cx_ = static_cast<float>(camInfoMsg.P[2]);
	float cy_ = static_cast<float>(camInfoMsg.P[6]);
}

void TlrDetector::ImageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    cv_bridge::CvImagePtr InputImage_cvPtr;
    cv::Mat mat_img;
    InputImage_cvPtr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGRA8);
    cv::cvtColor(InputImage_cvPtr->image, mat_img, cv::COLOR_BGRA2RGB);
    IplImage src = mat_img;

    std::vector<DetectedObjClass> objects = m_YoloTlrDetector.DetectObjects(src);

    autoware_msgs::DetectedObjectArray detections;

	for(auto& det_obj: objects)
	{
		autoware_msgs::DetectedObject obj;
		obj.valid = true;
		obj.height = det_obj.height;
		obj.width = det_obj.width;
		obj.label = det_obj.GetNameFromType();
		obj.score = det_obj.score;
		obj.x = det_obj.center_x;
		obj.y = det_obj.center_y;
		detections.objects.push_back(obj);
	}

    detections.header = msg->header;
    objects_pub.publish(detections);

	int est_state = EstimateTrafficLightState(objects);

	if(est_state != m_PrevLightState)
	{
		autoware_msgs::TrafficLight state_msg;
		state_msg.traffic_light = est_state;
		signalState_pub.publish(state_msg);
	}

	m_PrevLightState = est_state;
}

int TlrDetector::EstimateTrafficLightState(const std::vector<DetectedObjClass>& objects)
{
	int green_number = 0, red_number = 0;
	for(unsigned int i=0; i < objects.size(); i++)
	{
		DetectedObjClass det_obj = objects.at(i);

		if(det_obj.type == LEFT_RED || det_obj.type == RED || det_obj.type == RIGHT_RED || det_obj.type == YELLOW)
		{
			red_number++;
			if(det_obj.score > 0.6)
			{
				red_number++;
			}

			if(det_obj.type == RED)
			{
				red_number++;
			}

			if(objects.size() > 2 && i > 0 && i < objects.size()-1)
			{
				red_number++;
			}
		}
		else if(det_obj.type == LEFT_GREEN || det_obj.type == GREEN || det_obj.type == RIGHT_GREEN)
		{
			green_number++;
			if(det_obj.score > 0.6)
			{
				green_number++;
			}

			if(det_obj.type == GREEN)
			{
				green_number++;
			}

			if(objects.size() > 2 && i > 0 && i < objects.size()-1)
			{
				green_number++;
			}
		}
	}



	if(green_number > red_number)
	{
		m_PrevStates.push_back(1);
	}
	else
	{
		m_PrevStates.push_back(0);
	}

	if(m_PrevStates.size() > TRAFFIC_STATE_MAX_MEMORY)
	{
		m_PrevStates.erase(m_PrevStates.begin()+0);
	}

	int nGreen = 0, nRed = 0;

	for(auto& i_c: m_PrevStates)
	{
		if(i_c == 0)
		{
			nRed++;
		}
		else if(i_c == 1)
		{
			nGreen++;
		}
	}

	int current_state = 2;

	if(nRed >= nGreen)
	{
		current_state = 0;
	}
	else
	{
		current_state = 1;
	}

	//std::cout << "CurrentState: " << current_state << ", Reds (" << nRed << ", " <<  red_number << "), Greens (" << nGreen << ", "  << green_number << "), Detections: " << objects.size() << std::endl;

	if(bCloseLights)
	{
		if(m_bStopping && m_DistanceToClosestStopLine < 10 && objects.size() == 0 && UtilityHNS::UtilityH::GetTimeDiffNow(m_TimeOutTimer) > 100)
		{
			std::cout << "Sending fake GREEN light because of stuck situation ! " << std::endl;
			current_state = 1;
		}

		if(objects.size() > 0)
		{
			UtilityHNS::UtilityH::GetTickCount(m_TimeOutTimer);
		}
	}
	else
	{
		UtilityHNS::UtilityH::GetTickCount(m_TimeOutTimer);
		m_PrevStates.clear();
		current_state = 1;
	}

	return current_state;
}

void TlrDetector::GlobalPlannerPathCallback(const autoware_msgs::LaneArrayConstPtr& msg)
{
	if(msg->lanes.size() > 0 && bMap)
	{
		m_GlobalPaths.clear();
		for(unsigned int i = 0 ; i < msg->lanes.size(); i++)
		{
			PlannerHNS::ROSHelpers::ConvertFromAutowareLaneToLocalLane(msg->lanes.at(i), m_temp_path);

			if(bMap)
			{
				PlannerHNS::Lane* pPrevValid = 0;
				for(unsigned int j = 0 ; j < m_temp_path.size(); j++)
				{
					PlannerHNS::Lane* pLane = 0;
					pLane = PlannerHNS::MappingHelpers::GetLaneById(m_temp_path.at(j).laneId, m_Map);

					if(!pLane)
					{
						pLane = PlannerHNS::MappingHelpers::GetClosestLaneFromMap(m_temp_path.at(j), m_Map, 1, true);

						if(!pLane && !pPrevValid)
						{
							ROS_ERROR("Map inconsistency between Global Path and Local Planer Map, Can't identify current lane.");
							return;
						}

						if(!pLane)
							m_temp_path.at(j).pLane = pPrevValid;
						else
						{
							m_temp_path.at(j).pLane = pLane;
							pPrevValid = pLane ;
						}

						m_temp_path.at(j).laneId = m_temp_path.at(j).pLane->id;
					}
					else
						m_temp_path.at(j).pLane = pLane;
				}
			}

			m_GlobalPaths.push_back(m_temp_path);
		}

		bool bOldGlobalPath = true;
		if(m_PrevGlobalPaths.size() == m_GlobalPaths.size())
		{
			for(unsigned int i=0; i < m_GlobalPaths.size(); i++)
			{
				bOldGlobalPath = PlannerHNS::PlanningHelpers::CompareTrajectories(m_GlobalPaths.at(i), m_PrevGlobalPaths.at(i));
			}
		}
		else
		{
			bOldGlobalPath = false;
		}

		if(!bOldGlobalPath)
		{
			m_PrevGlobalPaths.clear();
			m_prev_index.clear();
			for(unsigned int i = 0; i < m_GlobalPaths.size(); i++)
			{
				m_PrevGlobalPaths.push_back(m_GlobalPaths.at(i));
				m_prev_index.push_back(0);
			}

			std::cout << "Received New Global Path op_tlr ! " << std::endl;
		}
	}
}

void TlrDetector::LoadMap(const std::string& file_name)
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

void TlrDetector::ExtractTrafficLightInformation()
{
	autoware_msgs::Signals signalsInFrame;
	signalsInFrame.header.stamp = ros::Time::now();
	signalsInFrame.header.frame_id = "map";

	if(m_GlobalPathsToUse.size() == 0) //Find traffic light by distance and and difference to the current position
	{
		for(auto& tl: m_Map.trafficLights)
		{
			double d = hypot(m_CurrentPos.pos.y - tl.pose.pos.y, m_CurrentPos.pos.x - tl.pose.pos.x);
			double a = atan2(m_CurrentPos.pos.y - tl.pose.pos.y, m_CurrentPos.pos.x - tl.pose.pos.x) * RAD2DEG;

			if(d < m_PlanningHorizon && a < 45)
			{
				autoware_msgs::ExtractedPosition sig_pos = ConstructTrafficSignal(tl);
				signalsInFrame.Signals.push_back(sig_pos);
			}
		}
	}
	else //use global path to find related traffic light
	{
		int closest_path_index = 0;
		if(m_GlobalPathsToUse.size() > 0)
		{
			double closest_distance = 100;
			for(unsigned int i=0; i < m_GlobalPathsToUse.size();  i++)
			{
				PlannerHNS::RelativeInfo info;
				PlannerHNS::PlanningHelpers::GetRelativeInfo(m_GlobalPathsToUse.at(i), m_CurrentPos, info);
				if(info.perp_distance < closest_distance)
				{
					closest_distance = info.perp_distance;
					closest_path_index = i;
				}
			}
		}

		int stopLineID = -1;
		int stopSignID = -1;
		std::vector<int> trafficLightIDs;
		m_DistanceToClosestStopLine = 0;
		std::vector<PlannerHNS::StopLine> slines;
		PlannerHNS::MappingHelpers::GetClosestStopLines(m_Map, m_CurrentPos, m_PlanningHorizon, slines);
		m_DistanceToClosestStopLine = PlannerHNS::PlanningHelpers::GetDistanceToClosestStopLineAndCheckV2(m_GlobalPathsToUse.at(closest_path_index),
				m_CurrentPos, slines, stopLineID, stopSignID, trafficLightIDs);

//		for(auto& sl: slines)
//		{
//			std::cout << " $$$ Found Stop LineID in the id: " << sl.id << std::endl;
//		}


//		if(trafficLightIDs.size() > 0)
//		{
//			std::cout << std::endl;
//			for(auto& tl_id: trafficLightIDs)
//			{
//				std::cout << "Lights .. Distance To StopLine: " << distanceToClosestStopLine << ", LineID: " << stopLineID << ", LightID: " << tl_id << ", IsRed: " << m_PrevLightState << std::endl;
//			}
//			std::cout << "-----------------" << std::endl;
//		}

		bCloseLights = false;
		if(m_DistanceToClosestStopLine > 0 && m_DistanceToClosestStopLine < m_PlanningHorizon)
		{
			for(auto& tl: m_Map.trafficLights)
			{
				for(auto& tl_id: trafficLightIDs)
				{
					if(tl.id == tl_id)
					{
						bCloseLights = true;
						autoware_msgs::ExtractedPosition sig_pos = ConstructTrafficSignal(tl);
						signalsInFrame.Signals.push_back(sig_pos);
					}
				}
			}
		}
	}

	roi_sign_pub.publish(signalsInFrame);
}

autoware_msgs::ExtractedPosition TlrDetector::ConstructTrafficSignal(const PlannerHNS::TrafficLight& tl)
{
	autoware_msgs::ExtractedPosition sign;
	sign.signalId = tl.id;

	sign.u = 0;
	sign.v = 0;

	sign.radius = 10.0;
	sign.x = tl.pose.pos.x;
	sign.y = tl.pose.pos.y;
	sign.z = tl.pose.pos.z;
	sign.hang = tl.horizontal_angle;  // hang is expressed in [0, 360] degree
	sign.type = 0;
	sign.linkId = tl.groupID;
	sign.plId = tl.stopLineID;
	sign.type = PlannerHNS::TRAFFIC_LIGHT_TYPE_STR.GetIndex(tl.lightType);

	return sign;
}

void TlrDetector::MainLoop()
{
	ros::Rate loop_rate(50);
	while (ros::ok())
	{
		if(!bMap)
		{
			ReadParams();
			if(m_MapType == PlannerHNS::MAP_KML_FILE)
			{
				LoadMap(m_MapPath);
				UtilityHNS::UtilityH::GetTickCount(m_TimeOutTimer);
				m_PrevPos = m_CurrentPos;
			}
		}

//		tf::StampedTransform transform;
//		tf::TransformListener tf_listener;
//		PlannerHNS::ROSHelpers::getTransformFromTF(m_Camera_frame_str, "map", tf_listener, transform);
//		PlannerHNS::ROSHelpers::getTransformFromTF("map", m_Camera_frame_str, tf_listener, transform);

		if(bMap && bNewCurrentPos)
		{
			m_GlobalPathsToUse.clear();
			std::vector<PlannerHNS::WayPoint> t_centerTrajectorySmoothed;
			for(unsigned int i = 0; i < m_PrevGlobalPaths.size(); i++)
			{
				t_centerTrajectorySmoothed.clear();
				m_prev_index.at(i) = PlannerHNS::PlanningHelpers::ExtractPartFromPointToDistanceDirectionFast(m_PrevGlobalPaths.at(i), m_CurrentPos, m_PlanningHorizon ,	m_PathDensity , t_centerTrajectorySmoothed, m_prev_index.at(i));
				if(m_prev_index.at(i) > 0 ) m_prev_index.at(i) = m_prev_index.at(i) -1;
				m_GlobalPathsToUse.push_back(t_centerTrajectorySmoothed);
			}

			ExtractTrafficLightInformation();
		}

		ros::spinOnce();
		loop_rate.sleep();
	}
}

}
