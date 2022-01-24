/*
 * DarknetDetector.h
 *
 *  Created on: Mar 17, 2021
 *      Author: Hatem Darweesh
 */

#ifndef DARKNETDETECTOR_H_
#define DARKNETDETECTOR_H_

#include <ros/ros.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "cuda_runtime.h"
#include "curand.h"
#include "cublas_v2.h"
#include "op_planner/MappingHelpers.h"

extern "C"{
#include "network.h"
#include "detection_layer.h"
#include "region_layer.h"
#include "cost_layer.h"
#include "utils.h"
#include "parser.h"
#include "box.h"
#include "image.h"
#include "image_opencv.h"
#include "demo.h"
#include "option_list.h"
#include <sys/time.h>
}

namespace op_tlr_darknet_4_ns
{

enum DETECT_CLASS_TYPE{
	LEFT_RED = 0,
	RED = 1,
	RIGHT_RED = 2,
	LEFT_GREEN = 3,
	GREEN = 4,
	RIGHT_GREEN = 5,
	YELLOW = 6,
	OFF = 7,
	UNKNOWN_CLASS = 8,
	};

static PlannerHNS::EnumString<DETECT_CLASS_TYPE> DETECT_CLASS_TYPE_STR(UNKNOWN_CLASS,
		{
				{LEFT_RED, "Left Red"},
				{RED, "Red"},
				{RIGHT_RED, "Right Red"},
				{LEFT_GREEN, "Left Green"},
				{GREEN, "Green"},
				{RIGHT_GREEN, "Right Green"},
				{YELLOW, "Yellow"},
				{OFF, "Off"},
				{UNKNOWN_CLASS, "Unknown"},
		});

class DetectedObjClass
{
public:
	double score;
	cv::Point top_right;
	cv::Point bottom_left;	
	double width;
	double height;
	double center_x;
	double center_y;
	unsigned int class_type_id;
	DETECT_CLASS_TYPE type;
	std::string class_label;

	DetectedObjClass()
	{
		center_x = 0;
		center_y = 0;
		width = 0;
		height = 0;
		score = 0;
		class_type_id = 0;
		type = UNKNOWN_CLASS;
	}

	std::string GetNameFromType()
	{
		return DETECT_CLASS_TYPE_STR.GetString(type);
	}

};

class DarknetParams
{
public:
	std::string config_file;
	std::string weights_file;

	double detect_threshold;

	DarknetParams()
	{
		detect_threshold = 0.5;
	}
};

class DarknetDetector
{

public:
	DarknetParams m_params;
	network m_net;

	DarknetDetector();
	virtual ~DarknetDetector();
	std::vector<DetectedObjClass> DetectObjects(IplImage& src_img);
	int Init( DarknetParams& _params);
	image ipl_to_image(IplImage* src);
	int m_gCounter = 0;
	timespec m_Timer;
	char** m_classes_names = nullptr;
	const int m_nClasses = 8;
};

}

#endif /* DARKNETDETECTOR_H_ */
