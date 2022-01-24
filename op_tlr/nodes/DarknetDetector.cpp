/*
 * DarknetDetector.cpp
 *
 *  Created on: Mar 17, 2021
 *      Author: Hatem Darweesh
 */

#include "DarknetDetector.h"
#include "op_utility/UtilityH.h"

namespace op_tlr_darknet_4_ns
{

DarknetDetector::DarknetDetector()
{
//	m_options = nullptr;
//	m_net = nullptr;

	UtilityHNS::UtilityH::GetTickCount(m_Timer);
	m_classes_names = new char*[m_nClasses];
	for(unsigned int i=0; i < m_nClasses; i++)
	{
		std::string c_name = DETECT_CLASS_TYPE_STR.GetString(i);
		m_classes_names[i] = new char[c_name.size()];
		strcpy(m_classes_names[i], c_name.c_str());
	}
}

DarknetDetector::~DarknetDetector()
{
	if(m_classes_names != nullptr)
	{
		free_ptrs((void**)m_classes_names, m_nClasses);
	}
}

int DarknetDetector::Init(DarknetParams& _params)
{
	m_params = _params;

	char* c_name =  new char[m_params.config_file.size()];
	strcpy(c_name, m_params.config_file.c_str());

	char* w_name =  new char[m_params.weights_file.size()];
	strcpy(w_name, m_params.weights_file.c_str());

	m_net = parse_network_cfg_custom(c_name, 1, 1);
	load_weights(&m_net, w_name);
	fuse_conv_batchnorm(m_net);
	calculate_binary_weights(m_net);

	delete [] c_name;
	delete [] w_name;

	return 0;
}

// compare to sort detection** by best_class probability
int compare_by_probs(const void *a_ptr, const void *b_ptr) {
    const detection_with_class* a = (detection_with_class*)a_ptr;
    const detection_with_class* b = (detection_with_class*)b_ptr;
    float delta = a->det.prob[a->best_class] - b->det.prob[b->best_class];
    return delta < 0 ? -1 : delta > 0 ? 1 : 0;
}

// compare to sort detection** by bbox.x
int compare_by_lefts(const void *a_ptr, const void *b_ptr) {
    const detection_with_class* a = (detection_with_class*)a_ptr;
    const detection_with_class* b = (detection_with_class*)b_ptr;
    const float delta = (a->det.bbox.x - a->det.bbox.w/2) - (b->det.bbox.x - b->det.bbox.w/2);
    return delta < 0 ? -1 : delta > 0 ? 1 : 0;
}

std::vector<DetectedObjClass> DarknetDetector::DetectObjects(IplImage& src_img)
{
	image img = ipl_to_image(&src_img);

	//image img_blr = blur_image(img, 3);
	//image img_blr = img;
	exposure_image(img, 0.75);
	image img_resized = resize_image(img, m_net.w, m_net.h);

	layer l = m_net.layers[m_net.n-1];
	int k;
	for (k = 0; k < m_net.n; ++k)
	{
		layer lk = m_net.layers[k];
		if (lk.type == YOLO || lk.type == GAUSSIAN_YOLO || lk.type == REGION)
		{
			l = lk;
		}
	}

	float *X = img_resized.data;
	network_predict(m_net, X);
	int nboxes = 0;
	detection *dets = get_network_boxes(&m_net, img.w, img.h, m_params.detect_threshold, 0.5, 0, 1, &nboxes, 0);

	do_nms_sort(dets, nboxes, l.classes, 0.45);

//	if(UtilityHNS::UtilityH::GetTimeDiffNow(m_Timer) > 1)
//	{
//		UtilityHNS::UtilityH::GetTickCount(m_Timer);
//
//		m_gCounter++;
//		std::string test_images_name = "/home/hatem/carla_board/team_agent/darknet/test_images/t";
//		std::ostringstream img_name;
//		img_name << test_images_name;
//		img_name << m_gCounter;
//		save_image(img, img_name.str().c_str());
////		int names_size = 0;
////		char **names = get_labels_custom("/home/hatem/carla_board/team_agent/darknet/tlr/bosch.names", &names_size);
//		image **alphabet = load_alphabet();
//		int ext_output = 0;
//		draw_detections_v3(img, dets, nboxes, m_params.detect_threshold, m_classes_names, alphabet, l.classes, ext_output);
//		m_gCounter++;
//		std::ostringstream img_name_res;
//		img_name_res << test_images_name;
//		img_name_res << m_gCounter;
//		save_image(img, img_name_res.str().c_str());
//
//		int selected_detections_num;
//		detection_with_class* selected_detections = get_actual_detections(dets, nboxes, m_params.detect_threshold, &selected_detections_num, m_classes_names);
//		qsort(selected_detections, selected_detections_num, sizeof(*selected_detections), compare_by_probs);
//
//		std::cout << " My Detections --------------------------- " << std::endl;
//		for (unsigned i = 0; i < selected_detections_num; ++i)
//		{
//			const int best_class = selected_detections[i].best_class;
//			printf("%s: %.0f%% \n", m_classes_names[best_class],    selected_detections[i].det.prob[best_class] * 100);
//		}
//
//		std::cout << std::endl << std::endl;
//	}

	int selected_detections_num;
	detection_with_class* selected_detections = get_actual_detections(dets, nboxes, m_params.detect_threshold, &selected_detections_num, m_classes_names);
	qsort(selected_detections, selected_detections_num, sizeof(*selected_detections), compare_by_lefts);

	std::vector<DetectedObjClass> detections;
	for (unsigned i = 0; i < selected_detections_num; ++i)
	{
		const int best_class = selected_detections[i].best_class;
		DetectedObjClass _c;
		_c.score = selected_detections[i].det.prob[best_class];
		_c.class_type_id = best_class;
		_c.type = DETECT_CLASS_TYPE_STR.GetEnum(_c.class_type_id);
		_c.bottom_left.x = (selected_detections[i].det.bbox.x-(selected_detections[i].det.bbox.w/2.0))*(double)img.w;
		_c.bottom_left.y = (selected_detections[i].det.bbox.y+(selected_detections[i].det.bbox.h/2.0))*(double)img.h;
		_c.top_right.x = (selected_detections[i].det.bbox.x+(selected_detections[i].det.bbox.w/2.0))*(double)img.w;
		_c.top_right.y = (selected_detections[i].det.bbox.y-(selected_detections[i].det.bbox.h/2.0))*(double)img.h;
		_c.width = selected_detections[i].det.bbox.w * (double)img.w;
		_c.height = selected_detections[i].det.bbox.h * (double)img.h;
		_c.center_x = _c.bottom_left.x;
		_c.center_y = _c.top_right.y;
		//std::cout << m_classes_names[best_class] << ": " << selected_detections[i].det.prob[best_class] * 100 << "% , (" << _c.width << ", " << _c.height << ")" <<  std::endl;
		if(_c.width > 5 && _c.height > 7 && _c.bottom_left.y < (img.h - 100))
		{
			detections.push_back(_c);
		}
	}

//	std::vector<DetectedObjClass> detections;
//	for (int i = 0; i < nboxes; ++i)
//	{
//		char buff[1024];
//		int class_id = -1;
//		float prob = m_params.detect_threshold;
//		for (int j = 0; j < l.classes; ++j)
//		{
//			if (dets[i].prob[j] > m_params.detect_threshold && dets[i].prob[j] > prob)
//			{
//				prob = dets[i].prob[j];
//				class_id = j;
//			}
//		}
//
//		if (class_id >= 0)
//		{
//			//printf("%d %2.4f \n", class_id, prob*100.0);
//		}
//
//		if(class_id < 0 ) continue;
//
//		DetectedObjClass _c;
//		_c.score = prob;
//		_c.class_type_id = class_id+1;
//		_c.type = DETECT_CLASS_TYPE_STR.GetEnum(_c.class_type_id);
//		_c.bottom_left.x = (dets[i].bbox.x-(dets[i].bbox.w/2.0))*(double)img.w;
//		_c.bottom_left.y = (dets[i].bbox.y+(dets[i].bbox.h/2.0))*(double)img.h;
//		_c.top_right.x = (dets[i].bbox.x+(dets[i].bbox.w/2.0))*(double)img.w;
//		_c.top_right.y = (dets[i].bbox.y-(dets[i].bbox.h/2.0))*(double)img.h;
//		_c.width = dets[i].bbox.w * (double)img.w;
//		_c.height = dets[i].bbox.h * (double)img.h;
//		_c.center_x = _c.bottom_left.x;
//		_c.center_y = _c.top_right.y;
//
//		//filter by size and location
//
//		if(_c.center_x < 30 || (_c.center_x > (img.w-30)) ||  _c.height < 30 || (_c.height > (img.h-30)))
//		{
//			continue;
//		}
//		//std::cout << "W: " << _c.width << ", H: " << _c.height << ", XC: " << _c.center_x << ", YC: " << _c.center_y << std::endl;
//
//		detections.push_back(_c);
//	}

	free_detections(dets, nboxes);
	free_image(img);
	free_image(img_resized);

	//free_image(img_blr);

	return detections;
}

image DarknetDetector::ipl_to_image(IplImage* src)
{
    image out = make_image(src->width, src->height, m_net.c);

    unsigned char *data = (unsigned char *)src->imageData;
	int h = src->height;
	int w = src->width;
	int c = src->nChannels;
	int step = src->widthStep;
	int i, j, k;

	for(i = 0; i < h; ++i){
		for(k= 0; k < c; ++k){
			for(j = 0; j < w; ++j){
				out.data[k*w*h + i*w + j] = data[i*step + j*c + k]/255.;
			}
		}
	}

    return out;
}

}

