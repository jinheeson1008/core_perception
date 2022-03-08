//#include <ros/ros.h>

#include "DarknetDetector.h"
#include "op_tlr_core.h"


int main(int argc, char** argv)
{
    ros::init(argc, argv, "op_tlr");

    op_tlr_darknet_4_ns::TlrDetector _DeepDetector;
    _DeepDetector.MainLoop();

    return 0;
}
