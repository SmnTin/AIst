#pragma once
#include <opencv2/opencv.hpp>

using namespace cv;

enum signs
{
	sign_none          = 0,
	sign_stop          = 1,	
	sign_crosswalk 	   = 2,
	sign_mainroad      = 3,
	sign_giveway       = 4,
	sign_starttrafficlight_green = 5,
	sign_starttrafficlight_red   = 6,
	sign_trafficlight_green      = 7,
	sign_trafficlight_yellow     = 8,
	sign_trafficlight_red        = 9
};

enum subSigns
{
	subsign_none          = 0,
	subsign_giveway_left  = 1,
	subsign_giveway_right = 2
};

enum trafficlight_states
{
	redlight    = 0,
	yellowlight = 1,
	greenlight  = 2
};