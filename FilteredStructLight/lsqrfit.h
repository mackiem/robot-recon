#pragma once
#include "fsl_common.h"

extern "C" {
#include "lsqr.h"
}

double find_optimal_edge_zero_crossing(std::vector<cv::Point2f>& crossing_points);


