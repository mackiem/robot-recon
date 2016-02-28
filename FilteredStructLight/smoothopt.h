#include "fsl_common.h"
#include <vector>
#include "include/opencv2/video/background_segm.hpp"

extern "C" {
#include "lsqr.h"
}

float
optimize_smoothness(WPt& worlds_pts, const IntensityPerImage& left_intensities, const IntensityPerImage& right_intensities);
