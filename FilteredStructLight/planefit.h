#include <fsl_common.h>
#include "robotreconstruction.h"

extern int
fit_gauss(cv::Mat& curr_img, int row, cv::Mat & non_zero_vals, double estimate_mean, double& mid_point);


extern int
fit_line(std::vector<cv::Point3f>& line_stripe_points, Ray& ray);

extern int
fit_plane(std::vector<Ray>& rays, Plane& plane);
