#include <fsl_common.h>
#include "reconstruct.h"

extern int
fit_gauss(cv::Mat& curr_img, int row, cv::Mat & non_zero_vals, double estimate_mean, double& mid_point);

int
optimize_image_coordinates(cv::Vec2d& left_image_pts, cv::Vec2d& right_img_pts, Reconstruct3D* reconstructor,
cv::Mat& left_projection_matrix, cv::Mat& right_projection_matrix);
