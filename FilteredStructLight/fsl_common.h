#pragma once

#include <glm/glm.hpp>
#include <glm/mat4x4.hpp> // glm::mat4
#include <glm/gtc/matrix_transform.hpp> // glm::translate, glm::rotate, glm::scale, glm::perspective
#include <glm/gtc/type_ptr.hpp>
#include "opencv2/opencv.hpp"
typedef std::vector<cv::Vec2d> IPt;
typedef std::vector<cv::Vec3d> WPt;
typedef std::vector<IPt> IPts;
typedef std::vector<WPt> WPts;
typedef std::vector<double> IntensityPerImage;
typedef std::vector<std::vector<double>> Intensities;
#include <unordered_map>
