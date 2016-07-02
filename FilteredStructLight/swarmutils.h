#pragma once
#include "fsl_common.h"

struct VertexBufferData;

enum Formation {
	GRID = 0,
	RANDOM = 1,
	SQUARE = 2,
	SQUARE_CLOSE_TO_EDGE = 3,
	CIRCLE = 4
};

class SwarmUtils
{
public:
	static void print_vector(const std::string& name, const glm::vec3& vector);
	static void load_obj(std::string filename, std::vector<cv::Vec3f>& vertices,
		std::vector<cv::Vec3f>& normals, std::vector<cv::Vec2f>& uvs, std::vector<unsigned int>& indices,
		std::vector<int>& count, std::vector<int>& offset, std::vector<int>& base_index);
	static void load_obj(std::string filename, VertexBufferData& vertex_buffer_data);
	static cv::Mat convert_mat(glm::mat3& input_mat);
};

