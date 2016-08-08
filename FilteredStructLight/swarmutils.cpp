#include "swarmutils.h"
#include <tiny_obj_loader.h>
#include "renderentity.h"
#include <qsettings.h>

const char* SwarmUtils::OPT_SWARM_CONFIGS_LIST = "OPT_SWARM_CONFIGS_LIST";
const char* SwarmUtils::OPT_NO_OF_THREADS = "OPT_NO_OF_THREADS";
const char* SwarmUtils::OPT_NO_OF_ITERATIONS = "OPT_NO_OF_ITERATIONS";
const char* SwarmUtils::OPT_CULLING_NTH_ITERATION = "OPT_CULLING_NTH_ITERATION";

OptimizationParams SwarmUtils::load_optimization_params(const QString& filename) {
	QSettings settings(filename, QSettings::IniFormat);
	
	OptimizationParams optimization_params;

	optimization_params.swarm_configs = settings.value(OPT_SWARM_CONFIGS_LIST, "").toStringList();
	optimization_params.no_of_iterations = settings.value(OPT_NO_OF_ITERATIONS, "10").toInt();
	optimization_params.no_of_threads = settings.value(OPT_NO_OF_THREADS, "10").toInt();
	optimization_params.culling_nth_iteration = settings.value(OPT_CULLING_NTH_ITERATION, "5").toInt();

	return optimization_params;
}

void SwarmUtils::save_optimization_params(const OptimizationParams& params, const QString& filename) {
	QSettings settings(filename, QSettings::IniFormat);

	settings.setValue(OPT_SWARM_CONFIGS_LIST, params.swarm_configs);
	settings.setValue(OPT_NO_OF_THREADS, params.no_of_threads);
	settings.setValue(OPT_NO_OF_ITERATIONS, params.no_of_iterations);
	settings.setValue(OPT_CULLING_NTH_ITERATION, params.culling_nth_iteration);
}

void SwarmUtils::print_vector(const std::string& name, const glm::vec3& vector) {
//#ifdef DEBUG
	std::cout << name << " : " << vector.x << ", " << vector.y << ", " << vector.z << std::endl;
//#endif
}

cv::Mat SwarmUtils::convert_mat(glm::mat3& input_mat) {

	cv::Mat mat(3, 3, CV_32F, glm::value_ptr(input_mat));
	cv::Mat transposed_mat = mat.t();
	return transposed_mat;
}

void SwarmUtils::load_obj(std::string filename,  std::vector<cv::Vec3f>& vertices,
	std::vector<cv::Vec3f>& normals, std::vector<cv::Vec2f>& uvs, std::vector<unsigned int>& indices,
	std::vector<int>& count, std::vector<int>& offset, std::vector<int>& base_index) {

	std::vector<unsigned int> glm_indices;
	std::vector<glm::vec3> glm_vertices;
	std::vector<glm::vec2> glm_uvs;
	std::vector<glm::vec3> glm_normals;
	//loadAssImp(filename.c_str(), glm_indices, glm_vertices, glm_uvs, glm_normals);

	std::vector<tinyobj::shape_t> shapes;
	std::vector < tinyobj::material_t> materials;
	std::string error;
	tinyobj::LoadObj(shapes, materials, error, filename.c_str());

	//std::vector<int> count;
	//std::vector<int> offset;

	
	const int no_of_vertices_per_polygon = 3;
	int total_offset = 0;
	int total_indices = 0;
	for (auto& shape : shapes) {
		auto& mesh = shape.mesh;

		assert(mesh.indices.size() % no_of_vertices_per_polygon == 0);
		assert(mesh.positions.size() % no_of_vertices_per_polygon == 0);

		for (size_t i = 0; i < (mesh.positions.size() / no_of_vertices_per_polygon); ++i) {
			glm::vec3 position(mesh.positions[(no_of_vertices_per_polygon * i) + 0], 
				mesh.positions[(no_of_vertices_per_polygon * i) + 1], 
				mesh.positions[(no_of_vertices_per_polygon * i) + 2]);
			glm::vec3 normal(mesh.normals[(no_of_vertices_per_polygon * i) + 0], 
				mesh.normals[(no_of_vertices_per_polygon * i) + 1], 
				mesh.normals[(no_of_vertices_per_polygon * i) + 2]);
			if (mesh.texcoords.size() > 0 && (mesh.texcoords.size() > 2 * i)) {
				glm::vec2 texcoord(mesh.texcoords[(2 * i) + 0],
					mesh.texcoords[(2 * i) + 1]);
				glm_uvs.push_back(texcoord);
			}
			glm_vertices.push_back(position);
			glm_normals.push_back(normal);
		}

		int no_of_vertices = mesh.positions.size() / 3;

		int indices_count = mesh.indices.size();
		for (size_t i = 0; i < indices_count; ++i) {
			glm_indices.push_back(mesh.indices[i]);
		}
		count.push_back(indices_count);
		offset.push_back(total_offset);
		base_index.push_back(total_indices);

		total_offset += (no_of_vertices);
		total_indices += indices_count;
	}
	
	for (auto& out_vertice : glm_vertices) {
		vertices.push_back(cv::Vec3f(static_cast<float>(out_vertice[0]), static_cast<float>(out_vertice[1])
			, static_cast<float>(out_vertice[2])));
	}

	for (auto& out_vertice : glm_normals) {
		normals.push_back(cv::Vec3f(static_cast<float>(out_vertice[0]), static_cast<float>(out_vertice[1])
			, static_cast<float>(out_vertice[2])));
	}

	for (auto& out_vertice : glm_uvs) {
		uvs.push_back(cv::Vec2f(static_cast<float>(out_vertice[0]), static_cast<float>(out_vertice[1])));
	}

	indices = glm_indices;
}

void SwarmUtils::load_obj(std::string filename, VertexBufferData& vertex_buffer_data) {
	load_obj(filename, vertex_buffer_data.positions, vertex_buffer_data.normals, vertex_buffer_data.uvs,
		vertex_buffer_data.indices, vertex_buffer_data.count, vertex_buffer_data.offset, vertex_buffer_data.base_index);
}
