#include "swarmutils.h"
#include <tiny_obj_loader.h>
#include "renderentity.h"
#include <qsettings.h>

const char* SwarmUtils::OPT_SWARM_CONFIGS_LIST = "OPT_SWARM_CONFIGS_LIST";
const char* SwarmUtils::OPT_NO_OF_THREADS = "OPT_NO_OF_THREADS";
const char* SwarmUtils::OPT_NO_OF_ITERATIONS = "OPT_NO_OF_ITERATIONS";
const char* SwarmUtils::OPT_CULLING_NTH_ITERATION = "OPT_CULLING_NTH_ITERATION";

const char* SwarmUtils::ROBOTS_NO_LABEL = "robots_no";
const char* SwarmUtils::EXPLORATION_CONSTANT_LABEL = "exploration_constant";
const char* SwarmUtils::SEPARATION_CONSTANT_LABEL = "separation_constant";
const char* SwarmUtils::ALIGNMENT_CONSTANT_LABEL = "alignment_constant";
const char* SwarmUtils::CLUSTER_CONSTANT_LABEL = "cluster_constant";
const char* SwarmUtils::PERIMETER_CONSTANT_LABEL = "perimeter_constant";
const char* SwarmUtils::GOTO_WORK_CONSTANT_LABEL = "goto_work_constant";
//const char* SwarmUtils::SEPARATION_DISTANCE_LABEL = "separation_distance";
const char* SwarmUtils::SHOW_FORCES_LABEL = "show_forces";

const char* SwarmUtils::COLLIDE_WITH_OTHER_ROBOTS_LABEL = "collide_with_other_robots";

const char* SwarmUtils::MAGICK_LABEL = "magick";
const char* SwarmUtils::FORMATION_LABEL = "formation";
const char* SwarmUtils::SENSOR_RANGE_LABEL = "sensor_range";
const char* SwarmUtils::DISCOVERY_RANGE_LABEL = "discovery_range";

const char* SwarmUtils::EXPLORE_RANGE_MIN_LABEL = "explore_range_min";
const char* SwarmUtils::SEPARATION_RANGE_MIN_LABEL = "separation_range_min";
const char* SwarmUtils::ALIGNMENT_RANGE_MIN_LABEL = "alignment_range_min";
const char* SwarmUtils::CLUSTER_RANGE_MIN_LABEL = "cluster_range_min";
const char* SwarmUtils::PERIMETER_RANGE_MIN_LABEL = "perimeter_range_min";
const char* SwarmUtils::OBSTACLE_AVOIDANCE_NEAR_RANGE_MIN_LABEL = "obstacle_avoidance_near_range_min";
const char* SwarmUtils::OBSTACLE_AVOIDANCE_NEAR_RANGE_MAX_LABEL = "obstacle_avoidance_near_range_max";

const char* SwarmUtils::EXPLORE_RANGE_MAX_LABEL = "explore_range_max";
const char* SwarmUtils::SEPARATION_RANGE_MAX_LABEL = "separation_range_max";
const char* SwarmUtils::ALIGNMENT_RANGE_MAX_LABEL = "alignment_range_max";
const char* SwarmUtils::CLUSTER_RANGE_MAX_LABEL = "cluster_range_max";
const char* SwarmUtils::PERIMETER_RANGE_MAX_LABEL = "perimeter_range_max";
const char* SwarmUtils::OBSTACLE_AVOIDANCE_FAR_RANGE_MIN_LABEL = "obstacle_avoidance_far_range_min";
const char* SwarmUtils::OBSTACLE_AVOIDANCE_FAR_RANGE_MAX_LABEL = "obstacle_avoidance_far_range_max";

const char* SwarmUtils::NEIGHBORHOOD_COUNT_LABEL = "neighborhood_count";
const char* SwarmUtils::GRID_RESOLUTION_LABEL = "grid_resolution";
const char* SwarmUtils::GRID_LENGTH_LABEL = "grid_length";
const char* SwarmUtils::BUILDING_INTERIOR_SCALE_LABEL = "interior_scale";
const char* SwarmUtils::BUILDING_OFFSET_X_LABEL = "interior_offset_x";
const char* SwarmUtils::BUILDING_OFFSET_Y_LABEL = "interior_offset_y";
const char* SwarmUtils::BUILDING_OFFSET_Z_LABEL = "interior_offset_z";
const char* SwarmUtils::SHOW_BUILDING_LABEL = "show_interior";
const char* SwarmUtils::INTERIOR_MODEL_FILENAME = "interior_model_filename";


const char* SwarmUtils::SQUARE_RADIUS = "square_radius";
const char* SwarmUtils::BOUNCE_FUNCTION_POWER = "bounce_function_power";
const char* SwarmUtils::BOUNCE_FUNCTION_MULTIPLIER = "bounce_function_multiplier";

const char* SwarmUtils::MAX_TIME_TAKEN = "max_time_taken";
const char* SwarmUtils::NO_OF_CLUSTERS = "no_of_clusters";
const char* SwarmUtils::DEATH_PERCENTAGE = "death_percentage";
const char* SwarmUtils::DEATH_TIME_TAKEN = "death_time_taken";



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

SwarmParams SwarmUtils::load_optimization_params(const QString& filename) {
	QSettings settings(filename, QSettings::IniFormat);
	
	SwarmParams swarm_params;
	swarm_params.no_of_robots = (settings.value(ROBOTS_NO_LABEL, "10").toInt());
	swarm_params.exploration_constant_ = (settings.value(EXPLORATION_CONSTANT_LABEL, "1").toDouble());
	swarm_params.separation_constant_ = (settings.value(SEPARATION_CONSTANT_LABEL, "1").toDouble());
	swarm_params.alignment_constant_ = (settings.value(ALIGNMENT_CONSTANT_LABEL, "1").toDouble());
	swarm_params.cluster_constant_ = (settings.value(CLUSTER_CONSTANT_LABEL, "1").toDouble());
	swarm_params.perimeter_constant_ = (settings.value(PERIMETER_CONSTANT_LABEL, "1").toDouble());
	swarm_params.goto_work_constant_ = (settings.value(GOTO_WORK_CONSTANT_LABEL, "1").toDouble());

	swarm_params.formation = (settings.value(FORMATION_LABEL, "0").toInt());

	swarm_params.grid_resolution_spin_box_ = (settings.value(GRID_RESOLUTION_LABEL, "4096").toInt());
	swarm_params.grid_length_spin_box_ = (settings.value(GRID_LENGTH_LABEL, "20").toInt());

	swarm_params.sensor_range_ = (settings.value(SENSOR_RANGE_LABEL, "5").toDouble());

	swarm_params.discovery_range_ = (settings.value(DISCOVERY_RANGE_LABEL, "0").toInt());

	swarm_params.separation_range_min_ = (settings.value(SEPARATION_RANGE_MIN_LABEL, "0").toDouble());
	swarm_params.separation_range_max_ = (settings.value(SEPARATION_RANGE_MAX_LABEL, "1").toDouble());

	swarm_params.alignment_range_min_ = (settings.value(ALIGNMENT_RANGE_MIN_LABEL, "0").toDouble());
	swarm_params.alignment_range_max_ = (settings.value(ALIGNMENT_RANGE_MAX_LABEL, "1").toDouble());

	swarm_params.cluster_range_min_ = (settings.value(CLUSTER_RANGE_MIN_LABEL, "0").toDouble());
	swarm_params.cluster_range_max_ = (settings.value(CLUSTER_RANGE_MAX_LABEL, "1").toDouble());

	swarm_params.perimeter_range_min_ = (settings.value(PERIMETER_RANGE_MIN_LABEL, "0").toDouble());
	swarm_params.perimeter_range_max_ = (settings.value(PERIMETER_RANGE_MAX_LABEL, "1").toDouble());

	swarm_params.explore_range_min_ = (settings.value(EXPLORE_RANGE_MIN_LABEL, "0").toDouble());
	swarm_params.explore_range_max_ = (settings.value(EXPLORE_RANGE_MAX_LABEL, "1").toDouble());


	swarm_params.obstacle_avoidance_far_range_min_ = (settings.value(OBSTACLE_AVOIDANCE_FAR_RANGE_MIN_LABEL, "1").toDouble());
	swarm_params.obstacle_avoidance_far_range_max_ = (settings.value(OBSTACLE_AVOIDANCE_FAR_RANGE_MAX_LABEL, "5").toDouble());

	swarm_params.obstacle_avoidance_near_range_min_ = (settings.value(OBSTACLE_AVOIDANCE_NEAR_RANGE_MIN_LABEL, "0").toDouble());
	swarm_params.obstacle_avoidance_near_range_max_ = (settings.value(OBSTACLE_AVOIDANCE_NEAR_RANGE_MAX_LABEL, "1").toDouble());


	//separation_distance_->setValue(settings.value(SEPARATION_DISTANCE_LABEL, "100").toDouble());


	swarm_params.model_filename_ = (settings.value(INTERIOR_MODEL_FILENAME, "interior/house interior.obj").toString());


	swarm_params.scale_spinbox_ = (settings.value(BUILDING_INTERIOR_SCALE_LABEL, "2").toDouble());
	swarm_params.x_spin_box_ = (settings.value(BUILDING_OFFSET_X_LABEL, "0").toInt());
	swarm_params.y_spin_box_ = (settings.value(BUILDING_OFFSET_Y_LABEL, "0").toInt());
	swarm_params.z_spin_box_ = (settings.value(BUILDING_OFFSET_Z_LABEL, "0").toInt());

	swarm_params.show_interior_ = settings.value(SHOW_BUILDING_LABEL, "1").toBool();

	swarm_params.show_forces_ = settings.value(SHOW_FORCES_LABEL, "1").toBool();

	swarm_params.collide_with_other_robots_ = settings.value(COLLIDE_WITH_OTHER_ROBOTS_LABEL, "1").toBool();

	swarm_params.neighborhood_count_ = (settings.value(NEIGHBORHOOD_COUNT_LABEL, "5").toInt());

	swarm_params.magic_k_spin_box_ = (settings.value(MAGICK_LABEL, "0.1").toDouble());
	swarm_params.square_radius_ = (settings.value(SQUARE_RADIUS, "4.0").toDouble());
	swarm_params.bounce_function_power_ = (settings.value(BOUNCE_FUNCTION_POWER, "3.0").toDouble());
	swarm_params.bounce_function_multiplier_ = (settings.value(BOUNCE_FUNCTION_MULTIPLIER, "1.0").toDouble());

	return swarm_params;
}


void SwarmUtils::save_swarm_params(const SwarmParams& params, const QString& filename) {
	QSettings settings(filename, QSettings::IniFormat);


	settings.setValue(ROBOTS_NO_LABEL, params.no_of_robots);
	settings.setValue(EXPLORATION_CONSTANT_LABEL, params.exploration_constant_);
	settings.setValue(SEPARATION_CONSTANT_LABEL, params.separation_constant_);
	settings.setValue(ALIGNMENT_CONSTANT_LABEL, params.alignment_constant_);
	settings.setValue(CLUSTER_CONSTANT_LABEL, params.cluster_constant_);
	settings.setValue(PERIMETER_CONSTANT_LABEL, params.perimeter_constant_);
	settings.setValue(GOTO_WORK_CONSTANT_LABEL, params.goto_work_constant_);

	settings.setValue(SEPARATION_RANGE_MIN_LABEL, params.separation_range_min_);
	settings.setValue(SEPARATION_RANGE_MAX_LABEL, params.separation_range_max_);

	settings.setValue(ALIGNMENT_RANGE_MIN_LABEL, params.alignment_range_min_);
	settings.setValue(ALIGNMENT_RANGE_MAX_LABEL, params.alignment_range_max_);

	settings.setValue(CLUSTER_RANGE_MIN_LABEL, params.cluster_range_min_);
	settings.setValue(CLUSTER_RANGE_MAX_LABEL, params.cluster_range_max_);

	settings.setValue(PERIMETER_RANGE_MIN_LABEL, params.perimeter_range_min_);
	settings.setValue(PERIMETER_RANGE_MAX_LABEL, params.perimeter_range_max_);

	settings.setValue(EXPLORE_RANGE_MIN_LABEL, params.explore_range_min_);
	settings.setValue(EXPLORE_RANGE_MAX_LABEL, params.explore_range_max_);

	settings.setValue(OBSTACLE_AVOIDANCE_NEAR_RANGE_MIN_LABEL, params.obstacle_avoidance_near_range_min_);
	settings.setValue(OBSTACLE_AVOIDANCE_NEAR_RANGE_MAX_LABEL, params.obstacle_avoidance_near_range_max_);

	settings.setValue(OBSTACLE_AVOIDANCE_FAR_RANGE_MIN_LABEL, params.obstacle_avoidance_far_range_min_);
	settings.setValue(OBSTACLE_AVOIDANCE_FAR_RANGE_MAX_LABEL, params.obstacle_avoidance_far_range_max_);

	settings.setValue(SENSOR_RANGE_LABEL, params.sensor_range_);

	settings.setValue(DISCOVERY_RANGE_LABEL, params.discovery_range_);

	settings.setValue(FORMATION_LABEL, params.formation);
	
	settings.setValue(MAGICK_LABEL, params.magic_k_spin_box_);

	settings.setValue(NEIGHBORHOOD_COUNT_LABEL, params.neighborhood_count_);
	
	//settings.setValue(SEPARATION_DISTANCE_LABEL, separation_distance_->value());

	settings.setValue(GRID_RESOLUTION_LABEL, params.grid_resolution_spin_box_);
	settings.setValue(GRID_LENGTH_LABEL, params.grid_length_spin_box_);

	settings.setValue(BUILDING_INTERIOR_SCALE_LABEL, params.scale_spinbox_);
	settings.setValue(BUILDING_OFFSET_X_LABEL, params.x_spin_box_);
	settings.setValue(BUILDING_OFFSET_Y_LABEL, params.y_spin_box_);
	settings.setValue(BUILDING_OFFSET_Z_LABEL, params.z_spin_box_);
	settings.setValue(SHOW_BUILDING_LABEL, params.show_interior_);
	settings.setValue(INTERIOR_MODEL_FILENAME, params.model_filename_);

	settings.setValue(SHOW_FORCES_LABEL, params.show_forces_);

	settings.setValue(COLLIDE_WITH_OTHER_ROBOTS_LABEL, params.collide_with_other_robots_);

	settings.setValue(SQUARE_RADIUS, params.square_radius_);
	settings.setValue(BOUNCE_FUNCTION_POWER, params.bounce_function_power_);
	settings.setValue(BOUNCE_FUNCTION_MULTIPLIER, params.bounce_function_multiplier_);
	
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
