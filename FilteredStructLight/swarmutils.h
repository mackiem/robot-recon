#pragma once
#include "fsl_common.h"
#include <QStringList>
#include <qspinbox.h>
#include <qcheckbox.h>

struct OptimizationParams {
	// optimization
	//QPushButton* add_swarm_configuration_button_;
	//QListView* swarm_configs_for_optimization_list_;
	//QSpinBox* no_of_threads_spin_box_;
	//QSpinBox* no_of_iterations_spin_box_;
	//QSpinBox* culling_nth_iteration_spin_box_;
	//QLineEdit* optimization_config_filename_;
	//QPushButton* optimization_config_filename_browse_;
	//QPushButton* load_optimization_config_button_;
	//QPushButton* save_optimization_config_button_;
	//QStringListModel* swarm_configs_list_model_;
	//QPushButton* remove_swarm_configuration_button_;
	//QPushButton* batch_optimize_button_;

	int no_of_threads;
	int no_of_iterations;
	int culling_nth_iteration;
	QStringList swarm_configs;
	
};

struct SwarmParams {

	int no_of_robots;
	double exploration_constant_;
	double separation_constant_;
	double alignment_constant_;
	double cluster_constant_;
	double perimeter_constant_;
	double goto_work_constant_;
	double separation_range_min_;
	double separation_range_max_;
	double alignment_range_min_;
	double alignment_range_max_;
	double cluster_range_min_;
	double cluster_range_max_;
	double perimeter_range_min_;
	double perimeter_range_max_;
	double explore_range_min_;
	double explore_range_max_;
	double obstacle_avoidance_near_range_min_;
	double obstacle_avoidance_near_range_max_;
	double obstacle_avoidance_far_range_min_;
	double obstacle_avoidance_far_range_max_;
	int formation;
	double magic_k_spin_box_;
	int neighborhood_count_;
	int grid_resolution_spin_box_;
	int grid_length_spin_box_;
	double scale_spinbox_;
	int x_spin_box_;
	int y_spin_box_;
	int z_spin_box_;
	bool show_interior_;
	QString model_filename_;
	bool show_forces_;
	bool collide_with_other_robots_;
	double square_radius_;
	double bounce_function_power_;
	double bounce_function_multiplier_;
	double sensor_range_;
	double discovery_range_;
};

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
	static const char* OPT_SWARM_CONFIGS_LIST;
	static const char* OPT_NO_OF_THREADS;
	static const char* OPT_NO_OF_ITERATIONS;
	static const char* OPT_CULLING_NTH_ITERATION;

	static const char* ROBOTS_NO_LABEL;
	static const char* EXPLORATION_CONSTANT_LABEL;
	static const char* SEPARATION_CONSTANT_LABEL;
	static const char* ALIGNMENT_CONSTANT_LABEL;
	static const char* CLUSTER_CONSTANT_LABEL;
	static const char* PERIMETER_CONSTANT_LABEL;


	static const char* EXPLORE_RANGE_MIN_LABEL;
	static const char* SEPARATION_RANGE_MIN_LABEL;
	static const char* ALIGNMENT_RANGE_MIN_LABEL;
	static const char* CLUSTER_RANGE_MIN_LABEL;
	static const char* PERIMETER_RANGE_MIN_LABEL;
	static const char* OBSTACLE_AVOIDANCE_NEAR_RANGE_MIN_LABEL;
	static const char* OBSTACLE_AVOIDANCE_NEAR_RANGE_MAX_LABEL;
	static const char* EXPLORE_RANGE_MAX_LABEL;
	static const char* SEPARATION_RANGE_MAX_LABEL;
	static const char* ALIGNMENT_RANGE_MAX_LABEL;
	static const char* CLUSTER_RANGE_MAX_LABEL;
	static const char* PERIMETER_RANGE_MAX_LABEL;
	static const char* OBSTACLE_AVOIDANCE_FAR_RANGE_MIN_LABEL;
	static const char* OBSTACLE_AVOIDANCE_FAR_RANGE_MAX_LABEL;
	static const char* GOTO_WORK_CONSTANT_LABEL;
	static const char* SEPARATION_DISTANCE_LABEL;
	static const char* FORMATION_LABEL;
	static const char* SENSOR_RANGE_LABEL;
	static const char* DISCOVERY_RANGE_LABEL;
	static const char* SHOW_FORCES_LABEL;
	static const char* COLLIDE_WITH_OTHER_ROBOTS_LABEL;
	static const char* MAGICK_LABEL;
	static const char* NEIGHBORHOOD_COUNT_LABEL;
	static const char* GRID_RESOLUTION_LABEL;
	static const char* GRID_LENGTH_LABEL;
	static const char* BUILDING_INTERIOR_SCALE_LABEL;
	static const char* BUILDING_OFFSET_X_LABEL;
	static const char* BUILDING_OFFSET_Y_LABEL;
	static const char* BUILDING_OFFSET_Z_LABEL;
	static const char* SHOW_BUILDING_LABEL;
	static const char* INTERIOR_MODEL_FILENAME;
	static const char* SQUARE_RADIUS;
	static const char* BOUNCE_FUNCTION_POWER;
	static const char* BOUNCE_FUNCTION_MULTIPLIER;
	static const char* MAX_TIME_TAKEN;
	static const char* NO_OF_CLUSTERS;
	static const char* DEATH_PERCENTAGE;
	static const char* DEATH_TIME_TAKEN;
public:
	static OptimizationParams load_optimization_params(const QString& filename);
	static void save_optimization_params(const OptimizationParams& params, const QString& filename);
	static SwarmParams load_swarm_params(const QString& filename);
	static void save_swarm_params(const SwarmParams& params, const QString& filename);
	static void print_vector(const std::string& name, const glm::vec3& vector);
	static void load_obj(std::string filename, std::vector<cv::Vec3f>& vertices,
		std::vector<cv::Vec3f>& normals, std::vector<cv::Vec2f>& uvs, std::vector<unsigned int>& indices,
		std::vector<int>& count, std::vector<int>& offset, std::vector<int>& base_index);
	static void load_obj(std::string filename, VertexBufferData& vertex_buffer_data);
	static cv::Mat convert_mat(glm::mat3& input_mat);
};

