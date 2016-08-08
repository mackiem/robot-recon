#pragma once
#include "fsl_common.h"
#include <QStringList>

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
public:
	static OptimizationParams load_optimization_params(const QString& filename);
	static void save_optimization_params(const OptimizationParams& params, const QString& filename);
	static void print_vector(const std::string& name, const glm::vec3& vector);
	static void load_obj(std::string filename, std::vector<cv::Vec3f>& vertices,
		std::vector<cv::Vec3f>& normals, std::vector<cv::Vec2f>& uvs, std::vector<unsigned int>& indices,
		std::vector<int>& count, std::vector<int>& offset, std::vector<int>& base_index);
	static void load_obj(std::string filename, VertexBufferData& vertex_buffer_data);
	static cv::Mat convert_mat(glm::mat3& input_mat);
};

