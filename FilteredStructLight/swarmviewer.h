#pragma once
#include "robotviewer.h"
#include <random>
#include "robot.h"
#include <cmath>
#include <memory>
#include "swarmtree.h"
#include <QtCore/QThread>
#include "swarmutils.h"

class ParallelMCMCOptimizer;
class RobotWorker : public QObject {
	Q_OBJECT
	std::vector<Robot*> robots_;
	bool aborted_;
	bool paused_;
	int time_step_count_;
	int step_count_;
	SwarmOccupancyTree* occupancy_grid_;
	bool sampling_updated_;
	bool slow_down_;
	long long last_updated_time_;
	double accumulator_;
	double max_time_taken_;
public:
	RobotWorker();
	void set_robots(std::vector<Robot*> robots);
	void set_occupancy_tree(SwarmOccupancyTree* occupancy_grid);
	void set_slow_down(int slow_down);
	void abort();
	void set_max_time_taken(int max_time_taken);
	double calculate_coverage();
	void finish_work();


	void init();


public slots:
	void pause();
	void resume();
	void step();
	void do_work();	

signals:
	void update_time_step_count(int count);
	void update_sampling(double sampling);
	void update_sim_results(double timesteps, double multi_sampling, double coverage);


};

class SwarmViewer : public RobotViewer {
	Q_OBJECT

struct Light : public VisObject {

protected:

public:
	explicit Light(UniformLocations& locations)
		: VisObject(locations) {
	}
	int light_color_location_;
	int light_position_location_;
	glm::vec3 position_;
	glm::vec3 color_;
	virtual void update(glm::mat4 global_model);
};


private:

	SwarmOccupancyTree* occupancy_grid_;
	SwarmCollisionTree* collision_grid_;

	bool render_;
	GLint model_loc_;
	GLint inverse_transpose_loc_;
	GLint mvp_loc_;
	GLint view_position_loc_;
	bool show_forces_;

	Range separation_range_;
	Range alignment_range_;
	Range cluster_range_;
	Range perimeter_range_;
	Range explore_range_;
	Range obstacle_near_range_;
	Range obstacle_far_range_;

	double time_steps_result_;
	double multi_sampling_result_;
	double coverage_result_;
	double magic_k_;
	bool gui_render_;
	bool slow_down_;
	int neighborhood_count_;
	bool collide_with_robots_;

	static const std::string DEFAULT_INTERIOR_MODEL_FILENAME;
	static const int DEFAULT_NO_OF_ROBOTS;
	static const std::string OCCUPANCY_GRID_NAME;
	static const std::string OCCUPANCY_GRID_OVERLAY_NAME;
	std::vector<Robot*> robots_;
	std::vector<Light*> lights_;
	QTimer* timer_;
	UniformLocations uniform_locations_;

	std::vector<VisObject*> reset_vis_objects_;
	
	std::vector<VisObject*> default_vis_objects_;

	std::map<int, cv::Vec4f> robot_color_map_;

	QThread robot_update_thread_;
	RobotWorker* robot_worker_;


	ParallelMCMCOptimizer* parallel_mcmc_optimizer_;


	bool show_interior_;

	QGLShaderProgram line_shader_;

	bool paused_;
	int step_count_;
	int time_step_count_; 
	GridOverlay* overlay_;

protected:

	void load_inital_models() override;
	void initialize_position();
	virtual void set_shaders() override;
	void derive_floor_plan(const VertexBufferData& bufferdata, float scale, const glm::vec3& offset);
	void load_interior_model();
	void change_to_top_down_view();
	void update_perimiter_positions_in_overlay();
	virtual void custom_init_code() override;
	virtual void custom_draw_code() override;
	virtual void draw_mesh(RenderMesh& mesh) override;
	void shutdown_worker();
	bool intersect(const cv::Vec3f& n, float d,
		const cv::Vec3f& a, const cv::Vec3f& b, cv::Vec3f& intersection_pt) const;
	void quad_tree_test();

public:
	bool sim_results_updated_;
	static const int OCCUPANCY_GRID_HEIGHT;
	SwarmViewer(const QGLFormat& format, QWidget* parent = 0);
	virtual ~SwarmViewer();
	void cleanup();
	void create_light_model(RenderMesh& light_mesh);
	void create_robot_model(RenderMesh& light_mesh, cv::Vec4f color);
	void create_lights();
	std::vector<glm::vec3> create_starting_formation(Formation type);
	void create_robots();
	//void create_occupancy_grid_overlay(int grid_resolution, int grid_size, bool initialize = false);
	void create_occupancy_grid(int grid_resolution, int grid_size);
	void get_sim_results(double& timesteps, double& multi_sampling, double& coverage);

	unsigned int grid_resolution_;
	float grid_length_;
	int grid_resolution_per_side_;
	int no_of_robots_;

	std::string interior_model_filename_;
	double interior_scale_;
	glm::vec3 interior_offset_;
	glm::mat4 model_rotation_;


	double explore_constant_;
	double separation_constant_;
	double goto_work_constant_;
	double cluster_constant_;
	double perimeter_constant_;
	double alignment_constant_;

	double max_time_taken_;
	double square_radius_;
	double bounce_function_power_;
	double bounce_function_multiplier_;

	double separation_distance_;
	int formation_;

	double sensor_range_;
	int discovery_range_;
signals:
	void update_time_step_count(int count);
	void update_sampling(double sampling);
	void physics_thread_pause();
	void physics_thread_step();
	void physics_thread_resume();
	void optimizer_reset_sim();
	void update_sim_results_to_optimizer(double timesteps, double multi_sampling, double coverage);

public slots:

void update_sim_results(double timesteps, double multi_sampling, double coverage);
void set_no_of_robots(int no_of_robots);

void set_separation_distance(float distance);

void set_grid_resolution(int grid_resolution);
void set_grid_length(int grid_length);

void set_interior_scale(double scale);
void set_interior_offset(glm::vec3 offset);

void set_exploration_constant(double constant);
void set_separation_constant(double constant);
void set_alignment_constant(double constant);
void set_cluster_constant(double constant);
void set_perimeter_constant(double constant);
void set_goto_work_constant(double constant);

void set_show_interior(int show);
void set_model_rotation(double x_rotation, double y_rotation, double z_rotation);
void set_square_formation_radius(double radius);
void set_bounce_function_power(double bounce_function_power);
void set_bounce_function_multiplier(double bounce_function_multiplier);
void reset_sim();

void set_show_forces(int show);

void set_model_filename(QString filename);

void set_formation(int formation);

void set_separation_range(double min, double max);
void set_alignment_range(double min, double max);
void set_cluster_range(double min, double max);
void set_perimeter_range(double min, double max);
void set_explore_range(double min, double max);

void set_obstacle_avoidance_near_range(double min, double max);
void set_obstacle_avoidance_far_range(double min, double max);

void set_sensor_range(double sensor_range);
void set_discovery_range(int discovery_range);
void set_neighborhood_count(int count);

void pause();
void step();
void resume();

void run_least_squared_optimization();
void run_mcmc_optimization();

void set_magic_k(double magic_k);
void set_should_render(int render);
void set_slow_down(int slow_down);

void set_collide_with_robots(int collide);

};
