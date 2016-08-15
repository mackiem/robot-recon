#pragma once
#include "qobject.h"
#include "experimentalrobot.h"
#include "swarmutils.h"
#include <qrunnable.h>
#include <qthreadpool.h>


class SimulatorThread;
class BridgeObject : public QObject {
	Q_OBJECT

		QThreadPool thread_pool_;
	QMutex lock_;
public:
void update_sim_results(int group_id, int thread_id, int iteration,
	double seperation_constant, double alignment_constant, double cluster_constant, double explore_constant,
	double seperation_distance, double simultaneous_sampling, double time_taken, double occlusion, double coverage);

//void start_thread(SimulatorThread* thread);

signals:
//void send_sim_results(int group_id, int thread_id, int iteration,
//	double seperation_constant, double alignment_constant, double cluster_constant, double explore_constant,
//	double seperation_distance, double simultaneous_sampling, double time_taken, double occlusion, double coverage);
void send_sim_results(int group_id, int thread_id, int iteration);
};

class SimulatorThread :
	public QObject, public QRunnable
{
	Q_OBJECT;
private:
	//double separation_constant_;
	//double alignment_constant_;
	//double cluster_constant_;
	//double explore_constant_;

	//double separation_distance_;
	//double sensor_range_;
	//int discovery_range_;


	//double max_time_taken_;

	//double square_radius_;
	//double bounce_function_power_;
	//double bounce_function_multiplier_;

	//int formation_;


	//unsigned int grid_resolution_;
	//float grid_length_;
	//int grid_resolution_per_side_;
	//
	//int no_of_robots_;
	//double interior_scale_;
	//glm::vec3 interior_offset_;
	//glm::mat4 model_rotation_;
	//std::string interior_model_filename_;

	SwarmOccupancyTree* occupancy_grid_;
	SwarmCollisionTree* collision_grid_;

	std::vector<Robot*> robots_;
	UniformLocations uniform_locations_;


	int time_step_count_;
	int group_id_;
	int thread_id_;
	Swarm3DReconTree* recon_grid_;
	static const std::string DEFAULT_INTERIOR_MODEL_FILENAME;
	static const int DEFAULT_NO_OF_ROBOTS;
	static const std::string OCCUPANCY_GRID_NAME;
	static const std::string OCCUPANCY_GRID_OVERLAY_NAME;
	static const int OCCUPANCY_GRID_HEIGHT;

	bool aborted_;
	int iteration_;
	SwarmParams swarm_params_;

	std::unordered_map<int, int> death_map_;

	//BridgeObject* bridge_;
public:



	//explicit SimulatorThread(BridgeObject* bridge, int group_id, int thread_id, int no_of_robots, unsigned int grid_resolution, float grid_length, 
	//	std::string interior_filename, double interior_scale, glm::vec3 interior_offset, glm::mat4 interior_rotation, 
	//	int max_time_taken,  double separation_constant, double alignment_constant,
	//	double cluster_constant, double explore_constant, double sensor_range,
	//	int discovery_range, double separation_distance, Formation formation, 
	//	double square_radius, double bounce_function_power, double bounce_function_multipler, int iteration);

	explicit SimulatorThread(int group_id, int thread_id, int iteration, SwarmParams& swarm_params);

	void init();
	//void create_robots();
	virtual ~SimulatorThread();
	void reset_sim();
	//void derive_floor_plan(const VertexBufferData& bufferdata, float scale, const glm::vec3& offset);
	//void load_interior_model();
	//std::vector<glm::vec3> create_starting_formation(Formation type);

	void cleanup();
	void finish_work();
	void run() override;
	void abort();
	//void do_work();

	//bool intersect(const cv::Vec3f& n, float d,
	//	const cv::Vec3f& a, const cv::Vec3f& b, cv::Vec3f& intersection_pt) const;


//void set_no_of_robots(int no_of_robots);
//	void set_separation_distance(double distance);
//	void set_grid_resolution(int grid_resolution);
//void set_grid_length(int grid_length);
//
//void set_interior_scale(double scale);
//void set_interior_offset(glm::vec3 offset);
//
//void set_exploration_constant(double constant);
//void set_separation_constant(double constant);
//void set_alignment_constant(double constant);
//void set_cluster_constant(double constant);
//
//void set_model_rotation(double x_rotation, double y_rotation, double z_rotation);
//void set_square_formation_radius(double radius);
//void set_bounce_function_power(double bounce_function_power);
//void set_bounce_function_multiplier(double bounce_function_multiplier);

void reset_sim(SwarmParams& swarm_params);

//void set_model_filename(QString filename);
//
//void set_formation(int formation);

//void set_separation_range(double min, double max);
//void set_alignment_range(double min, double max);
//void set_cluster_range(double min, double max);
//void set_perimeter_range(double min, double max);
//void set_explore_range(double min, double max);
//
//void set_obstacle_avoidance_near_range(double min, double max);
//void set_obstacle_avoidance_far_range(double min, double max);

//void set_sensor_range(double sensor_range);
//void set_discovery_range(int discovery_range);
//void set_neighborhood_count(int count);

//void set_magic_k(double magic_k);
//void set_should_render(int render);
//void set_slow_down(int slow_down);

//void set_collide_with_robots(int collide);

signals:

//void send_sim_results(int group_id, int thread_id, int iteration);
//void send_sim_results(int group_id, int thread_id, int iteration,
//	double seperation_constant, double alignment_constant, double cluster_constant, double explore_constant,
//	double seperation_distance, double simultaneous_sampling, double time_taken, double occlusion, double coverage);

void send_sim_results(int group_id, int thread_id, int iteration, SwarmParams params, OptimizationResults results);
};

