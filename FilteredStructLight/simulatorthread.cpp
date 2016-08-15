#include "simulatorthread.h"
#include "swarmutils.h"

#define PI 3.14159265

const std::string SimulatorThread::DEFAULT_INTERIOR_MODEL_FILENAME = "interior/l-shape-floor-plan.obj";

const int SimulatorThread::DEFAULT_NO_OF_ROBOTS = 3;
const std::string SimulatorThread::OCCUPANCY_GRID_NAME = "occupancy_grid";
const std::string SimulatorThread::OCCUPANCY_GRID_OVERLAY_NAME = "occupancy_grid_overlay";
const int SimulatorThread::OCCUPANCY_GRID_HEIGHT = 2;


void BridgeObject::update_sim_results(int group_id, int thread_id, int iteration, 
	double separation_constant, double alignment_constant, double cluster_constant, double explore_constant, 
	double separation_distance, double simultaneous_sampling, double time_taken, double occlusion, double coverage) {
	//emit send_sim_results(group_id, thread_id, iteration, separation_constant, alignment_constant, cluster_constant, explore_constant,
	//	separation_distance, simultaneous_sampling, time_taken, occlusion, coverage);
	lock_.lock();
	emit send_sim_results(group_id, thread_id, iteration);
	lock_.unlock();
}

//void BridgeObject::start_thread(SimulatorThread* sim_thread) {
//		sim_thread->reset_sim();
//		thread_pool_.start(sim_thread);
//}

//SimulatorThread::SimulatorThread(BridgeObject* bridge, int group_id, int thread_id, int no_of_robots, unsigned int grid_resolution, float grid_length, 
//		std::string interior_filename, double interior_scale, glm::vec3 interior_offset, glm::mat4 interior_rotation, 
//		int max_time_taken, 
//		double separation_constant, double alignment_constant,
//		double cluster_constant, double explore_constant, double sensor_range,
//		int discovery_range, double separation_distance, Formation formation, 
//		double square_radius, double bounce_function_power, double bounce_function_multipler, int iteration) :
//
//		separation_constant_(separation_constant), alignment_constant_(alignment_constant), cluster_constant_(cluster_constant), explore_constant_(explore_constant), 
//		separation_distance_(separation_distance), sensor_range_(sensor_range), discovery_range_(discovery_range), 
//		max_time_taken_(max_time_taken), square_radius_(square_radius),
//		bounce_function_power_(bounce_function_power), 
//		bounce_function_multiplier_(bounce_function_multipler), formation_(static_cast<Formation>(formation)),
//		grid_resolution_(grid_resolution), grid_length_(grid_length), no_of_robots_(no_of_robots),
//		occupancy_grid_(nullptr), collision_grid_(nullptr), group_id_(group_id), thread_id_(thread_id), iteration_(iteration), 
//		interior_scale_(interior_scale), interior_offset_(interior_offset),
//		model_rotation_(interior_rotation), interior_model_filename_(interior_filename),
//		bridge_(bridge)
//{
//	//reset_sim();
//}


SimulatorThread::~SimulatorThread()
{
	cleanup();
}



void SimulatorThread::reset_sim() {
	reset_sim(swarm_params_);

}

void SimulatorThread::reset_sim(SwarmParams& swarm_params) {
	cleanup();

	aborted_ = false;
	time_step_count_ = 0;

	occupancy_grid_ = new SwarmOccupancyTree(swarm_params.grid_length_, swarm_params.grid_resolution_);
	swarm_params.grid_resolution_per_side_ = occupancy_grid_->get_grid_resolution_per_side();
	collision_grid_ = new SwarmCollisionTree(swarm_params.grid_resolution_);
	recon_grid_ = new Swarm3DReconTree(swarm_params.grid_resolution_, swarm_params.grid_length_);


	VertexBufferData* vertex_buffer_data;
	SwarmUtils::load_interior_model(swarm_params, vertex_buffer_data, occupancy_grid_, recon_grid_);

	occupancy_grid_->create_perimeter_list();
	occupancy_grid_->create_empty_space_list();
	occupancy_grid_->create_interior_list();

	bool render_ = false;
	QGLShaderProgram* m_shader = nullptr;

	SwarmUtils::create_robots(swarm_params, death_map_, occupancy_grid_, collision_grid_, recon_grid_, uniform_locations_, m_shader, render_, robots_);

	// assumption - global position of other robots are known
	for (auto& robot : robots_) {
		robot->update_robots(robots_);
		//if (render_) {
		//	robot->set_show_forces(show_forces_);
		//}
	}
}

void SimulatorThread::finish_work() {
	std::random_device rd;
	std::mt19937 mt(rd());
	//std::normal_distribution<> normal_distribution(0.0, 1.0);
	std::uniform_real_distribution<> uniform_real_distribution(0, 1);

	OptimizationResults results;

	results.occlusion = SwarmUtils::calculate_occulusion_factor(robots_);
	results.multi_samping = recon_grid_->calculate_multi_sampling_factor();
	results.density = recon_grid_->calculate_density();
	results.time_taken = time_step_count_;
	results.simul_sampling = occupancy_grid_->calculate_simultaneous_sampling_factor();

	//emit send_sim_results(group_id_, thread_id_, iteration_, separation_constant_, alignment_constant_, cluster_constant_, explore_constant_,
	//	separation_distance_, simultaneous_sampling, time_step_count_, occlusion, coverage);
	emit send_sim_results(group_id_, thread_id_, iteration_, swarm_params_, results);

	abort();
}
	

void SimulatorThread::run() {
	//finish_work();
	while (!aborted_) {
		if (time_step_count_ > swarm_params_.max_time_taken_) {
			finish_work();
		}
		if (occupancy_grid_->no_of_unexplored_cells() > 0) {

		}
		else {
			finish_work();
		}
		try {
			for (auto& robot : robots_) {
				robot->update(time_step_count_);
			}
		} catch (OutOfGridBoundsException& ex) {
			time_step_count_ = swarm_params_.max_time_taken_;
			finish_work();
		}
		time_step_count_++;
		//QCoreApplication::processEvents();
	}
	std::cout << "Ending : " << group_id_ << " " << thread_id_ << " " << iteration_ << "\n";
}

void SimulatorThread::abort() {
	aborted_ = true;
}

SimulatorThread::SimulatorThread(int group_id, int thread_id, int iteration, SwarmParams& swarm_params) :
occupancy_grid_(nullptr), collision_grid_(nullptr), time_step_count_(0), group_id_(group_id), thread_id_(thread_id), recon_grid_(nullptr), aborted_(false), iteration_(iteration), swarm_params_(swarm_params)
{
}

void SimulatorThread::init() {
	aborted_ = false;
	time_step_count_ = 0;
}

void SimulatorThread::cleanup() {
	//for (auto& vis_object : default_vis_objects_) {
	//	vis_object->clear_gpu_structs();
	//	delete vis_object;
	//}

	//for (auto& vis_object : reset_vis_objects_) {
	//	vis_object->clear_gpu_structs();
	//	delete vis_object;
	//}

	for (auto& vis_object : robots_) {
		vis_object->clear_gpu_structs();
		delete vis_object;
	}

	//for (auto& vis_object : lights_) {
	//	vis_object->clear_gpu_structs();
	//	delete vis_object;
	//}

	//default_vis_objects_.clear();
	//robot_color_map_.clear();
	//reset_vis_objects_.clear();
	robots_.clear();
	//lights_.clear();

	if (occupancy_grid_) {
		delete occupancy_grid_;
	}
	if (collision_grid_) {
		delete collision_grid_;
	}
	if (recon_grid_) {
		delete recon_grid_;
	}
	
}

//void SimulatorThread::set_no_of_robots(int no_of_robots) {
//	no_of_robots_ = no_of_robots;
//}
//
//void SimulatorThread::set_separation_distance(double distance) {
//	separation_distance_ = distance;
//}
//
//void SimulatorThread::set_grid_resolution(int grid_resolution) {
//	//grid_resolution_ = std::pow(std::pow(2, grid_resolution), 3);
//	grid_resolution_ = grid_resolution;
//}
//
//void SimulatorThread::set_grid_length(int grid_length) {
//	grid_length_ = grid_length;
//}
//
//void SimulatorThread::set_interior_scale(double scale) {
//	interior_scale_ = scale;
//}
//
//void SimulatorThread::set_interior_offset(glm::vec3 offset) {
//	interior_offset_ = offset;
//}
//
//void SimulatorThread::set_exploration_constant(double constant) {
//	explore_constant_ = constant;
//}
//
//void SimulatorThread::set_separation_constant(double constant) {
//	separation_constant_ = constant;
//}
//
//void SimulatorThread::set_alignment_constant(double constant) {
//	alignment_constant_ = constant;
//}
//
//void SimulatorThread::set_cluster_constant(double constant) {
//	cluster_constant_ = constant;
//}

//void SimulatorThread::set_model_rotation(double x_rotation, double y_rotation, double z_rotation) {
//	glm::mat4 rotateX = glm::rotate(glm::mat4(1.f), glm::radians((float)x_rotation), glm::vec3(1.f, 0.f, 0.f));
//	glm::mat4 rotateY = glm::rotate(rotateX, glm::radians((float)y_rotation), glm::vec3(0.f, 1.f, 0.f));
//	glm::mat4 rotateZ = glm::rotate(rotateY,  glm::radians((float)z_rotation), glm::vec3(0.f, 0.f, 1.f));
//	
//	model_rotation_ = rotateZ;
//}

//void SimulatorThread::set_square_formation_radius(double radius) {
//	square_radius_ = radius;
//}
//
//void SimulatorThread::set_bounce_function_power(double bounce_function_power) {
//	bounce_function_power_ = bounce_function_power;
//}
//
//void SimulatorThread::set_bounce_function_multiplier(double bounce_function_multiplier) {
//	bounce_function_multiplier_ = bounce_function_multiplier;
//}
//
//void SimulatorThread::set_sensor_range(double sensor_range) {
//	sensor_range_ = sensor_range;
//}
//
//void SimulatorThread::set_discovery_range(int discovery_range) {
//	discovery_range_ = discovery_range;
//}
//
//void SimulatorThread::set_formation(int formation) {
//	formation_ = formation;
//}
//
////void SimulatorThread::set_show_forces(int show) {
////	show_forces_ = show;
////	for (auto& robot : robots_) {
////		robot->set_show_forces(show_forces_);
////	}
////}
//
//void SimulatorThread::set_model_filename(QString filename) {
//	interior_model_filename_ = filename.toStdString();
//}
