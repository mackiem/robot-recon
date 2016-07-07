#include "swarmviewer.h"
#include <random>
#include <glm/vec3.hpp> // glm::vec3
#include <glm/vec4.hpp> // glm::vec4
#include <glm/mat4x4.hpp> // glm::mat4
#include <glm/gtc/matrix_transform.hpp> // glm::translate, glm::rotate, glm::scale, glm::perspective
#include <glm/gtc/type_ptr.hpp>
#include <chrono>
#include "swarmutils.h"
#include "swarmopt.h"
#include <QtCore/qcoreapplication.h>
#include "experimentalrobot.h"


//const std::string SwarmViewer::DEFAULT_INTERIOR_MODEL_FILENAME = "interior/house interior.obj";
//const std::string SwarmViewer::DEFAULT_INTERIOR_MODEL_FILENAME = "interior/box-floor-plan.obj";

const std::string SwarmViewer::DEFAULT_INTERIOR_MODEL_FILENAME = "interior/l-shape-floor-plan.obj";

const int SwarmViewer::DEFAULT_NO_OF_ROBOTS = 3;

#define NO_OF_LIGHTS 2
#define NO_OF_ROBOTS 10
#define PI 3.14159265

const std::string SwarmViewer::OCCUPANCY_GRID_NAME = "occupancy_grid";
const std::string SwarmViewer::OCCUPANCY_GRID_OVERLAY_NAME = "occupancy_grid_overlay";
const int SwarmViewer::OCCUPANCY_GRID_HEIGHT = 2;


void SwarmViewer::Light::update(glm::mat4 global_model) {
	glm::vec3 initial_position(0.f);
	glm::mat4 model = glm::translate(glm::mat4(1.f), (position_));
	glm::vec4 transformed_position = global_model * model * glm::vec4(initial_position, 1.f);
	
	glm::vec3 world_position =
		glm::vec3(transformed_position.x, transformed_position.y, transformed_position.z) / transformed_position.w;

	glUniform3fv(light_position_location_, 1, glm::value_ptr(world_position));
}

RobotWorker::RobotWorker() : aborted_(false), paused_(false), sampling_updated_(false) {
	//refresh_rate_ = 1.f / 30.f;
	//accumulator_ = 0.f;
	
};

void RobotWorker::set_robots(std::vector<Robot*> robots) {
	robots_ = robots;
}

void RobotWorker::set_occupancy_tree(SwarmOccupancyTree* occupancy_grid) {
	occupancy_grid_ = occupancy_grid;
}
void RobotWorker::set_slow_down(int slow_down) {
	slow_down_ = slow_down;
}
void RobotWorker::abort() {
	aborted_ = true;
}

void RobotWorker::set_max_time_taken(int max_time_taken) {
	max_time_taken_ = max_time_taken;
}

double RobotWorker::calculate_coverage() {
	double coverage = 0.0;
	for (auto& robot : robots_) {
		coverage += robot->calculate_coverage();
	}
	if (robots_.size() > 0) {
		coverage /= robots_.size();
	}
	return coverage;
}

void RobotWorker::finish_work() {
	if (!sampling_updated_) {
		double simultaneous_sampling = occupancy_grid_->calculate_multi_sampling_factor();
		double coverage = calculate_coverage();
		//std::cout << "End - time steps : " << time_step_count_ << "  " <<
		//	occupancy_grid_->get_unexplored_perimeter_list().size() << "\n";
		emit update_sampling(simultaneous_sampling);
		emit update_sim_results(time_step_count_, simultaneous_sampling, coverage);
		sampling_updated_ = true;
		abort();
	}
}
	

void RobotWorker::do_work() {
	while (!aborted_) {
		if (step_count_ == 0) {
			paused_ = true;
		}
		if (!paused_) {
			if (slow_down_) {
				QThread::currentThread()->msleep(30);
			}
			step_count_--;
			if (time_step_count_ > max_time_taken_) {
				finish_work();
			}

			if (occupancy_grid_->no_of_unexplored_cells() > 0) {
				emit update_time_step_count(time_step_count_);
			} else {
				finish_work();
			}
		}
		if (!paused_) {
			//std::chrono::milliseconds current_timestamp = std::chrono::duration_cast<std::chrono::milliseconds>(
			//	std::chrono::system_clock::now().time_since_epoch());

			//if (last_updated_time_ < 0) {
			//	last_updated_time_ = current_timestamp.count();
			//	continue;
			//}

			//const float delta_time = 2500; // in ms

			//accumulator_ += current_timestamp.count() - last_updated_time_;

			//while (accumulator_ >= delta_time) {
				for (auto& robot : robots_) {
					robot->update(time_step_count_);
				}
				//accumulator_ -= delta_time;
				time_step_count_++;
			//}
		}
		QCoreApplication::processEvents();
	}
}

void RobotWorker::init() {
	sampling_updated_ = false;
	aborted_ = false;
	time_step_count_ = 0;
	step_count_ = -1;
	accumulator_ = 0.0;
	last_updated_time_ = -1;
}

void RobotWorker::pause() {
	paused_ = true;
}

void RobotWorker::resume() {
	paused_ = false;
	step_count_ = -1;
}

void RobotWorker::step() {
	if (paused_) {
		step_count_ = 1;
		paused_ = false;
	}
}

GridOverlay::GridOverlay(UniformLocations& locations, SwarmOccupancyTree* octree, 
	unsigned grid_resolution, float grid_length, std::map<int, cv::Vec4f> robot_color_map, QGLShaderProgram* shader) : 
	VisObject(locations), occupany_grid_(octree), grid_resolution_per_side_(grid_resolution), grid_length_(grid_length), robot_color_map_(robot_color_map), shader_(shader)  {
	
	fill_color_.resize(6);
	for (int i = 0; i < 6; ++i) {
		fill_color_[i] = cv::Vec4f(160.f, 32.f, 240.f, 255.f);
		fill_color_[i] /= 255.f;
	}
	create_mesh(true);
}

void GridOverlay::create_mesh(bool initialize) {

	if (initialize) {
		int grid_length_per_side = grid_resolution_per_side_ / 2;
		int y = SwarmViewer::OCCUPANCY_GRID_HEIGHT;

		VertexBufferData bufferdata;

		cv::Vec4f explored_color(1.f, 1.f, 1.f, 1.f);
		cv::Vec4f unexplored_color(.86f, 0.08f, .24f, 1.f);

		cv::Vec3f normal(0.f, 1.f, 0.f);

		//for (int x = -1 * grid_length_per_side; x < grid_length_per_side; ++x) {
		//	for (int z = -1 * grid_length_per_side; z < grid_length_per_side; ++z) {
		for (int x = 0; x < grid_resolution_per_side_; ++x) {
			for (int z = 0; z < grid_resolution_per_side_; ++z) {

				float x_pt = x * grid_length_;
				float y_pt = y * grid_length_;
				float z_pt = z * grid_length_;

				cv::Vec3f pt_1 = cv::Vec3f(x * grid_length_, y, z * grid_length_);
				cv::Vec3f pt_2 = cv::Vec3f(x * grid_length_, y, (z + 1) * grid_length_);
				cv::Vec3f pt_3 = cv::Vec3f((x + 1) * grid_length_, y, (z + 1) * grid_length_);
				cv::Vec3f pt_4 = cv::Vec3f((x + 1) * grid_length_, y, z * grid_length_);

				auto& positions = bufferdata.positions;
				positions.push_back(pt_1);
				positions.push_back(pt_2);
				positions.push_back(pt_3);
				positions.push_back(pt_3);
				positions.push_back(pt_4);
				positions.push_back(pt_1);

				auto& indices = bufferdata.indices;

				//glm::ivec3 current_position(x + grid_length_per_side, 0, z + grid_length_per_side);
				glm::ivec3 current_position(x, 0, z);
				cv::Vec4f color = unexplored_color;
				int explored_robot = occupany_grid_->explored_by(current_position);

				if (explored_robot >= 0) {
					color = robot_color_map_[explored_robot];
				}

				for (auto i = 0u; i < 6; ++i) {
					bufferdata.colors.push_back(color);
					bufferdata.normals.push_back(normal);
					indices.push_back((positions.size() - 6) + i);
				}
			}
		}
		bufferdata.count.push_back(bufferdata.positions.size());
		bufferdata.base_index.push_back(0);
		bufferdata.offset.push_back(0);

		if (initialize) {
			RenderMesh grid_overlay_mesh;
			RenderEntity grid_overlay(GL_TRIANGLES, shader_);
			grid_overlay.upload_data_to_gpu(bufferdata);
			grid_overlay_mesh.push_back(grid_overlay);

			mesh_ = grid_overlay_mesh;

		}
		else {
			mesh_[0].upload_data_to_gpu(bufferdata, true);
		}
	}

}

void GridOverlay::update(glm::mat4 global_model) {
	//create_mesh(false);
}

void GridOverlay::update_grid_position(const glm::ivec3& position, const cv::Vec4f& color) {
	RenderEntity& entity = mesh_[0];

	std::vector<cv::Vec4f> fill_color(6);
	std::fill(fill_color.begin(), fill_color.end(), color);

	glBindVertexArray(entity.vao_);
	glBindBuffer(GL_ARRAY_BUFFER, entity.vbo_[RenderEntity::COLOR]);
	glBufferSubData(GL_ARRAY_BUFFER, 6 * ( (position.x * grid_resolution_per_side_) + position.z) * sizeof(cv::Vec4f), 
		6 * sizeof(cv::Vec4f), &fill_color[0]);

	glBindVertexArray(0);

}

void GridOverlay::update_grid_position(const glm::ivec3& position) {
	RenderEntity& entity = mesh_[0];


	glBindVertexArray(entity.vao_);
	glBindBuffer(GL_ARRAY_BUFFER, entity.vbo_[RenderEntity::COLOR]);
	glBufferSubData(GL_ARRAY_BUFFER, 6 * ( (position.x * grid_resolution_per_side_) + position.z) * sizeof(cv::Vec4f), 
		6 * sizeof(cv::Vec4f), &fill_color_[0]);

	glBindVertexArray(0);

}

void SwarmViewer::load_inital_models() {

	//reset_sim();
	//create_occupancy_grid_overlay(grid_resolution_per_side_, grid_length_, true);
}

void SwarmViewer::initialize_position() {

	std::random_device rd;
	std::mt19937 rng(rd());
	std::uniform_int_distribution<int> gen(0, 100); // uniform, unbiased
	int r = gen(rng);
	
	//for (int i = 0; i < default_scene_.size(); ++i) {
	//	default_scene_.push_back(assets_[ROBOT]);
	//}
}

void SwarmViewer::set_shaders() {
	set_shader_paths(":/FilteredStructLight/swarm_vert.glsl", ":/FilteredStructLight/swarm_frag.glsl");
}



void SwarmViewer::derive_floor_plan(const VertexBufferData& bufferdata, float scale, const glm::vec3& offset) {
	std::vector<std::vector<cv::Vec3f>> triangle_list;
	cv::Vec3f vertex_offset(offset.x, offset.y, offset.z);
	glm::mat3 rot_mat = glm::mat3(model_rotation_);
	cv::Mat rotation_mat = convert_mat(rot_mat);

	for (size_t i = 0; i < bufferdata.count.size(); ++i) {
		// get triangle vertices

		int index_offset = bufferdata.base_index[i];
		int base_vertex = bufferdata.offset[i];
		std::vector<cv::Vec3f> triangle;
		for (size_t j = 0; j < bufferdata.count[i]; ++j) {
			int index = bufferdata.indices[index_offset + j];
			cv::Mat rotated_point_matrix = rotation_mat * cv::Mat(bufferdata.positions[base_vertex + index]);
			cv::Vec3f rotated_point((float*)rotated_point_matrix.datastart);
			cv::Vec3f triangle_point = scale * (rotated_point + vertex_offset);
			triangle.push_back(triangle_point);
			if ((j+1) % 3 == 0 && j > 0) {
				triangle_list.push_back(triangle);
				triangle.clear();
			}
		}
	}

	// get floor plane
	cv::Vec3f n(0.f, OCCUPANCY_GRID_HEIGHT, 0.f);
	float d = OCCUPANCY_GRID_HEIGHT * OCCUPANCY_GRID_HEIGHT;

	// for each triangle, check if 2 sides intersect with the plane,
	// then rasterize that line to the grid (bresenham line algorithm)
	for (auto& triangle : triangle_list) {
		assert(triangle.size() == 3);
		std::vector<glm::vec3> intersection_points;
		for (int i = 0; i < 3; ++i) {
			cv::Vec3f a = triangle[i];
			int b_index = (i == 2) ? 0 : i + 1;
			cv::Vec3f b = triangle[b_index];

			cv::Vec3f intersection_pt;
			bool does_intersect = intersect(n, d, a, b, intersection_pt);

			if (does_intersect) {
				intersection_points.push_back(glm::vec3(intersection_pt[0], intersection_pt[1],
					intersection_pt[2]));
			}

			if ((a[1] > OCCUPANCY_GRID_HEIGHT && b[1] < OCCUPANCY_GRID_HEIGHT)
				|| (a[1] < OCCUPANCY_GRID_HEIGHT && b[1] > OCCUPANCY_GRID_HEIGHT)) {
				std::string result = (does_intersect) ? "true" : "false";
				//std::cout << "Does intersect. Algorithm output : " <<  result << std::endl;
				assert(does_intersect);
			}
		}
			


		assert(intersection_points.size() == 0 || intersection_points.size() == 2);
		if (intersection_points.size() == 2) {
			occupancy_grid_->mark_interior_line(intersection_points[0],
				intersection_points[1]);
		}
	}
	int i = 0;
}

bool SwarmViewer::intersect(const cv::Vec3f& n, float d, 
	const cv::Vec3f& a, const cv::Vec3f& b, cv::Vec3f& intersection_pt) const {

	cv::Vec3f v = b - a;

	float denominator = n.dot(v);
	if (std::abs(denominator) < 1e-6) {
		return false;
	}

	float numerator = d - n.dot(a);
	float t = numerator / denominator;

	if (t >= 0 && t <= 1) {
		intersection_pt = a + t * v;
		return true;
	}
	return false;
}

void SwarmViewer::quad_tree_test() {
	//int resolution = 5;
	//int empty_value = -1;
	//quadtree_ = std::make_shared<mm::Quadtree<int>>(resolution, empty_value);

	//int resolution_per_side = std::pow(std::pow(4, 5), 0.5);

	//for (int x = 0; x < resolution_per_side; ++x) {
	//	for (int y = 0; y < resolution_per_side; ++y) {
	//		int empty_return_value = quadtree_->at(x, y);
	//		assert(empty_return_value == empty_value);

	//		int some_value = x + y;
	//		quadtree_->set(x, y, some_value);

	//		int return_value = quadtree_->at(x, y);
	//		assert(some_value == return_value);

	//		quadtree_->unset(x, y);
	//		int return_empty_value = quadtree_->at(x, y);
	//		assert(empty_return_value == empty_value);
	//	}
	//}

	//// do some random query tests
	//for (int x = 0; x < resolution_per_side; ++x) {
	//	for (int y = 0; y < resolution_per_side; ++y) {

	//		int some_value = x + y;
	//		quadtree_->set(x, y, some_value);
	//	}
	//}

	//std::random_device rd;
	//std::mt19937 rng(rd());
	//std::uniform_int_distribution<int> random_coords(0, resolution_per_side - 1);

	//for (int i = 0; i < 1000; ++i) {
	//	int x = random_coords(rng);
	//	int y = random_coords(rng);

	//	int return_value = quadtree_->at(x, y);
	//	assert(return_value == x + y);
	//}

	//
	//quadtree_.reset();
}

void SwarmViewer::load_interior_model() {
	cv::Mat unit_matrix = cv::Mat::eye(4, 4, CV_64F);

	std::string model_filename = (interior_model_filename_.compare("") == 0) ? DEFAULT_INTERIOR_MODEL_FILENAME
		: interior_model_filename_;

	VertexBufferData* vertex_buffer_data = new VertexBufferData();
	load_obj(model_filename,  *vertex_buffer_data);

	float scale = interior_scale_;

	if (render_) {
		cv::Vec4f color(0.8f, 0.5f, 0.5f, 1.f);
		vertex_buffer_data->colors = std::vector<cv::Vec4f>(vertex_buffer_data->positions.size(), color);

		RenderEntity interior_model(GL_TRIANGLES, &m_shader);
		//interior_model.upload_data_to_gpu(vertices, colors, normals, uvs, indices, count, offset, base_index);
		interior_model.upload_data_to_gpu(*vertex_buffer_data);


		RenderMesh interior_model_mesh;
		interior_model_mesh.push_back(interior_model);

		VisObject* interior_model_object = new VisObject(uniform_locations_);
		interior_model_object->mesh_ = interior_model_mesh;


		glm::mat4 model = glm::scale(glm::mat4(1.f), glm::vec3(scale, scale, scale));
		model = glm::translate(model, interior_offset_);
		model = model * model_rotation_;
		update_model(interior_model_object->mesh_, model);

		default_vis_objects_.push_back(interior_model_object);
	}


	if (interior_model_filename_.compare("") > 0) {
		derive_floor_plan(*vertex_buffer_data, scale, interior_offset_);
	}

	delete vertex_buffer_data;
}

void SwarmViewer::change_to_top_down_view() {
	look_at_z_ = 0.f;
	look_at_x_ = 0.f;
	look_at_y_ = 0.f;

	m_xRot = 0.f;
	m_yRot = 0.f;
	m_zRot = 0.f;

	camera_distance_ = 0.f;
	float length = grid_resolution_per_side_ * grid_length_;
	glm::vec3 mid_point(length / 2.f, 0.f, length/2.f);

	center_ = mid_point;
	eye_ = mid_point;
	float distance = (length / 2.f) / std::tan(glm::radians(fovy_ /2.f));
	eye_.y = distance;

	up_ = glm::vec3(1.f, 0.f, 0.f);
	
}

void SwarmViewer::update_perimiter_positions_in_overlay() {
	auto perimeter_list = occupancy_grid_->get_unexplored_perimeter_list();
	cv::Vec4f dark_green(60.f, 179.f, 113.f, 255.f);
	dark_green /= 255.f;
	for (auto& perimeter_pos : perimeter_list) {
		overlay_->update_grid_position(perimeter_pos, dark_green);
	}
}

void SwarmViewer::custom_init_code() {
	timer_ = new QTimer(this);
	connect(timer_, SIGNAL(timeout()), this, SLOT(update()));
	timer_->start(5);


	paused_ = false;

	glEnable(GL_CULL_FACE);
	glLineWidth(2);

	look_at_z_ = 600.f;
	look_at_x_ = 345.f;
	look_at_y_ = 245.f;

	eye_ = glm::vec3(look_at_x_, look_at_y_, look_at_z_);

	model_loc_ = m_shader.uniformLocation("model");
	inverse_transpose_loc_ = m_shader.uniformLocation("inverse_transpose_model");
	mvp_loc_ = m_shader.uniformLocation("mvp");
	view_position_loc_ = m_shader.uniformLocation("view_position");

	uniform_locations_.model_loc_ = model_loc_;
	uniform_locations_.inverse_transpose_loc_ = inverse_transpose_loc_;
	uniform_locations_.mvp_loc_ = mvp_loc_;

}

void SwarmViewer::custom_draw_code() {
	if (render_) {
		glm::mat4 mvp;

		glUniform3fv(view_position_loc_, 1, glm::value_ptr(eye_));

		draw_scene(default_scene_);

		if (show_interior_) {
			for (auto& vis_object : default_vis_objects_) {
				vis_object->update(model_);
				vis_object->draw(model_, camera_, projection_);
			}
		}

		for (auto& vis_object : reset_vis_objects_) {
			vis_object->update(model_);
			vis_object->draw(model_, camera_, projection_);
		}

		for (auto& robot : robots_) {
			robot->update_visualization_structs();
			robot->draw(model_, camera_, projection_);
		}

		for (auto& light : lights_) {
			light->update(model_);
		}
	}

}

void SwarmViewer::draw_mesh(RenderMesh& mesh) {
}

void SwarmViewer::shutdown_worker() {
	if (robot_update_thread_.isRunning()) {
		disconnect(this, &SwarmViewer::physics_thread_pause, robot_worker_, &RobotWorker::pause);
		disconnect(this, &SwarmViewer::physics_thread_resume, robot_worker_, &RobotWorker::resume);
		disconnect(this, &SwarmViewer::physics_thread_step, robot_worker_, &RobotWorker::step);
		disconnect(robot_worker_, &RobotWorker::update_time_step_count, this, &SwarmViewer::update_time_step_count);
		disconnect(robot_worker_, &RobotWorker::update_sampling, this, &SwarmViewer::update_sampling);
		disconnect(robot_worker_, &RobotWorker::update_sim_results, this, &SwarmViewer::update_sim_results);
		robot_worker_->abort();
		robot_update_thread_.quit();
		robot_update_thread_.wait();
		delete robot_worker_;
	}
	
}

SwarmViewer::~SwarmViewer() {
	shutdown_worker();
	cleanup();
	timer_->stop();
}

void SwarmViewer::cleanup() {
	for (auto& vis_object : default_vis_objects_) {
		vis_object->clear_gpu_structs();
		delete vis_object;
	}

	for (auto& vis_object : reset_vis_objects_) {
		vis_object->clear_gpu_structs();
		delete vis_object;
	}

	for (auto& vis_object : robots_) {
		vis_object->clear_gpu_structs();
		delete vis_object;
	}

	for (auto& vis_object : lights_) {
		vis_object->clear_gpu_structs();
		delete vis_object;
	}

	default_vis_objects_.clear();
	robot_color_map_.clear();
	reset_vis_objects_.clear();
	robots_.clear();
	lights_.clear();

	if (occupancy_grid_) {
		delete occupancy_grid_;
	}
	if (collision_grid_) {
		delete collision_grid_;
	}
	
}

void SwarmViewer::reset_sim() {
	makeCurrent();
	shutdown_worker();
	cleanup();

	//set_model_rotation(-90.f, 0.f, -90.f);
	sim_results_updated_ = false;
	step_count_ = -1;
	time_step_count_ = 0;

	render_ = gui_render_;

	occupancy_grid_ = new SwarmOccupancyTree(grid_length_, grid_resolution_);
	grid_resolution_per_side_ = occupancy_grid_->get_grid_resolution_per_side();
	collision_grid_ = new SwarmCollisionTree(grid_resolution_);


	load_interior_model();
	occupancy_grid_->create_perimeter_list();
	occupancy_grid_->create_empty_space_list();
	occupancy_grid_->create_interior_list();

	create_robots();
	// assumption - global position of other robots are known
	for (auto& robot : robots_) {
		robot->update_robots(robots_);
		if (render_) {
			robot->set_show_forces(show_forces_);
		}
	}




	// graphics setup
	change_to_top_down_view();
	if (render_) {
		create_occupancy_grid(grid_resolution_per_side_, grid_length_);
		robot_color_map_[occupancy_grid_->get_interior_mark()] = cv::Vec4f(1.f, 1.f, 1.f, 1.f);
		create_lights();

		overlay_ = new GridOverlay(uniform_locations_,
			occupancy_grid_, grid_resolution_per_side_, grid_length_, robot_color_map_, &m_shader);
		update_perimiter_positions_in_overlay();
		reset_vis_objects_.push_back(overlay_);
		for (auto& robot : robots_) {
			robot->set_grid_overlay(overlay_);
		}
	}

	robot_worker_ = new RobotWorker();
	robot_worker_->moveToThread(&robot_update_thread_);
	robot_worker_->init();
	connect(this, &SwarmViewer::physics_thread_pause, robot_worker_, &RobotWorker::pause);
	connect(this, &SwarmViewer::physics_thread_resume, robot_worker_, &RobotWorker::resume);
	connect(this, &SwarmViewer::physics_thread_step, robot_worker_, &RobotWorker::step);
	connect(robot_worker_, &RobotWorker::update_time_step_count, this, &SwarmViewer::update_time_step_count);
	connect(robot_worker_, &RobotWorker::update_sim_results, this, &SwarmViewer::update_sim_results);
	connect(robot_worker_, &RobotWorker::update_sampling, this, &SwarmViewer::update_sampling);
	connect(&robot_update_thread_, &QThread::started, robot_worker_, &RobotWorker::do_work);

	robot_worker_->set_robots(robots_);
	robot_worker_->set_occupancy_tree(occupancy_grid_);
	robot_worker_->set_slow_down(slow_down_);
	robot_worker_->set_max_time_taken(max_time_taken_);
	robot_update_thread_.start();
}

void SwarmViewer::set_show_forces(int show) {
	show_forces_ = show;
	for (auto& robot : robots_) {
		robot->set_show_forces(show_forces_);
	}
}

void SwarmViewer::set_model_filename(QString filename) {
	interior_model_filename_ = filename.toStdString();
}

void SwarmViewer::set_separation_range(double min, double max) {
	separation_range_ = Range(min, max);
	separation_distance_ = max * grid_length_;
}

void SwarmViewer::set_alignment_range(double min, double max) {
	alignment_range_ = Range(min, max);
}

void SwarmViewer::set_cluster_range(double min, double max) {
	cluster_range_ = Range(min, max);
}

void SwarmViewer::set_perimeter_range(double min, double max) {
	perimeter_range_ = Range(min, max);
}

void SwarmViewer::set_explore_range(double min, double max) {
	explore_range_ = Range(min, max);
}

void SwarmViewer::set_obstacle_avoidance_near_range(double min, double max) {
	obstacle_near_range_ = Range(min, max);

}

void SwarmViewer::set_obstacle_avoidance_far_range(double min, double max) {
	obstacle_far_range_ = Range(min, max);
}

void SwarmViewer::pause() {
	emit physics_thread_pause();
}

void SwarmViewer::step() {
	emit physics_thread_step();
}

void SwarmViewer::resume() {
	emit physics_thread_resume();
	paused_ = false;
	step_count_ = -1;
}

void SwarmViewer::run_least_squared_optimization() {
	int preferred_neighborhood_count = 5;

	SwarmOptimizer* optimizer_worker = new SwarmOptimizer();
	QThread* optimizer_thread = new QThread();

	connect(optimizer_thread, SIGNAL(started()), optimizer_worker, SLOT(optimize_swarm_params()));
	connect(optimizer_worker, SIGNAL(finished()), optimizer_thread, SLOT(quit()));
	connect(optimizer_thread, SIGNAL(finished()), optimizer_thread, SLOT(deleteLater()));
	connect(optimizer_worker, SIGNAL(finished()), optimizer_thread, SLOT(deleteLater()));


	optimizer_worker->set_optimize_swarm_params(this, separation_constant_, alignment_constant_, cluster_constant_, perimeter_constant_, explore_constant_,
		separation_range_, alignment_range_, cluster_range_, preferred_neighborhood_count);

	optimizer_worker->moveToThread(optimizer_thread);
	optimizer_thread->start();
}

void SwarmViewer::run_mcmc_optimization() {

	//int preferred_neighborhood_count = 5;

	ParallelMCMCOptimizer* optimizer_worker = new ParallelMCMCOptimizer();
	QThread* optimizer_thread = new QThread();

	////connect(optimizer_thread, SIGNAL(started()), optimizer_worker, SLOT(optimize_swarm_params()));
	connect(optimizer_thread, SIGNAL(started()), optimizer_worker, SLOT(run_optimizer()));
	//connect(optimizer_thread, SIGNAL(started()), optimizer_worker, SLOT(optimize_experimental_brute_force()));
	connect(optimizer_worker, SIGNAL(finished()), optimizer_thread, SLOT(quit()));
	connect(optimizer_thread, SIGNAL(finished()), optimizer_thread, SLOT(deleteLater()));
	connect(optimizer_worker, SIGNAL(finished()), optimizer_thread, SLOT(deleteLater()));


	optimizer_worker->set_viewer(this);
	//optimizer_worker->set_max_time_taken(max_time_taken_);

	optimizer_worker->moveToThread(optimizer_thread);
	optimizer_thread->start();

	//parallel_mcmc_optimizer_  = new ParallelMCMCOptimizer();
	//parallel_mcmc_optimizer_->set_viewer(this);
	//parallel_mcmc_optimizer_->run_optimizer();
	std::cout << "Optimizer started \n";
}

void SwarmViewer::set_magic_k(double magic_k) {
	magic_k_ = magic_k;
}

void SwarmViewer::set_should_render(int render) {
	gui_render_ = render;
}

void SwarmViewer::set_slow_down(int slow_down) {
	slow_down_ = slow_down;
}

void SwarmViewer::set_collide_with_robots(int collide) {
	collide_with_robots_ = collide;
}


void SwarmViewer::set_sensor_range(double sensor_range) {
	sensor_range_ = sensor_range;
}

void SwarmViewer::set_discovery_range(int discovery_range) {
	discovery_range_ = discovery_range;
}

void SwarmViewer::set_neighborhood_count(int count) {
	neighborhood_count_ = count;
}

void SwarmViewer::set_formation(int formation) {
	formation_ = formation;
}

void SwarmViewer::update_sim_results(double timesteps, double multi_sampling, double coverage) {
	time_steps_result_ = timesteps;
	multi_sampling_result_ = multi_sampling;
	coverage_result_ = coverage;
	sim_results_updated_ = true;
	//emit update_sim_results_to_optimizer(timesteps, multi_sampling, coverage);
}

void SwarmViewer::get_sim_results(double& timesteps, double& multi_sampling, double& coverage) {
	while (!sim_results_updated_) {
		QThread::currentThread()->msleep(10);
	}
	timesteps = time_steps_result_;
	multi_sampling = multi_sampling_result_;
	coverage = coverage_result_;
	sim_results_updated_ = false;
}

SwarmViewer::SwarmViewer(const QGLFormat& format, QWidget* parent) : RobotViewer(format, parent)
{

	quad_tree_test();
	connect(this, SIGNAL(optimizer_reset_sim()), this, SLOT(reset_sim()));
	occupancy_grid_ = nullptr;
	collision_grid_ = nullptr;
	render_ = false;
	max_time_taken_ = 20010.0;
	sim_results_updated_ = false;
	//square_radius_ = 4.f;


	//grid_ = VisObject(uniform_locations_);
	//grid_overlay_ = VisObject(uniform_locations_);
	//occupancy_grid_ = SwarmOctTree(grid_length_, grid_resolution_per_side_);
}

void SwarmViewer::create_light_model(RenderMesh& light_mesh) {
	VertexBufferData vb_data;
	load_obj("bulb/bulb.obj", vb_data);

	cv::Vec4f color(253.f, 184.f, 19.f, 255.f);
	color /= 255.f;
	for (auto& vertice : vb_data.positions) {
		vb_data.colors.push_back(color);
	}

	RenderEntity light_model(GL_TRIANGLES, &m_shader);
	light_model.upload_data_to_gpu(vb_data);

	light_model.set_model(glm::scale(glm::mat4(1.f), glm::vec3(10.f, 10.f, 10.f)));
	light_mesh.push_back(light_model);
}

void SwarmViewer::create_robot_model(RenderMesh& robot_mesh, cv::Vec4f color) {
	VertexBufferData vb_data;
	load_obj("PYROS/pyros-obj.obj", vb_data);

	//cv::Vec4f color(253.f, 184.f, 19.f, 255.f);
	//color /= 255.f;

	vb_data.colors.resize(vb_data.positions.size());
	for (auto& color_holder : vb_data.colors) {
		color_holder = color;
	}

	RenderEntity robot_model(GL_TRIANGLES, &m_shader);
	robot_model.upload_data_to_gpu(vb_data);
	robot_model.set_initial_model(glm::scale(glm::mat4(1.f), glm::vec3(.1f, .1f, .1f)));

	//robot_model.set_model(glm::scale(glm::mat4(1.f), glm::vec3(10.f, 10.f, 10.f)));
	robot_mesh.push_back(robot_model);
}

void SwarmViewer::create_lights() {

	std::random_device rd;
	std::mt19937 rng(rd());
	std::uniform_real_distribution<float> color(0.5, 1); // uniform, unbiased
	std::uniform_int_distribution<int> position(0, occupancy_grid_->get_grid_cube_length() 
		* occupancy_grid_->get_grid_resolution_per_side());

	m_shader.bind();

	RenderMesh light_bulb;
	create_light_model(light_bulb);

	for (size_t i = 0; i < NO_OF_LIGHTS; ++i) {
		Light* light = new Light(uniform_locations_);

		std::stringstream light_name;
		light_name << "lights[" << i << "]";

		std::stringstream light_position_name;
		light_position_name << light_name.str() << ".position";
		light->light_position_location_ = m_shader.uniformLocation(light_position_name.str().c_str());

		std::stringstream light_color_name;
		light_color_name << light_name.str() << ".color";
		light->light_color_location_ = m_shader.uniformLocation(light_color_name.str().c_str());

		//glm::vec3 light_position(0, 0, -100);
		glm::vec3 light_position(position(rng), 500.f, position(rng));
		glm::vec3 light_color(color(rng), color(rng), color(rng));

		light->color_ = light_color;
		light->position_ = light_position;

		glUniform3fv(light->light_color_location_, 1, glm::value_ptr(light_color));

		glm::mat4 RT = glm::translate(glm::mat4(1.f), light_position);
		RenderMesh light_mesh = light_bulb;
		update_model(light_mesh, RT);
		//default_scene_.push_back(light);
		 
		light->mesh_ = light_mesh;

		lights_.push_back(light);
	}
}


std::vector<glm::vec3> SwarmViewer::create_starting_formation(Formation type) {
	std::vector<glm::vec3> robot_positions;

	switch (type) {
	case SQUARE: {
		double side_length = square_radius_ * grid_length_ * 2;
		double distance_between_robots = side_length * 4.0 / (double)no_of_robots_;

		glm::vec3 robots_mid_point(side_length / 2.f, 0.f, side_length / 2.f);
		glm::vec3 grid_mid_point(grid_resolution_per_side_ / 2.0 * grid_length_, 0.f, grid_resolution_per_side_ / 2.0 * grid_length_);
		glm::vec3 offset = grid_mid_point - robots_mid_point;

		for (int i = 0; i < no_of_robots_; ++i) {
			double robot_position_on_line = i * distance_between_robots;
			int square_segment = std::floor(robot_position_on_line / side_length);
			double left_over_distance = std::fmod(robot_position_on_line, side_length);

			glm::vec3 square_starting_position;
			glm::vec3 added_distance;

			switch (square_segment) {
			case 0: {
				square_starting_position = glm::vec3(0.f, 0.f, 0.f);
				added_distance = glm::vec3(left_over_distance, 0.f, 0.f);
				break;
			}
			case 1: {
				square_starting_position = glm::vec3(side_length, 0.f, 0.f);
				added_distance = glm::vec3(0.f, 0.f, left_over_distance);
				break;
			}
			case 2: {
				square_starting_position = glm::vec3(side_length, 0.f, side_length);
				added_distance = glm::vec3(-left_over_distance, 0.f, 0.f);
				break;
			}
			case 3: {
				square_starting_position = glm::vec3(0.f, 0.f, side_length);
				added_distance = glm::vec3(0.f, 0.f, -left_over_distance);
				break;
			}
			default: {
				std::cout << "Something bad happened\n";
				break;
			}
			}

			glm::vec3 robot_position = square_starting_position + added_distance + offset;
			try {
				auto robot_grid_position = occupancy_grid_->map_to_grid(robot_position);
				if (!occupancy_grid_->is_out_of_bounds(robot_grid_position)
					&& !occupancy_grid_->is_interior(robot_grid_position)) {
					robot_positions.push_back(robot_position);
				}
			} catch (OutOfGridBoundsException& ex) {
				// ignore
			}
		}



		//int no_of_robots_per_side = ((no_of_robots_ - 4)/ 4.f) + 2;
		//glm::ivec3 robots_mid_point((no_of_robots_per_side / 2.f) - 1, 0, (no_of_robots_per_side / 2.f) - 1);
		//glm::ivec3 mid_point((grid_resolution_per_side_ / 2.f) - 1, 0, (grid_resolution_per_side_ / 2.f) - 1);
		//glm::ivec3 offset = mid_point - robots_mid_point;
		//
		//for (int x = 0; x < no_of_robots_per_side; ++x) {
		//	for (int z = 0; z < no_of_robots_per_side; z++) {
		//		if ((z == 0 || z == (no_of_robots_per_side - 1)) || (x == 0 || x == (no_of_robots_per_side - 1))) {

		//			//SwarmUtils::print_vector("init pos : ", glm::ivec3(x, 0, z));

		//			glm::ivec3 robot_grid_position = glm::ivec3(x, 0, z) + offset;

		//			glm::vec3 push_back_offset;
		//			glm::vec3 real_mid_point (grid_resolution_per_side_ * grid_length_ / 2.f, 0, grid_resolution_per_side_ * grid_length_ / 2.f);
		//			glm::vec3 vector_from_mid_point = glm::vec3(occupancy_grid_->map_to_position(robot_grid_position) - real_mid_point);
		//			auto length_from_mid_point = glm::length(vector_from_mid_point);
		//			
		//			if (length_from_mid_point > 1e-6) {
		//				float square_length = square_radius_ *  grid_length_;
		//				float cos_theta = 1.f;

		//				if ((x == 0 || x == (no_of_robots_per_side - 1)) && (z == 0 || z == (no_of_robots_per_side - 1))) {
		//					cos_theta = std::cos(glm::radians(45.f));
		//					//std::cout << "1\n";
		//				} else if (z == 0 || z == (no_of_robots_per_side - 1)) {
		//					cos_theta = glm::dot(glm::vec3(0.f, 0.f, 1.f), glm::normalize(vector_from_mid_point));
		//				} else if (x == 0 || x == (no_of_robots_per_side - 1)) {
		//					cos_theta = glm::dot(glm::vec3(1.f, 0.f, 0.f), glm::normalize(vector_from_mid_point));
		//				}

		//				float push_back_length = square_length / std::abs(cos_theta);
		//				//std::cout  << "\n" << x << ", " << z << " : " 
		//				//	<< std::abs(cos_theta) << " " << length_from_mid_point << " " << push_back_length << std::endl;
		//				//SwarmUtils::print_vector("robot grid pos", robot_grid_position);
		//				//SwarmUtils::print_vector("real mid point", real_mid_point);
		//				//SwarmUtils::print_vector("vector from mid point", vector_from_mid_point);
		//				

		//				//push_back_length = 0.f;
		//				push_back_offset = push_back_length * (glm::normalize(vector_from_mid_point));
		//			}

		//			if (!occupancy_grid_->is_out_of_bounds(robot_grid_position)
		//				&& !occupancy_grid_->is_interior(robot_grid_position)) {
		//				auto robot_position = occupancy_grid_->map_to_position(robot_grid_position) + push_back_offset;
		//				robot_positions.push_back(robot_position);
		//			}
		//		}
		//	}
		//}

		//int z = no_of_robots_per_side;
		//int x = 0;

		//std::cout << "No of robots : " << no_of_robots_per_side << " " << robot_positions.size() << std::endl;
		no_of_robots_ = robot_positions.size();
		break;
	}
	case SQUARE_CLOSE_TO_EDGE: {
		//int no_of_robots_per_side = ((no_of_robots_ - 4)/ 4.f) + 2;
		//glm::ivec3 robots_mid_point((no_of_robots_per_side / 2) - 1, 0, (no_of_robots_per_side / 2) - 1);
		//glm::ivec3 mid_point((grid_resolution_per_side_ / 2) - 1, 0, (grid_resolution_per_side_ / 2) - 1);
		//glm::ivec3 offset = mid_point - robots_mid_point;

		int distance_from_edge = 5;
		
		for (int x = 0; x < grid_resolution_per_side_ - 1; ++x) {
			for (int z = 0; z < grid_resolution_per_side_ - 1; z++) {
				if (((x - distance_from_edge == 0) || (grid_resolution_per_side_ - 1 - x - distance_from_edge == 0)) && 
					((z - distance_from_edge >= 0) && (grid_resolution_per_side_ - 1 - z - distance_from_edge >= 0)) ||
					(((z - distance_from_edge == 0) || (grid_resolution_per_side_ - 1 - z - distance_from_edge == 0)) && 
					((x - distance_from_edge >= 0) && (grid_resolution_per_side_ - 1 - x - distance_from_edge >= 0)))) {

					glm::ivec3 robot_grid_position = glm::ivec3(x, 0, z); // +offset;
					if (!occupancy_grid_->is_out_of_bounds(robot_grid_position)
						&& !occupancy_grid_->is_interior(robot_grid_position)) {
						if (robot_positions.size() < no_of_robots_) {
							robot_positions.push_back(occupancy_grid_->map_to_position(robot_grid_position));
						}
						//SwarmUtils::print_vector("Grid Pos : ", robot_grid_position);
					}
				}
			}
		}

		//int z = no_of_robots_per_side;
		//int x = 0;

		//std::cout << "No of robots : " << robot_positions.size() << std::endl;
		no_of_robots_ = robot_positions.size();
		break;
	}
	case GRID: {
		int no_of_robots_per_side = std::sqrt(no_of_robots_);
		glm::ivec3 robots_mid_point((no_of_robots_per_side / 2) - 1, 0, (no_of_robots_per_side / 2) - 1);
		glm::ivec3 mid_point((grid_resolution_per_side_ / 2) - 1, 0, (grid_resolution_per_side_ / 2) - 1);
		glm::ivec3 offset = mid_point - robots_mid_point;
		
		for (int x = 0; x < no_of_robots_per_side; ++x) {
			for (int z = 0; z < no_of_robots_per_side; z++) {
				//if ((z == 0 || z == (no_of_robots_per_side - 1)) || (x == 0 || x == no_of_robots_per_side - 1)) {
					glm::ivec3 robot_grid_position = glm::ivec3(x, 0, z) + offset;
					if (!occupancy_grid_->is_out_of_bounds(robot_grid_position)
						&& !occupancy_grid_->is_interior(robot_grid_position)) {
						robot_positions.push_back(occupancy_grid_->map_to_position(robot_grid_position));
					}
				//}
			}
		}

		int z = no_of_robots_per_side;
		int x = 0;

		no_of_robots_ = robot_positions.size();
		//while (robot_positions.size() < no_of_robots_) {
		//	std::cout << "Unable to fill all positions. Robots left : " << no_of_robots_ - robot_positions.size() << std::endl;
		//	glm::ivec3 robot_grid_position = glm::ivec3(x++, 0, z) + offset;
		//	occupancy_grid_->map_to_position(robot_grid_position);
		//	if (!occupancy_grid_->is_out_of_bounds(robot_grid_position)
		//		&& !occupancy_grid_->is_interior(robot_grid_position)) {
		//		robot_positions.push_back(occupancy_grid_->map_to_position(robot_grid_position));
		//	}
		//}
		break;

	}
	case RANDOM: {
		std::random_device rd;
		std::mt19937 rng;
		rng.seed(rd());
		std::uniform_int_distribution<int> position_generator(0, grid_resolution_per_side_ - 1);
		int robot_count = 0;
		int iterations = 0;
		while ((no_of_robots_ != robot_count) && ((no_of_robots_ + 100) > iterations)) {
			glm::ivec3 robot_grid_position(position_generator(rng), 0, position_generator(rng));
			if (!occupancy_grid_->is_out_of_bounds(robot_grid_position)
				&& !occupancy_grid_->is_interior(robot_grid_position)) {
				robot_positions.push_back(occupancy_grid_->map_to_position(robot_grid_position));
				robot_count++;
			}
			iterations++;
		}
		break;
	}
	case CIRCLE: {
		float perimeter_length = 2 * grid_length_ * no_of_robots_;
		float radius = perimeter_length / (2 * PI);
		glm::vec3 real_mid_point(grid_resolution_per_side_ * grid_length_ / 2.f, 0, grid_resolution_per_side_ * grid_length_ / 2.f);
		glm::vec3 center = real_mid_point;

		for (int i = 0; i < no_of_robots_; ++i) {
			float theta = 2 * PI * i / no_of_robots_;
			float x = center.x + radius * glm::cos(theta);
			float z = center.z + radius * glm::sin(theta);
			auto robot_position = glm::vec3(x, 0.f, z);
			auto robot_grid_position = occupancy_grid_->map_to_grid(robot_position);
			if (!occupancy_grid_->is_out_of_bounds(robot_grid_position)
				&& !occupancy_grid_->is_interior(robot_grid_position)) {
				robot_positions.push_back(robot_position);
			}
		}
		no_of_robots_ = robot_positions.size();

	}

	default: break;

	}

	return robot_positions;
}

void SwarmViewer::create_robots() {
	m_shader.bind();



	std::vector<glm::vec3> robot_positions = create_starting_formation(static_cast<Formation>(formation_));
	assert(no_of_robots_ == robot_positions.size());

	RenderMesh robot_mesh;
	cv::Vec4f color(0.f, 1.f, 1.f, 1.f);
	create_robot_model(robot_mesh, color);
		
	for (int i = 0; i < no_of_robots_; ++i) {
		std::random_device rd;
		std::mt19937 rng;
		rng.seed(rd());
		//std::uniform_real_distribution<float> color_generator(0.f, 1.f);
		//cv::Vec4f color(color_generator(rng), color_generator(rng), color_generator(rng), 1.f);
		//RenderMesh robot_mesh;
		////cv::Vec4f color(0.f, 1.f, 1.f, 1.f);
		//create_robot_model(robot_mesh, color);

		robot_color_map_[i] = color;


		//Robot robot(uniform_locations_, i, occupancy_grid_, explore_constant_, separation_constant_, goto_work_constant_, separation_distance_, &m_shader);
		//robot.mesh_.push_back(robot_mesh[0]);
		//robots_.push_back(robot);

		//Robot* robot = new ExperimentalRobot(uniform_locations_, 
		//	i, occupancy_grid_, collision_grid_, explore_constant_, separation_constant_, alignment_constant_, cluster_constant_, perimeter_constant_,
		//	goto_work_constant_,
		//	explore_range_, separation_range_, alignment_range_, cluster_range_, perimeter_range_, obstacle_near_range_,
		//	obstacle_far_range_, sensor_range_, discovery_range_, neighborhood_count_,
		//	separation_distance_, robot_positions[i], &m_shader, render_, magic_k_, collide_with_robots_, square_radius_, bounce_function_power_, bounce_function_multiplier_);
		Robot* robot = new ExperimentalRobot(uniform_locations_, 
			i, occupancy_grid_, collision_grid_,  separation_constant_, alignment_constant_, cluster_constant_, explore_constant_,
			sensor_range_, discovery_range_, 
			separation_distance_, robot_positions[i], square_radius_, bounce_function_power_, bounce_function_multiplier_, false, render_, &m_shader);

		robot->mesh_.push_back(robot_mesh[0]);
		robots_.push_back(robot);
	}
}



void SwarmViewer::create_occupancy_grid(int grid_resolution_per_side, int grid_size) {
	makeCurrent();

	//prepareShaderProgram(":/FilteredStructLight/model_vert.glsl", ":/FilteredStructLight/model_frag.glsl", line_shader_);

	cv::Vec4f green(0.f, 1.f, 0.f, 1.f);

	float y = OCCUPANCY_GRID_HEIGHT;
	float space = grid_size;
	int grid_length_per_side = grid_resolution_per_side / 2;
	//float max_value = grid_length_per_side * grid_length_;
	float max_value = grid_resolution_per_side * grid_size;

	VertexBufferData bufferdata;

	auto& points = bufferdata.positions;
	//std::vector<cv::Vec3f> points;
	//std::vector<cv::Vec4f> colors;
	//std::vector<cv::Vec3f> normals;

	//for (int i = -1 * grid_length_per_side; i < (grid_length_per_side + 1); ++i) {
	for (int i = 0; i < (grid_resolution_per_side); ++i) {
		// x lines
		//cv::Vec3f start_x(i * space, y, -1 * max_value);
		//cv::Vec3f end_x(i * space, y, max_value);

		cv::Vec3f start_x(i * space, y, 0);
		cv::Vec3f end_x(i * space, y, max_value);

		// z lines
		//cv::Vec3f start_z(-1 * max_value,  y, i * space);
		//cv::Vec3f end_z(max_value, y, i * space);

		cv::Vec3f start_z(0,  y, i * space);
		cv::Vec3f end_z(max_value, y, i * space);


		// points
		points.push_back(start_x);
		points.push_back(end_x);
		points.push_back(start_z);
		points.push_back(end_z);

		// colors
		// normals

		for (auto k = 0u; k < 4; ++k) {
			bufferdata.colors.push_back(green);
			bufferdata.normals.push_back(cv::Vec3f(0.f, 1.f, 0.f));
			bufferdata.indices.push_back((points.size() - 4) + k);
		}

	}

	bufferdata.count.push_back(bufferdata.positions.size());
	bufferdata.base_index.push_back(0);
	bufferdata.offset.push_back(0);

	RenderEntity occupancy_grid(GL_LINES, &m_shader);
	//occupancy_grid.set_type(RenderEntity::Default);
	occupancy_grid.upload_data_to_gpu(bufferdata);

	RenderMesh mesh;
	mesh.push_back(occupancy_grid);

	VisObject* grid =  new VisObject(uniform_locations_);
	grid->mesh_ = mesh;

	reset_vis_objects_.push_back(grid);
}

void SwarmViewer::set_no_of_robots(int no_of_robots) {
	no_of_robots_ = no_of_robots;
}

void SwarmViewer::set_separation_distance(float distance) {
	separation_distance_ = distance;
}

void SwarmViewer::set_grid_resolution(int grid_resolution) {
	//grid_resolution_ = std::pow(std::pow(2, grid_resolution), 3);
	grid_resolution_ = grid_resolution;
}

void SwarmViewer::set_grid_length(int grid_length) {
	grid_length_ = grid_length;
}

void SwarmViewer::set_interior_scale(double scale) {
	interior_scale_ = scale;
}

void SwarmViewer::set_interior_offset(glm::vec3 offset) {
	interior_offset_ = offset;
}

void SwarmViewer::set_exploration_constant(double constant) {
	explore_constant_ = constant;
}

void SwarmViewer::set_separation_constant(double constant) {
	separation_constant_ = constant;
}

void SwarmViewer::set_alignment_constant(double constant) {
	alignment_constant_ = constant;
}

void SwarmViewer::set_cluster_constant(double constant) {
	cluster_constant_ = constant;
}

void SwarmViewer::set_perimeter_constant(double constant) {
	perimeter_constant_ = constant;
}

void SwarmViewer::set_goto_work_constant(double constant) {
	goto_work_constant_ = constant;
}

void SwarmViewer::set_show_interior(int show) {
	show_interior_ = show;
}

void SwarmViewer::set_model_rotation(double x_rotation, double y_rotation, double z_rotation) {
	glm::mat4 rotateX = glm::rotate(glm::mat4(1.f), glm::radians((float)x_rotation), glm::vec3(1.f, 0.f, 0.f));
	glm::mat4 rotateY = glm::rotate(rotateX, glm::radians((float)y_rotation), glm::vec3(0.f, 1.f, 0.f));
	glm::mat4 rotateZ = glm::rotate(rotateY,  glm::radians((float)z_rotation), glm::vec3(0.f, 0.f, 1.f));
	
	model_rotation_ = rotateZ;
}

void SwarmViewer::set_square_formation_radius(double radius) {
	square_radius_ = radius;
}

void SwarmViewer::set_bounce_function_power(double bounce_function_power) {
	bounce_function_power_ = bounce_function_power;
}

void SwarmViewer::set_bounce_function_multiplier(double bounce_function_multiplier) {
	bounce_function_multiplier_ = bounce_function_multiplier;
}
