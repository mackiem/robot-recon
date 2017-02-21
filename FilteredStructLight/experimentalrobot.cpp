#include "experimentalrobot.h"
#include <chrono>
#include "swarmutils.h"
#include <glm/detail/type_mat.hpp>
#include "astar.h"

#define PI 3.14159265359	


//ExperimentalRobot::ExperimentalRobot(UniformLocations& locations, unsigned id, SwarmOccupancyTree* octree,
//	SwarmCollisionTree* collision_tree, Swarm3DReconTree* recon_tree, double explore_constant, double separation_constant, double alignment_constant,
//	double cluster_constant, double perimeter_constant, double work_constant, Range explore_range,
//	Range separation_range, Range alignment_range, Range cluster_range, Range perimeter_range,
//	Range obstacle_avoidance_near_range, Range obstacle_avoidance_far_range, double sensor_range,
//	int discovery_range, int neighborhood_count, double separation_distance, glm::vec3 position,
//	QGLShaderProgram* shader, bool render, double magic_k, bool collide_with_robots, double square_radius, double bounce_function_power, double bounce_function_multiplier)
//
//	: Robot(locations,
//	id, octree, collision_tree, explore_constant, separation_constant,
//	alignment_constant, cluster_constant, perimeter_constant,
//	work_constant,
//	explore_range, separation_range, alignment_range, cluster_range, perimeter_range, obstacle_avoidance_near_range,
//	obstacle_avoidance_far_range, sensor_range, discovery_range, neighborhood_count,
//	separation_distance, position, shader, render, magic_k, collide_with_robots), square_radius_(square_radius), bounce_function_power_(bounce_function_power), 
//	bounce_function_multiplier_(bounce_function_multiplier), recon_tree_(recon_tree)
//{
//	random_direction_ = glm::vec3(1.f, 0.f, 0.f);
//	max_velocity_ = 4.f;
//	robot_radius_ = 11.85f;
//	previous_no_of_explored_cells_ = -1;
//	death_time_ = -1;
//	dead_ = false;
//}

int VisibilityQuadrant::INVISIBLE = -1;
int VisibilityQuadrant::VISIBLE = 1;

//char** ExperimentalRobot::quadrants_ = nullptr;

ExperimentalRobot::ExperimentalRobot(UniformLocations& locations, unsigned id, int no_of_robots, SwarmOccupancyTree* octree,
	SwarmCollisionTree* collision_tree, Swarm3DReconTree* recon_tree, int cluster_id, double separation_constant, double alignment_constant,
	double cluster_constant, double explore_constant, double sensor_range,
	int discovery_range, double separation_distance, glm::vec3 position,
	double square_radius, double bounce_function_power, double bounce_function_multiplier, int max_time,
	bool collide_with_robots, bool render, QGLShaderProgram* shader)

	: Robot(locations, id, octree, collision_tree,  separation_constant,
	alignment_constant, cluster_constant, explore_constant,sensor_range, discovery_range, separation_distance, position, render, shader),  
	square_radius_(square_radius), bounce_function_power_(bounce_function_power), 
	bounce_function_multiplier_(bounce_function_multiplier), recon_tree_(recon_tree), max_time_(max_time), 
	local_map_(mm::Quadtree<int>(occupancy_grid_->get_grid_width(), occupancy_grid_->get_grid_height(), occupancy_grid_->get_grid_square_length(), 0)),
	no_of_robots_(no_of_robots)
{
	
	measurement_time_step_ = 100;
	random_direction_ = glm::vec3(1.f, 0.f, 0.f);
	max_velocity_ = 4.f;
	robot_radius_ = 11.85f;
	previous_no_of_explored_cells_ = -1;
	death_time_ = -1;
	dead_ = false;
	dead_color_changed_ = false;
	cluster_id_ = cluster_id;
	populate_occlusion_map();
	populate_clustering_map();
	random_constant_ = 10.f;

	diagonal_grid_length_ = std::sqrt(std::pow(occupancy_grid_->get_grid_height(), 2.f) + std::pow(occupancy_grid_->get_grid_width(), 2.f));

	explore_range_ = Range(0, diagonal_grid_length_);

	//local_map_ = new unsigned char[occupancy_grid_->get_grid_resolution_per_side() * occupancy_grid_->get_grid_resolution_per_side()];
	//local_no_of_unexplored_cells_ = occupancy_grid_->get_grid_resolution_per_side() * occupancy_grid_->get_grid_resolution_per_side();
	//std::fill(local_map_, local_map_ + local_no_of_unexplored_cells_, 0);

	local_no_of_unexplored_cells_ = occupancy_grid_->get_grid_width() * occupancy_grid_->get_grid_height();

	previous_no_of_local_explored_cells_ = 0;

	figure_mode_ = false;

	sensor_width_ = sensor_range_ * 2 + 1;
	sensor_height_ = sensor_range_ * 2 + 1;
	//half_sensor_width_ = std::ceil(sensor_width_ / 2.f);
	//half_sensor_height_ = std::ceil(sensor_height_ / 2.f);;
	half_sensor_width_ = (sensor_width_ / 2.f);
	half_sensor_height_ = (sensor_height_ / 2.f);;

	local_explore_state_ = EXPLORE;

	if (render_) {
		max_render_cell_size_ = occupancy_grid_->get_grid_width() * occupancy_grid_->get_grid_height();
		explored_cell_count_ = 0;
		vis_explored_cells_.resize(max_render_cell_size_);
		interior_explored_cell_count_ = 0;
		vis_interior_explored_cells_.resize(max_render_cell_size_);
	}

	max_adjacent_cells_ = std::pow(sensor_range_ * 2 + 1, 2);
	adjacent_cells_.resize(max_adjacent_cells_);
	current_adjacent_cells_ = 0;
	interior_cells_.resize(max_adjacent_cells_);
	current_interior_cells_ = 0;

	adjacent_robots_.resize(no_of_robots);
	current_no_of_robots_ = 0;

	global_explore_ = true;
	global_explore_ = false;
}

void ExperimentalRobot::populate_occlusion_map() {
	int no_of_divisions = max_time_ / measurement_time_step_;

	for (int i = 0; i < no_of_divisions; ++i) {
		occlusions_per_timestamp_map_[i * measurement_time_step_] = 0.0;
	}
}

void ExperimentalRobot::populate_clustering_map() {
	//measurement_time_step_ = 100;
	int no_of_divisions = max_time_ / measurement_time_step_;

	for (int i = 0; i < no_of_divisions; ++i) {
		clustered_neighbors_per_timestamp_map_[i * measurement_time_step_] = 0;
	}
}



void ExperimentalRobot::init_sensor_range() {
	sensor_width_ = sensor_range_ * 2 + 1;
	sensor_height_ = sensor_range_ * 2 + 1;
	sensor_cells_ = new int[sensor_width_ * sensor_height_];

	half_sensor_width_ = std::ceil(sensor_width_ / 2.f);
	half_sensor_height_ = std::ceil(sensor_height_ / 2.f);;


	glm::ivec3 robot_sensor_position(half_sensor_width_, 0, half_sensor_height_);
	auto robot_grid_position = occupancy_grid_->map_to_grid(position_);

	// init it
	for (int y = 0; y < sensor_height_; ++y) {
		for (int x = 0; x < sensor_width_; ++x) {
			glm::ivec3 sensor_cell(x, 0, y);
			glm::ivec3 relative_sensor_cell = sensor_cell - robot_sensor_position;
			auto sensor_grid_position = robot_grid_position + relative_sensor_cell;
			//SENSOR_STATE state = NORMAL;
			//if (occupancy_grid_->is_out_of_bounds(sensor_grid_position)) {
			//	state = OUT_OF_BOUNDS;
			//} else if (occupancy_grid_->going_through_interior_test(robot_grid_position, sensor_grid_position)) {
			//	state = INVISIBLE;
			//}
			//set_sensor_value(x, y, state);
		}
	}

	for (int y = 0; y < sensor_height_; ++y) {
		y_sensor_index_.push_back(y);
	}

	for (int x = 0; x < sensor_width_; ++x) {
		x_sensor_index_.push_back(x);
	}
}

void ExperimentalRobot::set_sensor_value(int x, int y, int value) const {
	int y_memory_index = y % sensor_height_;
	int x_memory_index = x % sensor_width_;

	sensor_cells_[y_memory_index * sensor_width_ + x_memory_index] = value;
}

int ExperimentalRobot::get_sensor_value(int x, int y) const {
	int y_memory_index = y % sensor_height_;
	int x_memory_index = x % sensor_width_;

	return sensor_cells_[y_memory_index * sensor_width_ + x_memory_index];
}

bool VisibilityQuadrant::is_visible_to_robot(const glm::ivec3& robot_position, const glm::ivec3& interior_position,
	const glm::ivec3& point_to_test) const {

	float length = glm::length((glm::vec3(point_to_test - robot_position)));
	float division_factor = (1.f / 100.f);
	int no_of_segments = (length / division_factor) + 1;
#ifdef DEBUG
	//std::cout << "No of segments : " << no_of_segments << std::endl;
#endif

	glm::vec3 direction;
	if (length > 1e-3) {
		direction = glm::normalize(glm::vec3(point_to_test - robot_position));
	}

	bool interior_found = false;
	float interior_threshold = 0.7;
	for (int i = 0; i < no_of_segments; ++i) {
		glm::vec3 testing_grid_position = glm::vec3(robot_position) + direction * (division_factor)* static_cast<float>(i);
		float distance_away = glm::length(testing_grid_position - glm::vec3(interior_position));
		if (distance_away < interior_threshold) {
			interior_found = true;
			break;
		}
	}

	return !interior_found;
}

VisibilityQuadrant* VisibilityQuadrant::instance_ = nullptr;

void VisibilityQuadrant::cleanup_internal() {
	if (quadrants_) {
		int no_of_cells = sensor_width_ * sensor_height_;
		for (int i = 0; i < no_of_cells; ++i) {
			delete [] quadrants_[i];
		}
		delete[] quadrants_;
		quadrants_ = nullptr;
	}
}

void VisibilityQuadrant::cleanup() {
	if (instance_) {
		instance_->cleanup_internal();
	}
}

VisibilityQuadrant::VisibilityQuadrant(int sensor_range) : sensor_width_(0), sensor_height_(0), half_sensor_width_(0), half_sensor_height_(0), sensor_range_(-1), quadrants_(nullptr) {
	sensor_range_ = sensor_range;
	sensor_width_ = sensor_range_ * 2 + 1;
	sensor_height_ = sensor_range_ * 2 + 1;
	half_sensor_width_ = (sensor_width_ / 2.f);
	half_sensor_height_ = (sensor_height_ / 2.f);;
	create_visibility_quadrant();
}

VisibilityQuadrant* VisibilityQuadrant::visbility_quadrant(int sensor_range) {
	if (instance_) {
		if (instance_->sensor_range_ >= sensor_range) {
			return instance_;
		}
		instance_->cleanup_internal();
	}
	std::cout << "Creating sensor range visibility quadrant : " << sensor_range << "\n";
	instance_ = new VisibilityQuadrant(sensor_range);
	return instance_;
}

void VisibilityQuadrant::create_visibility_quadrant() {

#if defined(_DEBUG)
	return;
#endif

	int no_of_sensor_cells = sensor_width_ * sensor_height_;

	quadrants_ = new char*[no_of_sensor_cells];

	//no_of_edge_cells_ = half_sensor_width_ + half_sensor_height_ - 1;
	//no_of_edge_cell_chars_ = std::ceil(no_of_edge_cells_/static_cast<float>(no_of_bits_));

	for (int i = 0; i < no_of_sensor_cells; ++i) {
		quadrants_[i] = new char[no_of_sensor_cells];
		std::fill(quadrants_[i], quadrants_[i] + no_of_sensor_cells, static_cast<char>(VISIBLE));
	}

	glm::ivec3 robot_position(half_sensor_width_, 0, half_sensor_height_);

	for (int interior_y = 0; interior_y < sensor_height_; ++interior_y) {
		for (int interior_x = 0; interior_x < sensor_width_; ++interior_x) {
			glm::ivec3 interior_position(interior_x, 0, interior_y);

			for (int y = 0; y < sensor_height_; ++y) {
				for (int x = 0; x < sensor_width_; ++x) {
					glm::ivec3 sensor_cell_position(x, 0, y);
					if (x == interior_x && y == interior_y) {
						continue;
					}
					if (!is_visible_to_robot(robot_position, interior_position, sensor_cell_position)) {
						quadrants_[interior_y * sensor_width_ + interior_x][y * sensor_width_ + x] = static_cast<char>(INVISIBLE);
					}
				}
			}
		}
	}

}

VisibilityQuadrant::~VisibilityQuadrant() {
	//delete local_map_;
	cleanup_internal();
}

ExperimentalRobot::QUADRANT ExperimentalRobot::get_quadrant(const glm::ivec3& pt) const {
	//if (pt.x > half_sensor_width_ && pt.y > half_sensor_height_) {
	//	return NE;
	//}
	//if (pt.x > half_sensor_width_ && pt.y <= half_sensor_height_) {
	//	return SE;
	//}
	//if (pt.x <= half_sensor_width_ && pt.y > half_sensor_height_) {
	//	return NW;
	//}
	//if (pt.x <= half_sensor_width_ && pt.y <= half_sensor_height_) {
	//	return SW;
	//}
	return SW;
}

glm::ivec3 ExperimentalRobot::flip_to_SW(const glm::ivec3& pt, ExperimentalRobot::QUADRANT quadrant) const {
	glm::ivec3 flipped_pt = pt;
	switch (quadrant) {
	case NE : {
		flipped_pt.x = sensor_width_ - flipped_pt.x;
		flipped_pt.y = sensor_height_- flipped_pt.y;
		break;
	}
	case SW: {
		break;
	}
	case SE: {
		flipped_pt.x = sensor_width_ - flipped_pt.x;
		break;
	}
	case NW: {
		flipped_pt.y = sensor_height_- flipped_pt.y;
		break;
	}
	default: break;
	}
	return flipped_pt;
}


bool VisibilityQuadrant::is_sensor_cell_visible(const glm::ivec3& robot_position, const glm::ivec3& interior_position,
	const glm::ivec3& point_to_test) {

#if defined(_DEBUG)
	return true;
#endif

		glm::ivec3 relative_point_position = point_to_test - robot_position + glm::ivec3(half_sensor_width_, 0, half_sensor_height_);;
		glm::ivec3 relative_interior_position = interior_position - robot_position + glm::ivec3(half_sensor_width_, 0, half_sensor_height_);

		if (relative_point_position.x > (sensor_width_ - 1)
			|| relative_point_position.z > (sensor_height_- 1)
			|| relative_interior_position.x > (sensor_width_- 1)
			|| relative_interior_position.z > (sensor_height_- 1)

			|| relative_point_position.x < 0
			|| relative_point_position.z < 0
			|| relative_interior_position.x < 0
			|| relative_interior_position.z < 0

			) {

			return false;
		}

		char visible = quadrants_[relative_interior_position.z * sensor_width_ + relative_interior_position.x]
			[relative_point_position.z * sensor_width_ + relative_point_position.x];
		//char*& edge_bits = quadrants_[flipped_interior_pt.z * half_sensor_width_ + flipped_interior_pt.x];
		//int char_no = edge_cell_index / no_of_bits_;
		//int bit_no = edge_cell_index % no_of_bits_;
		//char visible = edge_bits[char_no] >> bit_no & 1;
		
		//if (visible == INVISIBLE) {
		//	SwarmUtils::print_vector("test pt", point_to_test);
		//	SwarmUtils::print_vector("interior", interior_position);
		//}

		return (visible == VISIBLE);
}



void ExperimentalRobot::increment_sensor_range(glm::ivec3 increment) {
	if (increment.x > 0) {
		while (increment.x-- != 0) {
			// pop west
			// increment east
			// store new sensored values
			int easttmost_index = x_sensor_index_.front();
			x_sensor_index_.pop_back();
			x_sensor_index_.push_front(++easttmost_index);

			for (int y = 0; y < sensor_height_; ++y) {
				// check out of bounds
				// check for interior
				// check for visibility
				int visibility  = 0;
				set_sensor_value(easttmost_index, y, visibility);
				int value = get_sensor_value(easttmost_index, y);
			}
		}
	} 
	
}



void ExperimentalRobot::update_visualization_structs() {
	// update visualization
	if (show_forces_) {
		//if (!figure_mode_) {
			update_force_visualization(0, 1000.f * explore_force_);
			update_force_visualization(1, 1000.f * separation_force_);
			update_force_visualization(3, perimeter_force_);
			update_force_visualization(4, 1000.f * cluster_force_);
			update_force_visualization(5, 1000.f* alignment_force_);
			update_force_visualization(2, 1000.f * resultant_force_);
		//}
	}
	// update rendered mesh
	for (auto& render_entity : mesh_) {
		glm::mat4 translate_model = glm::translate(glm::mat4(1.f), position_);
		render_entity.set_model(translate_model * render_entity.get_initial_model());
	}

	explored_mutex_.lock();
	//for (auto& adjacent_sensor_cell : vis_explored_cells_) {
	//	overlay_->update_grid_position(adjacent_sensor_cell, color_ /2.f);
	//	//overlay_->update_grid_position(adjacent_sensor_cell);
	//}
	//for (auto& adjacent_sensor_cell : interior_explored_cells_) {
	//	overlay_->update_grid_position(adjacent_sensor_cell, color_ );
	//}
	for (int i = 0; i < explored_cell_count_; ++i) {
		overlay_->update_grid_position(vis_explored_cells_[i], color_ /2.f);
	}
	for (int i = 0; i < interior_explored_cell_count_; ++i) {
		overlay_->update_grid_position(vis_interior_explored_cells_[i], color_);
	}

	// reset counters
	explored_cell_count_ = 0;
	interior_explored_cell_count_ = 0;

	for (auto& adjacent_sensor_cell : vis_astar_cells_) {
		overlay_->update_grid_position(adjacent_sensor_cell, search_color_ );
	}

	std::random_device rd;
	std::mt19937 eng(rd());
	std::uniform_real_distribution<float> dist(0.f, 1.f);
	for (auto& adjacent_sensor_cell : vis_goal_cells_) {
		//overlay_->update_grid_position(adjacent_sensor_cell, cv::Vec4f(1.f, 1.f, 1.f, 1.f));
		overlay_->update_grid_position(adjacent_sensor_cell, cv::Vec4f(dist(eng), dist(eng), dist(eng), 1.f));
	}

	//explored_cells_.clear();
	//interior_explored_cells_.clear();
	vis_astar_cells_.clear();
	vis_goal_cells_.clear();

	for (auto& visited_cells : vis_poo_cells_) {
		//overlay_->update_grid_position(adjacent_sensor_cell, color_ /2.f);
		overlay_->update_poo_position(visited_cells, color_ );
	}
	vis_poo_cells_.clear();
	explored_mutex_.unlock();
	recon_mutex_.lock();
	for (auto& reconstructed_position : reconstructed_positions_) {
		//recon_points_->update_3d_points(occupancy_grid_->map_to_grid(reconstructed_position));
		recon_points_->update_3d_points((reconstructed_position));
	}
	reconstructed_positions_.clear();
	if (!dead_color_changed_ && dead_) {
		cv::Vec4f black(0.f, 0.f, 0.f, 1.f);
		change_color(black);
		dead_color_changed_ = true;
	}
	recon_mutex_.unlock();
}

void ExperimentalRobot::change_color(cv::Vec4f& color) {
	color_ = color;
	search_color_ = color;
	search_color_[1] /= 2.f;
	search_color_[3] = 1.f;
	RenderEntity& entity = mesh_[mesh_.size() - 1];

	if (colors_.size() > 0) {
		std::vector<cv::Vec4f> fill_color(colors_.size());
		std::fill(fill_color.begin(), fill_color.end(), color);

		glBindVertexArray(entity.vao_);
		glBindBuffer(GL_ARRAY_BUFFER, entity.vbo_[RenderEntity::COLOR]);
		glBufferSubData(GL_ARRAY_BUFFER, 0, colors_.size() * sizeof(cv::Vec4f), &fill_color[0]);

		glBindVertexArray(0);
	}
}

void ExperimentalRobot::set_colors_buffer(std::vector<cv::Vec4f>& colors) {
	colors_ = colors;
}

void ExperimentalRobot::set_death_time(int death_time) {
	death_time_ = death_time;
}

void ExperimentalRobot::set_cluster_id(int cluster_id) {
	cluster_id_ = cluster_id;
}

void ExperimentalRobot::set_figure_mode(bool figure_mode) {
	figure_mode_ = figure_mode;
	measurement_time_step_ = 5;
}

ExperimentalRobot::~ExperimentalRobot()
{
}

glm::vec3 ExperimentalRobot::calculate_separation_velocity() {
	const float max_distance = separation_distance_;
	glm::vec3 c;

	for (int i = 0; i < current_no_of_robots_; ++i) {
		auto& robot_id = adjacent_robots_[i];
	//for (auto& robot_id : robot_ids_) {
		if (robot_id != id_) {
			float distance_apart = glm::length(robots_[robot_id]->get_position() - position_);
			if (distance_apart < max_distance) {
				auto move_away_vector = (robots_[robot_id]->get_position() - position_);
				//c -= glm::vec3(move_away_vector.x * move_away_vector.x, move_away_vector.y * move_away_vector.y,
				//	move_away_vector.z * move_away_vector.z);
				auto inverse_normalized_dist = (max_distance - distance_apart) / max_distance;
				float normalize_constant = std::pow(inverse_normalized_dist, 2);
				c -= normalize_constant * move_away_vector * separation_constant_;
			}
		}
	}
	return c;
}

glm::vec3 ExperimentalRobot::calculate_alignment_velocity() {
	glm::vec3 v;
	int count = 0;

	for (int i = 0; i < current_no_of_robots_; ++i) {
		auto& robot_id = adjacent_robots_[i];
	//for (auto& robot_id : robot_ids_) {
		if (robots_[robot_id]->get_cluster_id() == cluster_id_
			&& !robots_[robot_id]->is_dead()) {
			if (robot_id != id_) {
				v += robots_[robot_id]->get_velocity();
				count++;
			}
		}
	}
	if (count > 0) {
		v /= static_cast<float>(count);
	} else {
		v = velocity_;
	}
	//return (v - velocity_) / 8.f;
	glm::vec3 alignment_velocity = (v - velocity_) * alignment_constant_;
	alignment_force_ = alignment_velocity;
	return alignment_velocity;
}

//glm::vec3 ExperimentalRobot::bounce_off_corners_velocity() {
//	float x_max, y_max, z_max;
//	float x_min, y_min, z_min;
//	float outer_boundary = 3.5f;
//	x_min = y_min = z_min = occupancy_grid_->get_grid_square_length() * outer_boundary;
//	x_max = y_max = z_max = occupancy_grid_->get_grid_square_length() * (occupancy_grid_->get_grid_resolution_per_side() - outer_boundary);
//
//	float x_wall_max, y_wall_max, z_wall_max;
//	float x_wall_min, y_wall_min, z_wall_min;
//
//	float inner_boundary = 1.5f;
//	x_wall_min = y_wall_min = z_wall_min = occupancy_grid_->get_grid_square_length() * inner_boundary;
//	x_wall_max = y_wall_max = z_wall_max = occupancy_grid_->get_grid_square_length() * (occupancy_grid_->get_grid_resolution_per_side() - inner_boundary);
//	
//	const float opposite_vel = max_velocity_;
//	const float power = 2.f;
//	glm::vec3 v;
//
//	float bounce_off_normalizing_constant = 0.001f;
//
//	if (position_.x > x_max
//		&& position_.z > z_max) {
//			float normalizing_constant = 1.f / std::pow(position_.x - x_wall_max, power);
//			v.x = bounce_off_normalizing_constant * -opposite_vel;
//			v.z = normalizing_constant * -opposite_vel;
//
//	} else if (position_.x < x_min
//		&& position_.z < z_min) {
//			float normalizing_constant = 1.f / std::pow(position_.z - z_wall_min, power);
//			v.z = bounce_off_normalizing_constant * opposite_vel;
//			v.x = normalizing_constant * opposite_vel;
//		
//	}
//	else {
//
//
//		if (position_.x < x_min) {
//			float normalizing_constant = 1.f / std::pow(position_.x - x_wall_min, power);
//			v.x = bounce_off_normalizing_constant * opposite_vel;
//			// send in the normal direction
//			v.z = normalizing_constant * opposite_vel;
//		}
//		else if (position_.x > x_max) {
//			float normalizing_constant = 1.f / std::pow(position_.x - x_wall_max, power);
//			// send in the normal direction
//			v.x = bounce_off_normalizing_constant * -opposite_vel;
//			v.z = normalizing_constant * -opposite_vel;
//		}
//
//		//if (position_.y < y_min) {
//		//	v.y = opposite_vel;
//		//} else if (position_.y > y_may) {
//		//	v.y = -opposite_vel;
//		//}
//
//		if (position_.z < z_min) {
//			float normalizing_constant = 1.f / std::pow(position_.z - z_wall_min, power);
//			v.z = bounce_off_normalizing_constant * opposite_vel;
//			v.x = normalizing_constant * -opposite_vel;
//		}
//		else if (position_.z > z_max) {
//			float normalizing_constant = 1.f / std::pow(position_.z - z_wall_max, power);
//			v.z = bounce_off_normalizing_constant * -opposite_vel;
//			v.x = normalizing_constant * opposite_vel;
//		}
//	}
//	return v;
//}

glm::vec3 ExperimentalRobot::calculate_clustering_velocity() {
	glm::vec3 pc;
	int count = 0;

	for (int i = 0; i < current_no_of_robots_; ++i) {
		auto& robot_id = adjacent_robots_[i];
	//for (auto& robot_id : robot_ids_) {
		if (robots_[robot_id]->get_cluster_id() == cluster_id_ 
			&& !robots_[robot_id]->is_dead()) {
			if (robot_id != id_) {
				pc += robots_[robot_id]->get_position();
				count++;
			}
		}
	}

	if (count > 0) {
		pc /= static_cast<float>(count);
	} else {
		pc = position_;
	}

	if (current_timestamp_ > 0 && ((current_timestamp_ % measurement_time_step_) == 0)) {
		clustered_neighbors_per_timestamp_map_[current_timestamp_] = count ;
	}


	float normalizing_constant = std::pow(glm::length(pc - position_) / (sensor_range_ * 1.414 * occupancy_grid_->get_grid_square_length()), 2);
	//glm::vec3 cluster_velocity = normalizing_constant * (pc - position_) / 10.f;
	glm::vec3 cluster_velocity = normalizing_constant * (pc - position_) * cluster_constant_;
	cluster_force_ = cluster_velocity;
	return cluster_velocity;
}

glm::vec3 ExperimentalRobot::calculate_random_direction() {
	glm::vec3 v;
	
	return v;
}

glm::vec3 ExperimentalRobot::calculate_bounce_explore_velocity(const glm::vec3& interior_cell) const {
	glm::vec3 bounce_velocity;
	//
	//auto corners = get_corners(interior_cell);

	//std::vector<glm::vec3> normals(4);
	//for (int i = 1; i < corners.size(); ++i) {
	//	auto mid_point = (corners[i % 4] + corners[i - 1]) / 2.f;
	//	normals[i - 1]
	//}
	return bounce_velocity;
}


glm::vec3 ExperimentalRobot::calculate_bounce_explore_velocity(const std::vector<glm::vec3>& interior_cells) const {
	glm::vec3 bounce_velocity;
	for (auto& interior_cell : interior_cells) {
		bounce_velocity += calculate_bounce_explore_velocity(interior_cell);
	}
	return bounce_velocity;
}



bool ExperimentalRobot::local_explore_search(glm::ivec3& explore_cell_position) {

	// if there is no interior - means in a nxn sensored area, there is no interior
	// if there is interior - go to interior, but there is no global interior map, so a local map for everyone?
	// how to coordinate among the cluster - leader election (too much work?) or
	// maybe just try local picking of a location and hope everyone agrees through cluster and alignment?
	//make the  local picking pick the same direction

	// sensor grid size will be	(sensor_range_ * 2 + 1)^2;

	// do it in a bfs way

	glm::ivec3 grid_position = occupancy_grid_->map_to_grid(position_);
	bool cell_found = false;

	//for (auto& interior : interior_cells_) {
	for (int k = 0; k < current_interior_cells_; ++k) {
		auto& interior = interior_cells_[k];
		for (int i = 0; i < current_adjacent_cells_; ++i) {
			auto& visibility_aware_adjacent_cell = adjacent_cells_[i];
			if (!visibility_aware_adjacent_cell.is_visible()) {
				continue;
			}
			auto& adjacent_cell = visibility_aware_adjacent_cell.cell;
		//for (auto& adjacent_cell : adjacent_cells_) {
			float distance = glm::length(glm::vec3(occupancy_grid_->map_to_grid(interior) - adjacent_cell));
			if (!occupancy_grid_->is_interior(adjacent_cell) 
				&& not_locally_visited(adjacent_cell)
				&& distance < 1.414) {
				cell_found = true;
				explore_cell_position = adjacent_cell;
				return true;
			}
		}
	}

	//for (int sensor_level = 1; sensor_level <= sensor_range_; ++sensor_level) {
	//	for (int z = -sensor_level; z <= sensor_level; ++z) {
	//		for (int x = -sensor_level; x <= sensor_level; ++x) {
	//			if (std::abs(x) == sensor_level
	//				|| std::abs(z) == sensor_level) {
	//				glm::ivec3 cell_position = grid_position + glm::ivec3(x, 0, z);
	//				if (!occupancy_grid_->is_out_of_bounds(cell_position)
	//					&& occupancy_grid_->is_interior(cell_position)) {
	//					// get adjacent cells
	//					int free_adjacent_cells = 0;
	//					int sensor_level_interior = 1;
	//					for (int z_interior = -sensor_level_interior; z_interior <= sensor_level_interior; ++z_interior) {
	//						for (int x_interior = -sensor_level_interior; x_interior <= sensor_level_interior; ++x_interior) {
	//							if (!(x_interior == 0 && z_interior == 0)) {
	//								glm::ivec3 adjacent_cell = cell_position + glm::ivec3(x_interior, 0.f, z_interior);
	//								if (!occupancy_grid_->is_out_of_bounds(adjacent_cell)
	//									&& !occupancy_grid_->is_interior(adjacent_cell)
	//									&& not_locally_visited(adjacent_cell)
	//									//&& VisibilityQuadrant::visbility_quadrant(sensor_range_)->is_sensor_cell_visible(current_position, occupancy_grid_->map_to_grid(interior_cell), *itr)
	//									//&& !occupancy_grid_->going_through_interior_test(grid_position, adjacent_cell)
	//										)
	//								{
	//									free_adjacent_cells++;
	//									cell_found = true;
	//									explore_cell_position = adjacent_cell;
	//									return true;
	//								}
	//							}
	//						}
	//					}

	//					//if (free_adjacent_cells > 1) {
	//					//	cell_found = true;
	//					//	explore_cell_position  = cell_
	//					//	
	//					//}

	//				}
	//			}
	//		}
	//	}
	//}
	return false;
}

bool  ExperimentalRobot::local_perimeter_search(glm::ivec3& explore_cell_position) {

	glm::ivec3 current_robot_grid_position = occupancy_grid_->map_to_grid(position_);
	bool cell_found = false;

	for (int sensor_level = sensor_range_ + 1; sensor_level <= diagonal_grid_length_; ++sensor_level) {
		//int no_of_iter = 0;
		//int out_of_bounds = 0;
		for (int z = -sensor_level; z <= sensor_level; ++z) {
			for (int x = -sensor_level; x <= sensor_level; ++x) {
				if (std::abs(x) == sensor_level
					|| std::abs(z) == sensor_level) {

					glm::ivec3 cell_position = current_robot_grid_position + glm::ivec3(x, 0, z);
					//no_of_iter++;
					//if (occupancy_grid_->is_out_of_bounds(cell_position)) {
					//	out_of_bounds++;
					//}

					if (!occupancy_grid_->is_out_of_bounds(cell_position)
						//&& !occupancy_grid_->going_through_interior_test(grid_position, cell_position)
						&& !occupancy_grid_->is_interior(cell_position)
						&& not_locally_visited(cell_position)) {

							bool visible = true;
							if (sensor_range_ * 2 >= sensor_level) {
								for (int k = 0; k < current_interior_cells_; ++k) {
									auto& interior_cell = interior_cells_[k];
								//for (auto& interior_cell : interior_cells_) {
									auto interior_grid_pos = occupancy_grid_->map_to_grid(interior_cell);
									if (!VisibilityQuadrant::visbility_quadrant(sensor_level)->is_sensor_cell_visible(current_robot_grid_position, interior_grid_pos, cell_position)) {
										visible = false;
										break;
									}
								}
							} else {
								visible = !occupancy_grid_->going_through_interior_test(current_robot_grid_position, cell_position);
							}

							if (visible) {
								cell_found = true;
								explore_cell_position = cell_position;
								return true;
							}



						//// get adjacent cells
						//int free_adjacent_cells = 0;
						//for (int z_interior = -sensor_level; z_interior <= sensor_level; ++z_interior) {
						//	for (int x_interior = -sensor_level; x_interior <= sensor_level; ++x_interior) {
						//		if (!(x_interior == 0 && z_interior == 0)) {
						//			glm::ivec3 adjacent_cell = cell_position + glm::ivec3(x_interior, 0.f, z_interior);

						//			if (!occupancy_grid_->is_interior(adjacent_cell)
						//				&& !occupancy_grid_->is_out_of_bounds(adjacent_cell)
						//				&& !occupancy_grid_->going_through_interior_test(grid_position, adjacent_cell))
						//			{
						//				free_adjacent_cells++;
						//				cell_found = true;
						//				explore_cell_position = adjacent_cell;
						//				return true;
						//			}
						//		}
						//	}
						//}
					}
				}
			}
		}
		//if (no_of_iter > 0 && no_of_iter == out_of_bounds) {
		//	// we are outside the defined world, time to quit
		//	break;
		//}
	}
	return false;
}

bool ExperimentalRobot::local_perimeter_search_for_astar(glm::ivec3& explore_cell_position) {

	glm::ivec3 current_robot_grid_position = occupancy_grid_->map_to_grid(position_);
	bool cell_found = false;

	for (int sensor_level = sensor_range_ + 1; sensor_level <= diagonal_grid_length_; ++sensor_level) {
		//int no_of_iter = 0;
		//int out_of_bounds = 0;
		for (int z = -sensor_level; z <= sensor_level; ++z) {
			for (int x = -sensor_level; x <= sensor_level; ++x) {
				if (std::abs(x) == sensor_level
					|| std::abs(z) == sensor_level) {

					glm::ivec3 cell_position = current_robot_grid_position + glm::ivec3(x, 0, z);

					if (!occupancy_grid_->is_out_of_bounds(cell_position)
						//&& !occupancy_grid_->going_through_interior_test(grid_position, cell_position)
						&& !occupancy_grid_->is_interior(cell_position)
						&& not_locally_visited(cell_position)) {

						cell_found = true;
						explore_cell_position = cell_position;
						return true;
					}
				}
			}
		}
		//if (no_of_iter > 0 && no_of_iter == out_of_bounds) {
		//	// we are outside the defined world, time to quit
		//	break;
		//}
	}
	return false;
}

glm::vec3 ExperimentalRobot::calculate_local_explore_velocity() {

	glm::vec3 explore_velocity;
	auto explored_cells = local_no_of_unexplored_cells_;

	auto current_cell = occupancy_grid_->map_to_grid(previous_position_);
	auto previous_cell = occupancy_grid_->map_to_grid(previous_nminus2_position_);

	bool something_to_explore = false;
	glm::ivec3 explore_cell;
		
	if ((previous_no_of_local_explored_cells_ == explored_cells)) {
		same_cell_count_++;
		something_to_explore = true;
		explore_cell = previous_local_explore_cell;
		if (same_cell_count_ > 300) {
			something_to_explore = false;
		}
	}

	if (!something_to_explore) {
		same_cell_count_ = 0;

		something_to_explore = local_explore_search(explore_cell);
		local_explore_state_ = EXPLORE;

		if (!something_to_explore) {
			// go left
			something_to_explore = local_perimeter_search(explore_cell);
			local_explore_state_ = PERIMETER;


			if (!something_to_explore) {
				explore_cell = occupancy_grid_->map_to_grid(position_) + glm::ivec3(0.f, 0.f, 1.f);
				something_to_explore = true;
			}
		} 
		//std::string str = std::to_string(id_) + " " + std::to_string(local_no_of_unexplored_cells_) + " ";
		//SwarmUtils::print_vector(str, explore_cell);
	}


	if (something_to_explore) {
		previous_local_explore_cell = explore_cell;
		//explored_mutex_.lock();
		//search_output_cells_.push_back(explore_cell);
		//explored_mutex_.unlock();
		auto move_to_position = occupancy_grid_->map_to_position(explore_cell);
		float max_distance = diagonal_grid_length_ * occupancy_grid_->get_grid_square_length();
		float normalizing_constant = std::pow(glm::length(move_to_position - position_) / max_distance, 2);
		explore_velocity = normalizing_constant * (move_to_position - position_) * explore_constant_;
	}
	explore_force_ = explore_velocity;
	previous_no_of_local_explored_cells_ = explored_cells;
	return explore_velocity;
}

bool ExperimentalRobot::get_next_goal(glm::ivec3& position) {

	if (path_.size() > 0) {
		auto next_goal_grid_position = path_.front();
		glm::ivec3 current_grid_pos = 
			occupancy_grid_->map_to_grid(position_);

		if (current_grid_pos == next_goal_grid_position) {
			// we are there 
			path_.pop_front();
			if (path_.size() == 0) {
				return false;
			}

			position = path_.front();

		}
		else {
			position = next_goal_grid_position;
		}

		return true;
	}
	return false;
}

void ExperimentalRobot::calculate_astar_path(Grid* grid, const glm::ivec3& current_cell, const glm::ivec3& goal_cell, glm::ivec3& explore_cell) {

	AStar astar(grid);
	auto current_pos = occupancy_grid_->map_to_position(current_cell);
	auto goal_pos = occupancy_grid_->map_to_position(goal_cell);
	path_ = astar.search(current_pos, goal_pos);

	if (render_) {
		for (auto& path_cell : path_) {
			explored_mutex_.lock();
			vis_astar_cells_.push_back(path_cell);
			explored_mutex_.unlock();
		}
	}

	if (path_.size() > 0) {
		explore_cell = path_.front();
	}
	
}


glm::vec3 ExperimentalRobot::calculate_astar_explore_velocity() {

	glm::vec3 explore_velocity;
	auto current_cell = occupancy_grid_->map_to_grid(previous_position_);
	auto previous_cell = occupancy_grid_->map_to_grid(previous_nminus2_position_);

	glm::ivec3 explore_cell;
	bool something_to_explore = false;

	int explored_cells = 0;
	glm::ivec3 next_cell;

	if (global_explore_) {
		explored_cells = occupancy_grid_->no_of_unexplored_cells();
		something_to_explore = get_next_goal(explore_cell);
		
		if (!something_to_explore) {
			something_to_explore = occupancy_grid_->next_cell_to_explore_visibility_non_aware(current_cell, next_cell, 
				//explore_range_.min_, explore_range_.max_);
				0, max_render_cell_size_);
			calculate_astar_path(occupancy_grid_, current_cell, next_cell, explore_cell);
		}

	} else {
		explored_cells = local_no_of_unexplored_cells_;


		//if ((previous_no_of_local_explored_cells_ == explored_cells)) {
		//	same_cell_count_++;
		//	something_to_explore = true;
		//	explore_cell = previous_local_explore_cell;
		//	if (same_cell_count_ > 300) {
		//		something_to_explore = false;
		//	}
		//}
		if (not_locally_visited(previous_local_explore_cell) && path_.size() == 0) {
			next_cell = previous_local_explore_cell;

			calculate_astar_path(&local_map_, current_cell, next_cell, explore_cell);
			
			something_to_explore = true;
		}

		if (!something_to_explore) {

			something_to_explore = get_next_goal(explore_cell);

			if (!something_to_explore) {
				same_cell_count_ = 0;

				glm::ivec3 next_cell;
				something_to_explore = local_explore_search(next_cell);
				local_explore_state_ = EXPLORE;

				if (!something_to_explore) {
					// go left
					something_to_explore = local_perimeter_search_for_astar(next_cell);
					local_explore_state_ = PERIMETER;
				}

				if (something_to_explore) {
					// find apath store
					calculate_astar_path(&local_map_, current_cell, next_cell, explore_cell);
				}

				if (!something_to_explore) {
					explore_cell = occupancy_grid_->map_to_grid(position_) + glm::ivec3(0.f, 0.f, 1.f);
					something_to_explore = true;
				}
			}
		}
	}

	if (something_to_explore) {
		//SwarmUtils::print_vector("explore", explore_cell);
		previous_local_explore_cell = explore_cell;

		auto move_to_position = occupancy_grid_->map_to_position(explore_cell);
		float max_distance = diagonal_grid_length_ * occupancy_grid_->get_grid_square_length();
		float normalizing_constant = std::pow(glm::length(move_to_position - position_) / max_distance, 2);
		explore_velocity = normalizing_constant * (move_to_position - position_) * explore_constant_;

		explored_mutex_.lock();
		vis_goal_cells_.push_back(next_cell);
		explored_mutex_.unlock();
	}

	explore_force_ = explore_velocity;
	previous_no_of_local_explored_cells_ = explored_cells;
	return explore_velocity;
}

glm::vec3 ExperimentalRobot::calculate_explore_velocity() {
	glm::vec3 explore_velocity;

	auto explored_cells = occupancy_grid_->no_of_unexplored_cells();

	auto current_cell = occupancy_grid_->map_to_grid(previous_position_);
	auto previous_cell = occupancy_grid_->map_to_grid(previous_nminus2_position_);

	bool something_to_explore = false;
	glm::ivec3 explore_cell;
	if ((previous_no_of_explored_cells_ == explored_cells) && (previous_cell == current_cell)) {
		something_to_explore = true;
		explore_cell = previous_explore_cell;
	}
	else {
		something_to_explore = occupancy_grid_->next_cell_to_explore(current_cell, explore_cell, explore_range_.min_, explore_range_.max_);
	}
	//something_to_explore = occupancy_grid_->next_cell_to_explore(current_cell, explore_cell, explore_range_.min_, explore_range_.max_);

	if (something_to_explore) {
		previous_explore_cell = explore_cell;
		auto move_to_position = occupancy_grid_->map_to_position(explore_cell);
		float max_distance = diagonal_grid_length_ * occupancy_grid_->get_grid_square_length();
		float normalizing_constant = std::pow(glm::length(move_to_position - position_) / max_distance, 2);
		//explore_velocity = normalizing_constant * (move_to_position - position_) / 2.5f;
		explore_velocity = normalizing_constant * (move_to_position - position_) * explore_constant_;

		//if (id_ == 0) {
		//	SwarmUtils::print_vector("explore", explore_cell);
		//}
	}

	explore_force_ = explore_velocity;

	previous_no_of_explored_cells_ = explored_cells;


	return explore_velocity;

	//float x_max, y_max, z_max;
	//float x_min, y_min, z_min;
	//float outer_boundary = 2.5f;
	//x_min = y_min = z_min = occupancy_grid_->get_grid_square_length() * outer_boundary;
	//x_max = y_max = z_max = occupancy_grid_->get_grid_square_length() * (occupancy_grid_->get_grid_resolution_per_side() - outer_boundary);

	//float x_wall_max, y_wall_max, z_wall_max;
	//float x_wall_min, y_wall_min, z_wall_min;

	//float inner_boundary = 1.5f;
	//x_wall_min = y_wall_min = z_wall_min = occupancy_grid_->get_grid_square_length() * inner_boundary;
	//x_wall_max = y_wall_max = z_wall_max = occupancy_grid_->get_grid_square_length() * (occupancy_grid_->get_grid_resolution_per_side() - inner_boundary);
	//

	//glm::vec3 v;
	////float power = 3.0f;

	//float normalizingConstant = (outer_boundary - inner_boundary) * occupancy_grid_->get_grid_square_length();
	//if (position_.x <= x_min) {
	//	float dist = 1.0 - std::abs(position_.x - x_wall_min) / normalizingConstant;  // ranges 0 to 1
	//	dist = pow(dist, bounce_function_power_);
	//	float bounce_velocity = max_velocity_ * dist;

	//	v.x = bounce_velocity;
	//}
	//else if (position_.x >= x_max) {
	//	float dist = 1.0 - std::abs(position_.x - x_wall_max) / normalizingConstant;  // ranges 0 to 1
	//	dist = pow(dist, bounce_function_power_);
	//	float bounce_velocity = -max_velocity_ * dist;
	//	v.x = bounce_velocity;
	//}

	//if (position_.z <= z_min) {
	//	float dist = 1.0 - std::abs(position_.z - z_wall_min) / normalizingConstant;  // ranges 0 to 1
	//	dist = pow(dist, bounce_function_power_);
	//	float bounce_velocity = max_velocity_ * dist;
	//	v.z = bounce_velocity;
	//}
	//else if (position_.z >= z_max) {
	//	float dist = 1.0 - std::abs(position_.z - z_wall_max) / normalizingConstant;  // ranges 0 to 1
	//	dist = pow(dist, bounce_function_power_);
	//	float bounce_velocity = -max_velocity_ * dist;
	//	v.z = bounce_velocity;
	//}

	//explore_velocity += v;

	//return explore_velocity;
	
}


void ExperimentalRobot::reconstruct_points() {
	// make a reconstruction grid
	//for (auto& adjacent : adjacent_cells_) {
	//	// this may slow things down, look into doing it when grid cell changes
	//	//auto points_3d = recon_tree_->get_3d_points(interior);
	//	//update_some_data_struct_to_render_points(points_3d);
	//	if (render_) {
	//		if (past_reconstructed_positions_.find(adjacent) == past_reconstructed_positions_.end()) {
	//			recon_mutex_.lock();
	//			reconstructed_positions_.push_back(adjacent);
	//			recon_mutex_.unlock();
	//			past_reconstructed_positions_.insert(adjacent);
	//		}
	//	}
	//}
	//for (auto& interior : interior_cells_) {
	//	recon_tree_->update_multi_sampling_map(occupancy_grid_->map_to_grid(interior));
	//}
}

std::vector<glm::vec3> ExperimentalRobot::get_corners(const glm::vec3& interior_cell) const {
	// 4 corners
	std::vector<glm::vec3> corners(4);
	float half_distance = occupancy_grid_->get_grid_square_length() / 2.f;

	//int iter = 0;
	//for (int x = -1; x < 1; ++x) {
	//	if (x != 0) {
	//		for (int z = -1; z < 1; ++z) {
	//			if (z != 0) {
	//				corners[iter] = interior_cell + glm::vec3(x * half_distance, 0, z * half_distance);
	//				iter++;
	//			}
	//		}
	//	}
	//}

	int x, z;

	x = z = 1;
	corners[0] = interior_cell + glm::vec3(x * half_distance, 0, z * half_distance);

	x = 1; z = -1;
	corners[1] = interior_cell + glm::vec3(x * half_distance, 0, z * half_distance);

	x = -1; z = -1;
	corners[2] = interior_cell + glm::vec3(x * half_distance, 0, z * half_distance);

	x = -1; z = 1;
	corners[3] = interior_cell + glm::vec3(x * half_distance, 0, z * half_distance);
	
	return corners;
}

double ExperimentalRobot::calculate_occulsion() {
	double occlusion = 0.0;
	double entry_count = 0;

	for (auto& occlusion_per_timestamp: occlusions_per_timestamp_map_) {
		if (occlusion_per_timestamp.first <= current_timestamp_) {
			occlusion += occlusion_per_timestamp.second;
			entry_count++;
		}
	}
	if (entry_count > 0) {
		occlusion /= entry_count;
	}
	return occlusion;
	
}

double ExperimentalRobot::calculate_clustering() {
	double clustering = 0.0;
	double entry_count = 0;

	for (auto& clustering_per_timestamp: clustered_neighbors_per_timestamp_map_) {
		if (clustering_per_timestamp.first <= current_timestamp_) {
			clustering += clustering_per_timestamp.second;
			entry_count++;
		}
	}
	if (entry_count > 0) {
		clustering /= entry_count;
	}
	return clustering;
	
}

bool ExperimentalRobot::is_colliding_precisely(const glm::vec3& interior_cell) {

	auto corners = get_corners(interior_cell);

	auto grid_cell = occupancy_grid_->map_to_grid(interior_cell);
	// intersection test with lines
	for (int i = 1; i <= 4; ++i) {
		glm::vec3 corner_1 = corners[i - 1];
		glm::vec3 corner_0 = corners[i % 4];

		glm::vec3 line =  corner_1 - corner_0;
		glm::vec3 robot_to_corner =  position_ - corner_0;
		line.y = robot_to_corner.y = 0.f;

		// projection
		glm::vec3 proj_vector = line * glm::dot(line, robot_to_corner) / glm::dot(line, line);

		glm::vec3 projected_point = corner_0 + proj_vector;

		// check if projected point in between corners
		if (!(
			(((corner_0.x <= projected_point.x) && (projected_point.x <= corner_1.x))
			|| ((corner_1.x <= projected_point.x) && (projected_point.x <= corner_0.x)))
			&&
			(((corner_0.z <= projected_point.z) && (projected_point.z <= corner_1.z))
			|| ((corner_1.z <= projected_point.z) && (projected_point.z <= corner_0.z)))
			)) 
		{
			continue;
		}

		projected_point.y = 10.f;

		//std::cout << "intersection length : " << glm::length(projected_point - position_) << "\n";
		if (glm::length(projected_point - position_) <= robot_radius_) {
			return true;
		}
	}

	return false;

}

bool ExperimentalRobot::is_colliding_precisely(const std::vector<glm::vec3>& interior_cells) {
	for (auto& interior_cell : interior_cells) {
		bool is_colliding = is_colliding_precisely(interior_cell);
		if (is_colliding) {
			return true;
		}
	}
	return false;
	
}

glm::vec3 ExperimentalRobot::calculate_obstacle_avoidance_velocity() {
	glm::vec3 bounce_force;

	//for (auto& interior_center : interior_cells_) {
	for (int k = 0; k < current_interior_cells_; ++k) {
		auto& interior_center = interior_cells_[k];

		float x_max, y_max, z_max;
		float x_min, y_min, z_min;

		//float outer_boundary = sensor_range_;
		float outer_boundary = 2.5f;

		x_min = interior_center.x - (occupancy_grid_->get_grid_square_length() * outer_boundary);
		z_min = interior_center.z - (occupancy_grid_->get_grid_square_length() * outer_boundary);

		x_max = interior_center.x + (occupancy_grid_->get_grid_square_length() * outer_boundary);
		z_max = interior_center.z + (occupancy_grid_->get_grid_square_length() * outer_boundary);
			
		float x_wall_max, y_wall_max, z_wall_max;
		float x_wall_min, y_wall_min, z_wall_min;

		float inner_boundary = 1.0f;
		x_wall_min = interior_center.x - (occupancy_grid_->get_grid_square_length() * inner_boundary);
		z_wall_min = interior_center.z - (occupancy_grid_->get_grid_square_length() * inner_boundary);

		x_wall_max = interior_center.x + (occupancy_grid_->get_grid_square_length() * inner_boundary);
		z_wall_max = interior_center.z + (occupancy_grid_->get_grid_square_length() * inner_boundary);

		glm::vec3 v;
		//float power = 3.0f;

		float epsilon = 2.f * occupancy_grid_->get_grid_square_length();

		float normalizingConstant = (outer_boundary - inner_boundary) * occupancy_grid_->get_grid_square_length();

		if (std::abs(position_.z - interior_center.z) < epsilon) {
			if (position_.x >= x_min && position_.x <= interior_center.x) {
				float dist = 1.0 - std::abs(position_.x - x_wall_min) / normalizingConstant;  // ranges 0 to 1
				dist = pow(dist, bounce_function_power_);
				float bounce_velocity = -max_velocity_ * dist;
				v.x = bounce_velocity;
			}
			else if (position_.x <= x_max && position_.x >= interior_center.x) {
				float dist = 1.0 - std::abs(position_.x - x_wall_max) / normalizingConstant;  // ranges 0 to 1
				dist = pow(dist, bounce_function_power_);
				float bounce_velocity = max_velocity_ * dist;
				v.x = bounce_velocity;
			}
		}

		if (std::abs(position_.x - interior_center.x) < epsilon) {
			if (position_.z >= z_min && position_.z <= interior_center.z) {
				float dist = 1.0 - std::abs(position_.z - z_wall_min) / normalizingConstant;  // ranges 0 to 1
				dist = pow(dist, bounce_function_power_);
				float bounce_velocity = -max_velocity_ * dist;
				v.z = bounce_velocity;
			}
			else if (position_.z <= z_max && position_.z >= interior_center.z) {
				float dist = 1.0 - std::abs(position_.z - z_wall_max) / normalizingConstant;  // ranges 0 to 1
				dist = pow(dist, bounce_function_power_);
				float bounce_velocity = max_velocity_ * dist;
				v.z = bounce_velocity;
			}
		}
		bounce_force += v * static_cast<float>(bounce_function_multiplier_);

	}

	separation_force_ = bounce_force;
	return bounce_force;
	
}

glm::vec3 ExperimentalRobot::get_random_velocity() {
	std::random_device rd;
	std::mt19937 eng(rd());
	std::uniform_real_distribution<float> dist(-max_velocity_, max_velocity_);

	glm::vec3 random_velocity(dist(eng), 0.f, dist(eng));

	return random_velocity * random_constant_;
}

bool ExperimentalRobot::not_locally_visited(const glm::ivec3& grid_position) {
	//int map_position = grid_position.x * occupancy_grid_->get_grid_resolution_per_side() + grid_position.z;
	if (local_map_.at(grid_position.x, grid_position.z) == 0) {
		return true;
	}
	return  false;
}

void ExperimentalRobot::mark_locally_covered(const glm::ivec3& grid_position, bool is_interior) {
	//int map_position = grid_position.x * occupancy_grid_->get_grid_resolution_per_side() + grid_position.z;
	//if (is_interior) {
	//	if (local_map_.at(grid_position.x, grid_position.z) == 0) {
	//		local_map_.set(grid_position.x, grid_position.z, SwarmOccupancyTree::INTERIOR_MARK);
	//		local_no_of_unexplored_cells_--;
	//	}
	//}
	if (local_map_.at(grid_position.x, grid_position.z) == 0) {
		int val = is_interior ? SwarmOccupancyTree::INTERIOR_MARK : 1;
		local_map_.set(grid_position.x, grid_position.z, val);
		local_no_of_unexplored_cells_--;
		path_.clear();
	} else if (local_map_.at(grid_position.x, grid_position.z) < SwarmOccupancyTree::INTERIOR_MARK
		&& is_interior) {
		local_map_.set(grid_position.x, grid_position.z, SwarmOccupancyTree::INTERIOR_MARK);
		path_.clear();
	}
}

void ExperimentalRobot::mark_othere_robots_ranges() {
	// mark other robots 
	//for (auto& robot_id : robot_ids_) {
	for (int i = 0; i < current_no_of_robots_; ++i) {
		auto& robot_id = adjacent_robots_[i];
		//if (robots_[robot_id]->get_cluster_id() == cluster_id_
		//	&& !robots_[robot_id]->is_dead()) {
			if (robot_id != id_) {
				auto other_robot_position = robots_[robot_id]->get_position();
				glm::ivec3 other_robot_grid_position = occupancy_grid_->map_to_grid(other_robot_position);

				for (int sensor_level = 1; sensor_level <= sensor_range_; ++sensor_level) {
					int no_of_iter = 0;
					int out_of_bounds = 0;
					for (int z = -sensor_level; z <= sensor_level; ++z) {
						for (int x = -sensor_level; x <= sensor_level; ++x) {
							if (std::abs(x) == sensor_level
								|| std::abs(z) == sensor_level) {
								glm::ivec3 cell_position = other_robot_grid_position + glm::ivec3(x, 0, z);
								no_of_iter++;
								if (!occupancy_grid_->is_out_of_bounds(cell_position)
									&& not_locally_visited(cell_position)) {
										float distance = glm::length(glm::vec3(other_robot_grid_position - cell_position));
										if (distance < (sensor_range_ - 2)) {
											mark_locally_covered(cell_position, false);
										}
										if (!occupancy_grid_->is_interior(cell_position)) {
											if (render_) {
												//explored_mutex_.lock();
												//explored_cells_.push_back(cell_position);
												//explored_mutex_.unlock();
											}
										}
										//if (id_ == 2) {
										//	std::string str = std::to_string(robot_id) + " " + std::to_string(local_no_of_unexplored_cells_) + " ";
										//	SwarmUtils::print_vector(str, cell_position);
										//}
								}
							}
						}
					}
				}
			}
		//}
	}

}


void ExperimentalRobot::update_adjacent_and_interior(const glm::vec3& previous_position, const glm::vec3& current_position) {
	if (current_position == previous_position && adjacent_cells_.size() > 0) {
		interior_updated_ = false;
		return;
	}

	adjacent_cells_.clear();
	adjacent_cells_.reserve(std::pow(sensor_range_ * 2 + 1, 2));

	////get_adjacent_cells(occupancy_grid_->map_to_position(current_position), adjacent_cells_);
	//int sensor_range = sensor_range_;
	//try {
	//	auto position = position_;
	//	//auto grid_pos = current_position;
	//	//occupancy_grid_->get_adjacent_cells(grid_pos, cells, sensor_range_);

	//	int y = 0;
	//	//std::vector<glm::ivec3> cells;
	//	//cells.reserve(9);

	//	for (int x = -sensor_range; x < sensor_range + 1; ++x) {
	//		for (int z = -sensor_range; z < sensor_range + 1; ++z) {
	//			//if (x != 0 && z != 0) {
	//			glm::ivec3 adjacent_operator(x, y, z);
	//			glm::ivec3 adjacent_cell = position + adjacent_operator;
	//			if (!mm::Quadtree<int>::is_out_of_bounds(adjacent_cell.x, adjacent_cell.z)) {
	//				cells.push_back(adjacent_cell);
	//			}
	//			//}
	//		}
	//	}
	//} catch (OutOfGridBoundsException& ex) {
	//	// ignore
	//}

	////interior_cells_ = get_interior_cell_positions(adjacent_cells_);

	//std::vector<glm::vec3> interior_cell_positions;
	//interior_cell_positions.reserve(10);
	//for (auto& adjacent_cell : adjacent_cells) {
	//	if (is_interior(adjacent_cell)) {
	//		glm::vec3 adj_pos;
	//		mm::Quadtree<int>::map_to_position(adjacent_cell.x, adjacent_cell.z, adj_pos.x, adj_pos.z);
	//		interior_cell_positions.push_back(adj_pos);
	//	}
	//}
	////return interior_cell_positions;

	////for (auto& interior_cell : interior_cells_) {
	////	for (auto itr = adjacent_cells_.begin(); itr != adjacent_cells_.end(); ) {
	////		if (!VisibilityQuadrant::visbility_quadrant(sensor_range_)->is_sensor_cell_visible(current_position, occupancy_grid_->map_to_grid(interior_cell), *itr)) {
	////			itr = adjacent_cells_.erase(itr);
	////		} else {
	////			++itr;
	////		}
	////	}
	////}

	//for (auto& interior_cell : interior_cells_) {
	//	for (auto itr = adjacent_cells_.begin(); itr != adjacent_cells_.end(); ) {
	//		if (!VisibilityQuadrant::visbility_quadrant(sensor_range_)->is_sensor_cell_visible(current_position, occupancy_grid_->map_to_grid(interior_cell), *itr)) {
	//			itr = adjacent_cells_.erase(itr);
	//		} else {
	//			++itr;
	//		}
	//	}
	//}
	interior_updated_ = true;
}

void ExperimentalRobot::update_adjacent_and_interior_memory_save(const glm::vec3& previous_position, const glm::vec3& current_position) {
	//if (current_position == previous_position && adjacent_cells_.size() > 0) {
	if (current_position == previous_position && current_adjacent_cells_ > 0) {
		interior_updated_ = false;
		return;
	}

	// init
	current_adjacent_cells_ = 0;
	current_interior_cells_ = 0;

	//adjacent_cells_.clear();
	//adjacent_cells_.reserve(std::pow(sensor_range_ * 2 + 1, 2));

	//get_adjacent_cells(occupancy_grid_->map_to_position(current_position), adjacent_cells_);
	int sensor_range = sensor_range_;
	try {
		glm::ivec3 grid_position = current_position;
		//auto grid_pos = current_position;
		//occupancy_grid_->get_adjacent_cells(grid_pos, cells, sensor_range_);

		int y = 0;
		//std::vector<glm::ivec3> cells;
		//cells.reserve(9);

		for (int x = -sensor_range; x < sensor_range + 1; ++x) {
			for (int z = -sensor_range; z < sensor_range + 1; ++z) {
				//if (x != 0 && z != 0) {
				glm::ivec3 adjacent_operator(x, y, z);
				glm::ivec3 adjacent_cell = grid_position + adjacent_operator;
				if (!occupancy_grid_->is_out_of_bounds(adjacent_cell)) {
					if (occupancy_grid_->is_interior(adjacent_cell)) {
						
						//mm::Quadtree<int>::map_to_position(adjacent_cell.x, adjacent_cell.z, adj_pos.x, adj_pos.z);
						glm::vec3 adj_pos = occupancy_grid_->map_to_position(adjacent_cell);
						interior_cells_[current_interior_cells_++] = (adj_pos);
					}
					adjacent_cells_[current_adjacent_cells_].cell = adjacent_cell;
					adjacent_cells_[current_adjacent_cells_].visible = true;
					current_adjacent_cells_++;
					//cells.push_back(adjacent_cell);
				}
				//if (!occupancy_grid_->is_out_of_bounds(adjacent_cell.x, adjacent_cell.z)) {
				//	cells.push_back(adjacent_cell);
				//}
				//}
			}
		}
	} catch (OutOfGridBoundsException& ex) {
		// ignore
	}

	//interior_cells_ = get_interior_cell_positions(adjacent_cells_);

	//std::vector<glm::vec3> interior_cell_positions;
	//interior_cell_positions.reserve(10);
	//for (auto& adjacent_cell : adjacent_cells) {
	//	if (is_interior(adjacent_cell)) {
	//		glm::vec3 adj_pos;
	//		mm::Quadtree<int>::map_to_position(adjacent_cell.x, adjacent_cell.z, adj_pos.x, adj_pos.z);
	//		interior_cell_positions.push_back(adj_pos);
	//	}
	//}
	//return interior_cell_positions;

	//for (auto& interior_cell : interior_cells_) {
	//	for (auto itr = adjacent_cells_.begin(); itr != adjacent_cells_.end(); ) {
	//		if (!VisibilityQuadrant::visbility_quadrant(sensor_range_)->is_sensor_cell_visible(current_position, occupancy_grid_->map_to_grid(interior_cell), *itr)) {
	//			itr = adjacent_cells_.erase(itr);
	//		} else {
	//			++itr;
	//		}
	//	}
	//}

	//for (auto& interior_cell : interior_cells_) {
	//	for (auto itr = adjacent_cells_.begin(); itr != adjacent_cells_.end(); ) {
	//		if (!VisibilityQuadrant::visbility_quadrant(sensor_range_)->is_sensor_cell_visible(current_position, occupancy_grid_->map_to_grid(interior_cell), *itr)) {
	//			itr = adjacent_cells_.erase(itr);
	//		} else {
	//			++itr;
	//		}
	//	}
	//}
		
	for (auto i = 0; i < current_interior_cells_; ++i) {
		for (auto k = 0; k < current_adjacent_cells_; ++k) {
			auto& curr_adj_cell = adjacent_cells_[k];
			if (curr_adj_cell.is_visible() && !VisibilityQuadrant::visbility_quadrant(sensor_range_)->
				is_sensor_cell_visible(current_position,
				occupancy_grid_->map_to_grid(interior_cells_[i]), curr_adj_cell.cell)) {
				curr_adj_cell.visible = false;
			} 
		}
	}
	interior_updated_ = true;
}

void ExperimentalRobot::get_other_robots_memory_wise() {
	//std::cout << current_adjacent_cells_ << ", "<< current_no_of_robots_ << "\n";
	collision_grid_->find_adjacent_robots_memory_save(id_, adjacent_cells_, current_adjacent_cells_, adjacent_robots_, current_no_of_robots_);
}

#define LOCAL

void ExperimentalRobot::update(int timestamp) {

	//std::chrono::milliseconds current_timestamp = std::chrono::duration_cast<std::chrono::milliseconds>(
	//	std::chrono::system_clock::now().time_since_epoch());

	//if (last_updated_time_ < 0) {
	//	last_updated_time_ = current_timestamp.count();
	//	return;
	//}
	auto current_grid_position = occupancy_grid_->map_to_grid(position_);
	if (occupancy_grid_->is_out_of_bounds(current_grid_position)
		|| occupancy_grid_->is_interior(current_grid_position)) {
		throw OutOfGridBoundsException("Out of bounds");
	}

	if (dead_) {
		return;
	}

	const float delta_time = 2500; // in ms

	if (death_time_ > 0 && timestamp > death_time_) {
		dead_ = true;
	}

	current_timestamp_ = timestamp;

	//robot_ids_ = get_other_robots(adjacent_cells_);
	get_other_robots_memory_wise();

	if (timestamp > 0 && ((timestamp % measurement_time_step_) == 0)) {
		double avg_inverse_distance = 0.0;
		for (int i = 0; i < current_no_of_robots_; ++i) {
			auto& robot_id = adjacent_robots_[i];
		//for (auto& robot_id : robot_ids_) {
			auto other_robot_pos = robots_[robot_id]->get_position();
			float length = glm::length(other_robot_pos - position_);
			float inverse_length = 0.f;
			if (length > 1e-6) {
				inverse_length = 1.f / length;
			}
			avg_inverse_distance += inverse_length;
		}
		if (current_no_of_robots_ > 0) {
			avg_inverse_distance /= current_no_of_robots_;
		}
		//if (robot_ids_.size() > 0) {
		//	avg_inverse_distance /= robot_ids_.size();
		//}
		occlusions_per_timestamp_map_[timestamp] = avg_inverse_distance;

		for (int k = 0; k < current_interior_cells_; ++k) {
			auto& interior = interior_cells_[k];
		//for (auto& interior : interior_cells_) {
			occupancy_grid_->mark_perimeter_covered_by_robot(occupancy_grid_->map_to_grid(interior), timestamp, id_, timestamp);
		}
	}

	glm::vec3 separation_velocity = calculate_separation_velocity();
	glm::vec3 alignment_velocity = calculate_alignment_velocity();
	glm::vec3 clustering_velocity = calculate_clustering_velocity();
#ifdef LOCAL
	//glm::vec3 explore_velocity = calculate_local_explore_velocity();
	glm::vec3 explore_velocity = calculate_astar_explore_velocity();
#else
	glm::vec3 explore_velocity = calculate_explore_velocity();
#endif
	glm::vec3  random_velocity = get_random_velocity();
	glm::vec3 obstacle_avoidance_velocity = calculate_obstacle_avoidance_velocity();

	velocity_ += separation_velocity;
	velocity_ += alignment_velocity;
	velocity_ += clustering_velocity;
	velocity_ += explore_velocity;
	velocity_ += obstacle_avoidance_velocity;

	//if (timestamp % 100 == 0) {
	//	velocity_ += random_velocity;
	//}

	velocity_.y = 0.f;

	// euler integration
	glm::vec3 old_position = position_;
	if (glm::length(velocity_) > max_velocity_) {
		auto normalized_velocity = glm::normalize(velocity_);
		velocity_ = max_velocity_ * normalized_velocity;
	}

	position_ += velocity_ * delta_time / 1000.f / 10.f;

#ifdef LOCAL
	auto current_cell = occupancy_grid_->map_to_grid(previous_position_);
	auto previous_cell = occupancy_grid_->map_to_grid(previous_nminus2_position_);

	if (!global_explore_) {
		if (current_cell != previous_cell) {
			mark_othere_robots_ranges();
		}
	}
#endif
	//bool is_colliding = false;
	//is_colliding |= is_colliding_precisely(interior_cells_);

	//if (is_colliding) {
	//	//position_ = old_position;
	//	//velocity_ = glm::vec3(0.f);
	//	//velocity_ = -1 * 0.8f * velocity_;
	//	int v = 0;
	//}

	//for (auto& interior : interior_cells_) {
	//	occupancy_grid_->mark_perimeter_covered_by_robot(occupancy_grid_->map_to_grid(interior), timestamp, id_);
	//}


	// update grid data structure
	int explored = id_;
	glm::ivec3 grid_position = occupancy_grid_->map_to_grid(position_);
	glm::ivec3 previous_grid_position = occupancy_grid_->map_to_grid(previous_position_);
	collision_grid_->update(id_, previous_grid_position, grid_position);

	update_adjacent_and_interior_memory_save(previous_grid_position, grid_position);


	if (figure_mode_) {
		if (timestamp % 10 == 0 && timestamp > 3000) {
			if (render_) {
				explored_mutex_.lock();
				vis_poo_cells_.push_back(position_);
				explored_mutex_.unlock();
			}
		}
	}

	if (interior_updated_) {
		for (int i = 0; i < current_adjacent_cells_; ++i) {
		//for (auto sensored_cell : adjacent_cells_) {
			auto& visiblility_aware_sensored_cell = adjacent_cells_[i];

			// continue on only if visible
			if (!visiblility_aware_sensored_cell.is_visible()) {
				continue;
			}
			auto& sensored_cell = visiblility_aware_sensored_cell.cell;

			float distance = glm::length(glm::vec3(grid_position - sensored_cell));

			// discovery range is not needed
			//if (distance <= discovery_range_) {

#ifdef LOCAL
			// interior or not we need to mark as covered
			if (distance < (sensor_range_ - 2)) {
				if (!global_explore_) {
					bool is_interior_cell = occupancy_grid_->is_interior(sensored_cell);
					mark_locally_covered(sensored_cell, is_interior_cell);
				}

				if (path_.size() > 0) {
					if (sensored_cell == path_.front() && 
						glm::distance(glm::vec3(sensored_cell), glm::vec3(grid_position)) < 2.f) {
						// we are there 
						path_.pop_front();
					}
				}
			}
#endif
			if (!occupancy_grid_->is_interior(sensored_cell)) {
				occupancy_grid_->set(sensored_cell.x, sensored_cell.z, explored);

				// We need to go one extra grid to cover interior, so make sure it's always less than 1
				if (distance < (sensor_range_ - 2)) {
					if (render_) {
						explored_mutex_.lock();
						if (explored_cell_count_ < max_render_cell_size_ - 1) {
							//vis_explored_cells_[explored_cell_count_++] = (sensored_cell);
						}
						explored_mutex_.unlock();
					}
					occupancy_grid_->mark_explored_in_perimeter_list(sensored_cell);
				}
			} else {
				if (render_) {
					explored_mutex_.lock();
					//interior_explored_cells_.push_back(sensored_cell);
					if (interior_explored_cell_count_ < max_render_cell_size_ - 1) {
						vis_interior_explored_cells_[interior_explored_cell_count_++] = (sensored_cell);
					}
					explored_mutex_.unlock();
				}
				occupancy_grid_->mark_explored_in_interior_list(sensored_cell);
			}
		}
		//reconstruct_points();
	}

	previous_nminus2_position_ = previous_position_;
	previous_position_ = position_;
	//	accumulator_ -= delta_time;
	//}
}

