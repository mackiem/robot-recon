#include "robot.h"
#include <chrono>
#include <glm/gtc/matrix_transform.hpp> // glm::translate, glm::rotate, glm::scale, glm::perspective
#include <cmath>
#include <queue>
#include "swarmutils.h"

int Robot::MAX_DEPTH = 10;


void Robot::set_explore_constant(float constant) {
	explore_constant_ = constant;
}

void Robot::set_separation_constant(float constant) {
	separation_constant_ = constant;
}

void Robot::set_work_constant(float constant) {
	work_constant_ = constant;
}

void Robot::set_grid_overlay(GridOverlay* overlay) {
	overlay_ = overlay;
}

void Robot::change_state(RobotState state) {
	current_robot_state_ = state;
}

void Robot::change_state(SwarmingState state) {
	current_swarming_state_ = state;
}

void Robot::update_robots(const std::vector<std::shared_ptr<Robot>>& robots) {
	robots_ = robots;
}

std::vector<glm::vec3> Robot::get_interior_cell_positions(const std::vector<glm::ivec3>& grid_positions) const {
	return occupancy_grid_->find_adjacent_interiors(grid_positions);
}

std::vector<int> Robot::get_other_robots(const std::vector<glm::ivec3>& grid_positions) const {
	return collision_grid_->find_adjacent_robots(id_, grid_positions);
	
}

void Robot::get_adjacent_cells(const glm::vec3& position, std::vector<glm::ivec3>& cells) const {
	occupancy_grid_->get_adjacent_cells(occupancy_grid_->map_to_grid(position), cells);
}

Robot::Robot(UniformLocations& locations, unsigned int id, std::shared_ptr<SwarmOccupancyTree> octree, std::shared_ptr<SwarmCollisionTree> collision_tree, 
	double explore_constant, double separation_constant, double work_constant,
	double separation_distance, glm::vec3 position, QGLShaderProgram* shader) : VisObject(locations), all_goals_explored_(false),
	accumulator_(0.f), id_(id), timeout_(5000), last_timeout_(0), last_updated_time_(0),
	explore_constant_(explore_constant), separation_constant_(separation_constant), work_constant_(work_constant),
	separation_distance_threshold_(separation_distance), occupancy_grid_(octree), position_(position),  collision_grid_(collision_tree), shader_(shader) {

	std::random_device rd;
	rng_.seed(rd());
	velocity_generator_ = std::uniform_real_distribution<float>(-0.2, 0.2);
	position_generator_ = std::uniform_int_distribution<int>(octree->get_grid_cube_length(), 
		octree->get_grid_cube_length() * (occupancy_grid_->get_grid_resolution_per_side() - 1));
	//position_ = glm::vec3(position_generator_(rng_), 10.f, position_generator_(rng_));
	position_.y = 10.f;
	//SwarmUtils::print_vector("Robot position vector : ", position_);

	collision_grid_->insert(id_, occupancy_grid_->map_to_grid(position_));
	previous_position_ = position_;

	robot_radius_ = 10.f;
	mass_ = 10;
	max_velocity_ = 50;


	distance_to_goal_threshold_ = 5.f;

	travelling_in_bound_ = false;

	cv::Vec4f green(0.f, 1.f, 0.f, 1.f);
	cv::Vec4f red(0.f, 1.f, 0.f, 1.f);
	cv::Vec4f yellow(1.f, 1.f, 0.f, 1.f);

	visualize_force(0, explore_force_, green, true);
	visualize_force(1, separation_force_, red, true);
	visualize_force(2, resultant_direction_, yellow, true);

}

Robot& Robot::operator=(const Robot& other) {
	if (this != &other) {
		id_ = other.id_;
		occupancy_grid_ = other.occupancy_grid_;

		velocity_ = other.velocity_;
		position_ = other.position_;
		timeout_ = other.timeout_;
		last_timeout_ = other.last_timeout_;
		last_updated_time_ = other.last_updated_time_;
		rng_ = other.rng_;
		velocity_generator_ = other.velocity_generator_;
		position_generator_ = other.position_generator_;

		explore_constant_ = other.explore_constant_;
		separation_constant_ = other.separation_constant_;
		work_constant_ = other.work_constant_;

		explore_force_ = other.explore_force_;
		separation_force_ = other.separation_force_;
		work_force_ = other.work_force_;

		// separation force requirements
		//glm::vec3 center_of_mass_;
		//std::vector<Robot> robots_ = other.robots_;
		minimum_separation_distance_ = other.minimum_separation_distance_;
		separation_distance_threshold_ = other.separation_distance_threshold_;

		// exploration force requirements
		// location in grid for frontier calculation
		mass_ = other.mass_;
		max_velocity_ = other.max_velocity_;
	}
	return *this;
}

void Robot::visualize_force(const int& mesh_id, const glm::vec3& force, const cv::Vec4f& color, bool initialize) {

	VertexBufferData bufferdata;
	cv::Vec3f normal(0.f, 1.f, 0.f);

	cv::Vec3f force_vec(force.x, force.y, force.z);

	cv::Vec3f pt_1 = cv::Vec3f(0.f, 0.f, 0.f);
	cv::Vec3f pt_2 = pt_1 + force_vec;

	auto& positions = bufferdata.positions;
	positions.push_back(pt_1);
	positions.push_back(pt_2);

	auto& indices = bufferdata.indices;


	for (auto i = 0u; i < 2; ++i) {
		bufferdata.colors.push_back(color);
		bufferdata.normals.push_back(normal);
		indices.push_back((positions.size() - 2) + i);
	}

	bufferdata.count.push_back(bufferdata.positions.size());
	bufferdata.base_index.push_back(0);
	bufferdata.offset.push_back(0);

	if (initialize) {
		RenderEntity force_entity(GL_LINES, shader_);
		force_entity.upload_data_to_gpu(bufferdata);

		mesh_.push_back(force_entity);

	}
	else {
		if (mesh_id < mesh_.size()) {
			mesh_[mesh_id].upload_data_to_gpu(bufferdata, true);
		}
		else {
			std::cout << "force mesh id greater than force mesh " << std::endl;
		}
	}

}

void Robot::set_show_forces(bool show) {
	show_forces_ = show;
	//hack
	glm::vec3 zero_force;
	cv::Vec4f blue(0.f, 0.f, 1.f, 1.f);
	visualize_force(0, zero_force, blue, false);
	visualize_force(1, zero_force, blue, false);
	visualize_force(2, zero_force, blue, false);
}

glm::vec3 Robot::calculate_force(glm::vec3 move_to_position) {
	auto move_to_vector = (move_to_position - position_);

	//// acceleration needed to get there at max velocity
	float distance = glm::length(move_to_vector);
	float acceleration = (glm::dot(max_velocity_, max_velocity_) - glm::dot(velocity_, velocity_)) / (2 * distance);

	glm::vec3 acceleration_vector = acceleration * glm::normalize(move_to_vector);
	glm::vec3 force = mass_ * acceleration_vector;

	return force;
}

glm::vec3 Robot::calculate_force(glm::vec3 move_to_position, float constant) const {
	// use Hooke's law, F = -kx
	auto move_to_vector = (move_to_position - position_);
	glm::vec3 force = constant * move_to_vector;
	return force;
}

void Robot::calculate_explore_force() {
	// if there's a frontier (unexplore space around me) head towards that
	// naive algorithm - find the first
	
	if (all_goals_explored_) {
		explore_force_ = glm::vec3(0.f);
		return;
	}

	try {
		auto current_cell = occupancy_grid_->map_to_grid(position_);

		// find the near-closest cell that needs exploration
		glm::ivec3 frontier_cell;
		bool something_to_explore = occupancy_grid_->frontier_bread_first_search(current_cell, frontier_cell, MAX_DEPTH);
		if (something_to_explore) {
			auto move_to_position = occupancy_grid_->map_to_position(frontier_cell);

			// force to move to that cell (in 2D)
			move_to_position.y = 10.f;
			explore_force_ = calculate_force(move_to_position, explore_constant_);
#ifdef DEBUG
			//std::cout << "Frontier Cell : [" << frontier_cell.x << ", " << frontier_cell.y << ", " << frontier_cell.z << "]" << std::endl;
#endif
		}
		else {
			// we are done: nothing to explore
			explore_force_ = glm::vec3(0.f);
			all_goals_explored_ = true;
#ifdef DEBUG
			std::cout << "Nothing to explore..." << std::endl;
#endif
		}
	}
	catch (OutOfGridBoundsException& ex) {
		//std::cout << "Position is out of bounds : [" << position_.x << ", " << position_.y << ", " << position_.z << "]" << std::endl;
		explore_force_ = glm::vec3(0.f);
	}
}

void Robot::calculate_separation_force(const std::vector<int>& other_robots, const std::vector<glm::vec3>& interior_cells) {

	separation_force_ = glm::vec3(0.f, 0.f, 0.f);
	
	float red_distance_threshold_ = separation_distance_threshold_;
	float amber_distance_threshold_ = red_distance_threshold_ + 15.f;

	glm::vec3 amber_separation_force;

	for (auto& other_robot_id : other_robots) {
		auto separation_vector = position_ - robots_[other_robot_id]->position_;
		float distance = glm::length(separation_vector);

		// minimum separation distance + some threshold??

		if (distance < amber_distance_threshold_ && distance > red_distance_threshold_) {
			float distance_to_travel = (amber_distance_threshold_ - distance);
			glm::vec3 move_to_direction = glm::normalize(separation_vector);
			glm::vec3 move_to_position = position_ + distance_to_travel * move_to_direction;
			// amber zone
			// move away if only a lower id
			if (id_ < other_robot_id) {
					separation_force_ += calculate_force(move_to_position, separation_constant_);
					//amber_separation_force += calculate_force(move_to_position, separation_constant_);
			}

		} else if (distance <= red_distance_threshold_) {
			float distance_to_travel = (red_distance_threshold_ - distance);
			glm::vec3 move_to_direction = glm::normalize(separation_vector);
			glm::vec3 move_to_position = position_ + distance_to_travel * move_to_direction;
			// red zone
			// move away, regardless of id
			if (id_ != other_robot_id) {
			//if (id_ < other_robot_id) {
				separation_force_ += calculate_force(move_to_position, separation_constant_);
			}
		} else {
			// green zone
			// do nothing
		}
	}

	red_distance_threshold_ = occupancy_grid_->get_grid_cube_length() + robot_radius_;
	amber_distance_threshold_ = red_distance_threshold_ + 10;

	for (auto& other_cell_position : interior_cells) {
		auto move_away_vector = position_ - other_cell_position;

		glm::vec3 along_the_wall_vector;

		glm::ivec3 interior_grid_position = occupancy_grid_->map_to_grid(other_cell_position);
		int interior_x = interior_grid_position.x;
		int interior_z = interior_grid_position.z;
		
		if (interior_x == 0 || interior_x == (occupancy_grid_->get_grid_resolution_per_side() - 1)) {
			along_the_wall_vector += glm::vec3(0.f, other_cell_position.y, 1.f);
		}

		if (interior_z == 0 || interior_z == (occupancy_grid_->get_grid_resolution_per_side() - 1)) {
			along_the_wall_vector += glm::vec3(1.f, other_cell_position.y, 0.f);
		}


		auto projection_of_move_away_along_the_wall = along_the_wall_vector * (glm::dot(explore_force_, along_the_wall_vector))
			/ glm::dot(along_the_wall_vector, along_the_wall_vector);

		float distance = glm::length(move_away_vector);

		// minimum separation distance + some threshold??
		float distance_to_travel = (separation_distance_threshold_ - distance);
		glm::vec3 move_to_direction = glm::normalize(projection_of_move_away_along_the_wall);
		glm::vec3 move_to_position = position_ + distance_to_travel * move_to_direction;
	}

	separation_force_.y = 0.f;
}

void Robot::calculate_work_force() {
	work_force_ = glm::vec3(0.f, 0.f, 0.f);

}

glm::vec3 Robot::calculate_resultant_direction(const std::vector<int>& other_robots, const std::vector<glm::vec3>& interior_cells) {

	// calculate important forces
	calculate_explore_force();
	calculate_separation_force(other_robots, interior_cells);
	calculate_work_force();

	// add forces together to get direction of motion
	glm::vec3 actual_force = explore_force_ + separation_force_ + work_force_;
	glm::vec3 force_direction;
	if (glm::length(actual_force) > 1e-6) {
		 force_direction = glm::normalize(actual_force);
	}
	return force_direction;

#ifdef DEBUG
	//if (force.y > 0.f) {
	//std::cout << "Robot : " << id_ << std::endl;
	//std::cout << "explore : [" << explore_force_.x << ", " << explore_force_.y << ", " << explore_force_.z << "]" << std::endl;
	//std::cout << "separation : [" << separation_force_.x << ", " << separation_force_.y << ", " << separation_force_.z << "]" << std::endl;
	//std::cout << "out_of_bounds : [" << out_of_bounds_force_.x << ", " << out_of_bounds_force_.y << ", " << out_of_bounds_force_.z << "]" << std::endl;
	//std::cout << std::endl;
	// }
#endif
}

bool Robot::is_colliding(const glm::vec3& other_object_position, float radius) const {
	// using circle intersection check - (R0 - R1) ^ 2 <= (x0 - x1) ^ 2 + (y0 - y1) ^ 2 <= (R0 + R1) ^ 2

	glm::vec3 position_difference = other_object_position - position_;
	if (glm::dot(position_difference, position_difference) <= std::pow(robot_radius_ + radius, 2)) {
		return true;
	}
	return false;
}

bool Robot::is_colliding_with_interior(const std::vector<glm::vec3>& interior_positions) const {
	for (auto& interior : interior_positions) {
		if (is_colliding(interior, (occupancy_grid_->get_grid_cube_length() / 2.f))) {
			return true;
		}
	}
	return false;
}

bool Robot::is_colliding_with_robots(const std::vector<int>& robot_ids) const {
	for (auto& robot_id : robot_ids) {
		if (is_colliding(robots_[robot_id]->position_, robot_radius_)) {
			return true;
		}
	}
	return false;
}

void Robot::update(glm::mat4 global_model) {

	std::chrono::milliseconds current_timestamp = std::chrono::duration_cast<std::chrono::milliseconds>(
		std::chrono::system_clock::now().time_since_epoch());

	if (last_updated_time_ <= 0) {

		last_updated_time_ = current_timestamp.count();

	} else {

		auto delta_time = (current_timestamp.count() - last_updated_time_) / 1000.f;

		float time_step_duration = 1.f / 30.f; // 60 hz 

		accumulator_ += delta_time;

		// get next direction for motion
		std::vector<glm::ivec3> adjacent_cells;
		adjacent_cells.reserve(9);
		get_adjacent_cells(position_, adjacent_cells);
		auto robot_ids = get_other_robots(adjacent_cells);
		auto interior_cells = get_interior_cell_positions(adjacent_cells);
	    resultant_direction_ = calculate_resultant_direction(robot_ids, interior_cells);

		// calculate new position
		float small_dist = occupancy_grid_->get_grid_cube_length() * 0.1f;
		glm::vec3 position_delta = resultant_direction_ * small_dist;
		position_ += position_delta;

		// update visualization
		if (show_forces_) {
			visualize_force(0, explore_force_, cv::Vec4f(0.f, 0.f, 1.f, 0.f), false);
			visualize_force(1, separation_force_, cv::Vec4f(1.f, 0.f, 0.f, 0.f), false);
		}

		// update rendered mesh
		for (auto& render_entity : mesh_) {
			glm::mat4 translate_model = glm::translate(glm::mat4(1.f), position_);
			render_entity.set_model(translate_model * render_entity.get_initial_model());
		}

		// update grid data structure
   		int explored = id_;
		try {
			glm::ivec3 grid_position = occupancy_grid_->map_to_grid(position_);
			glm::ivec3 previous_grid_position = occupancy_grid_->map_to_grid(previous_position_);
			collision_grid_->update(id_, previous_grid_position, grid_position);
	
			if (!occupancy_grid_->is_interior(grid_position)) {
				occupancy_grid_->set(grid_position.x, grid_position.z, explored);
				overlay_->update_grid_position(grid_position);
			}
		} catch (OutOfGridBoundsException& exception) {
			// ignore
		}

		previous_position_ = position_;
	}

	last_updated_time_ = current_timestamp.count();
}

