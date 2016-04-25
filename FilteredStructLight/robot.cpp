#include "robot.h"
#include <chrono>
#include <glm/gtc/matrix_transform.hpp> // glm::translate, glm::rotate, glm::scale, glm::perspective
#include <cmath>
#include <queue>
#include "swarmutils.h"

int Robot::MAX_DEPTH = 10;

#define PI 3.14159


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
	try {
		auto grid_pos = occupancy_grid_->map_to_grid(position);
		occupancy_grid_->get_adjacent_cells(grid_pos, cells, sensor_range_);
	} catch (OutOfGridBoundsException& ex) {
		// ignore
	}
}

Robot::Robot(UniformLocations& locations, unsigned int id, std::shared_ptr<SwarmOccupancyTree> octree, std::shared_ptr<SwarmCollisionTree> collision_tree, 
	double explore_constant, double separation_constant, double alignment_constant, double cluster_constant, double perimeter_constant, double work_constant,
	double separation_distance, glm::vec3 position, QGLShaderProgram* shader) : VisObject(locations), all_goals_explored_(false),
	accumulator_(0.f), id_(id), position_(position), timeout_(5000), last_timeout_(0),
	last_updated_time_(0), explore_constant_(explore_constant), separation_constant_(separation_constant), alignment_constant_(alignment_constant),
	work_constant_(work_constant), perimeter_constant_(perimeter_constant),
	cluster_constant_(cluster_constant), separation_distance_threshold_(separation_distance), occupancy_grid_(octree),  collision_grid_(collision_tree), shader_(shader) {

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

	sensor_range_ = 5;

	attraction_distance_threshold_ = separation_distance_threshold_ + 10;

	distance_to_goal_threshold_ = 5.f;

	travelling_in_bound_ = false;

	cv::Vec4f green(0.f, 1.f, 0.f, 1.f);
	cv::Vec4f red(1.f, 0.f, 0.f, 1.f);
	cv::Vec4f yellow(1.f, 1.f, 0.f, 1.f);
	cv::Vec4f blue(0.f, 0.f, 1.f, 1.f);
	cv::Vec4f black(0.f, 0.f, 0.f, 1.f);
	cv::Vec4f orange = (cv::Vec4f(255.f, 153.f, 153.f, 255.f)) / 255.f;
	cv::Vec4f pink = cv::Vec4f(0.f, 255.f, 204.f, 255.f) / 255.f;


	init_force_visualization(0, explore_force_, blue);
	init_force_visualization(1, separation_force_, green);
	init_force_visualization(2, resultant_direction_, black);
	init_force_visualization(3, perimeter_force_, yellow);
	init_force_visualization(4, cluster_force_, orange);
	init_force_visualization(5, alignment_force_, black);


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

void Robot::init_force_visualization(const int& mesh_id, const glm::vec3& force, const cv::Vec4f& color) {

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

	RenderEntity force_entity(GL_LINES, shader_);
	force_entity.upload_data_to_gpu(bufferdata);

	mesh_.push_back(force_entity);

}

void Robot::update_force_visualization(const int& mesh_id, const glm::vec3& force) {
	RenderEntity& entity = mesh_[mesh_id];

	cv::Vec3f force_vec(force.x, force.y, force.z);

	glBindVertexArray(entity.vao_);
	glBindBuffer(GL_ARRAY_BUFFER, entity.vbo_[RenderEntity::POSITION]);
	glBufferSubData(GL_ARRAY_BUFFER, 1 * sizeof(cv::Vec3f), 
		1 * sizeof(cv::Vec3f), &force_vec[0]);

	glBindVertexArray(0);
	
}

void Robot::set_show_forces(bool show) {
	show_forces_ = show;
	//hack
	glm::vec3 zero_force;
	cv::Vec4f blue(0.f, 0.f, 1.f, 1.f);
	update_force_visualization(0, zero_force);
	update_force_visualization(1, zero_force);
	update_force_visualization(2, zero_force);
}

glm::vec3 Robot::calculate_direction(glm::vec3 move_to_position) {
	auto move_to_vector = (move_to_position - position_);

	//// acceleration needed to get there at max velocity
	float distance = glm::length(move_to_vector);
	float acceleration = (glm::dot(max_velocity_, max_velocity_) - glm::dot(velocity_, velocity_)) / (2 * distance);

	glm::vec3 acceleration_vector = acceleration * glm::normalize(move_to_vector);
	glm::vec3 force = mass_ * acceleration_vector;

	return force;
}

glm::vec3 Robot::calculate_direction(glm::vec3 move_to_position, float constant) const {
	// use Hooke's law, F = -kx
	auto move_to_vector = (move_to_position - position_);
	glm::vec3 direction;
	if (glm::length(move_to_vector) > 1e-6) {
		 direction = glm::normalize(move_to_vector);
	}
	glm::vec3 constant_times_direction = constant * direction;
	return constant_times_direction;
}

void Robot::calculate_explore_force() {
	// if there's a frontier (unexplore space around me) head towards that
	// naive algorithm - find the first
	
	try {
		auto current_cell = occupancy_grid_->map_to_grid(position_);
		glm::ivec3 explore_cell;
		bool something_to_explore = occupancy_grid_->next_cell_to_explore(current_cell, explore_cell);

		if (something_to_explore) {
			auto move_to_position = occupancy_grid_->map_to_position(explore_cell);

			// force to move to that cell (in 2D)
			move_to_position.y = 10.f;
			explore_force_ = calculate_direction(move_to_position, explore_constant_);
		} else {
			explore_force_ = glm::vec3(0.f);
		}

	} catch (OutOfGridBoundsException& ex) {
		//std::cout << "Position is out of bounds : [" << position_.x << ", " << position_.y << ", " << position_.z << "]" << std::endl;
		explore_force_ = glm::vec3(0.f);
	}


//	if (all_goals_explored_) {
//		explore_force_ = glm::vec3(0.f);
//		return;
//	}
//
//	try {
//		auto current_cell = occupancy_grid_->map_to_grid(position_);
//
//		// find the near-closest cell that needs exploration
//		glm::ivec3 frontier_cell;
//		bool something_to_explore = occupancy_grid_->frontier_bread_first_search(current_cell, frontier_cell, MAX_DEPTH);
//		if (something_to_explore) {
//			auto move_to_position = occupancy_grid_->map_to_position(frontier_cell);
//
//			// force to move to that cell (in 2D)
//			move_to_position.y = 10.f;
//			explore_force_ = calculate_direction(move_to_position, explore_constant_);
//#ifdef DEBUG
//			//std::cout << "Frontier Cell : [" << frontier_cell.x << ", " << frontier_cell.y << ", " << frontier_cell.z << "]" << std::endl;
//#endif
//		}
//		else {
//			// we are done: nothing to explore
//			explore_force_ = glm::vec3(0.f);
//			all_goals_explored_ = true;
//#ifdef DEBUG
//			std::cout << "Nothing to explore..." << std::endl;
//#endif
//		}
//	}
	//catch (OutOfGridBoundsException& ex) {
	//	//std::cout << "Position is out of bounds : [" << position_.x << ", " << position_.y << ", " << position_.z << "]" << std::endl;
	//	explore_force_ = glm::vec3(0.f);
	//}
}

glm::vec3 Robot::calculate_force_from_piecewise_squared_function(const glm::vec3& separation_vector, float constant,
	float distance_from_threshold) {

	// distance from threshold is positive, if closer than the threshold
	glm::vec3 force;
	glm::vec3 movement_direction = glm::normalize(separation_vector);
	force = constant * std::pow(distance_from_threshold, 2) * movement_direction;
	return force;
}

void Robot::calculate_separation_force(const std::vector<int>& other_robots, const std::vector<glm::vec3>& interior_cells) {

	separation_force_ = glm::vec3(0.f, 0.f, 0.f);
	
	for (auto& other_robot_id : other_robots) {
		auto separation_vector = position_ - robots_[other_robot_id]->position_;

		if (other_robot_id != id_) {
			float length = glm::length(separation_vector);
			float distance_from_separation_threshold = separation_distance_threshold_ - length;
			float distance_from_attraction_threshold = attraction_distance_threshold_ - length;
			if (distance_from_separation_threshold > 0) {
				//separation_force_ += calculate_force_from_piecewise_squared_function(separation_vector, separation_constant_, 
				//	distance_from_separation_threshold);
				separation_force_ += calculate_direction(position_ + separation_vector, separation_constant_);
			} else if (distance_from_attraction_threshold < 0) {
				//separation_force_ += calculate_force_from_piecewise_squared_function(-separation_vector, separation_constant_, 
				//	distance_from_attraction_threshold);
			}
		}
	}

	for (auto& interior_cell_position : interior_cells) {
		auto separation_vector = position_ - interior_cell_position;

		float length = glm::length(separation_vector);
		float distance_from_threshold = (separation_distance_threshold_) - length;
		if (distance_from_threshold > 0) {
			//separation_force_ += calculate_force_from_piecewise_squared_function(separation_vector, separation_constant_, distance_from_threshold);
				separation_force_ += calculate_direction(position_ + separation_vector, separation_constant_);
		}
	}

	separation_force_.y = 0.f;
}

void Robot::calculate_work_force() {
	work_force_ = glm::vec3(0.f, 0.f, 0.f);
}


void Robot::calculate_cluster_force(const std::vector<int>& other_robots) {

	cluster_force_ = glm::vec3(0.f, 0.f, 0.f);

	if (other_robots.size() > 0) {
		std::vector<PerimeterPos> other_robot_distance_vector;
		for (auto itr = other_robots.begin(); itr != other_robots.end(); ++itr) {
			auto other_robot = *itr;
			other_robot_distance_vector.push_back(PerimeterPos(glm::length(glm::vec3(occupancy_grid_->map_to_grid(robots_[other_robot]->position_) -
				occupancy_grid_->map_to_grid(position_))), occupancy_grid_->map_to_grid(robots_[other_robot]->position_)));
		}

		std::sort(other_robot_distance_vector.begin(), other_robot_distance_vector.end());

		if (other_robot_distance_vector[other_robots.size() - 1].distance_ < (3)) {
			// too close, implement this as zones
			return;
		}

		glm::vec3 centroid;
		int cluster_size = 0;
		for (auto& other_robots_distance : other_robot_distance_vector) {
			if (other_robots_distance.distance_ > (sensor_range_ - 1)) {
				centroid += occupancy_grid_->map_to_position(other_robots_distance.grid_position_);
				cluster_size++;
			}
		}

		if (cluster_size > 0) {
			centroid /= cluster_size;
			cluster_force_ = calculate_direction(centroid, cluster_constant_);
			cluster_force_.y = 0.f;
		}

	} 	
}

void Robot::calculate_alignment_force(const std::vector<int>& other_robots) {
	alignment_force_ = glm::vec3(0.f, 0.f, 0.f);

	if (other_robots.size() > 0) {
		float delta_time = 0.1f;
		std::vector<glm::vec3> velocities;
		for (auto& other_robot_id : other_robots) {
			if (other_robot_id != id_) {
				auto robot_distance = 
					glm::length(glm::vec3(occupancy_grid_->map_to_grid(position_) - occupancy_grid_->map_to_grid(robots_[other_robot_id]->position_)));
				if (robot_distance > 1) {
					velocities.push_back(robots_[other_robot_id]->resultant_direction_);
				}
			}
		}

		if (velocities.size() < 1) {
			return;
		}

		std::sort(velocities.begin(), velocities.end(), Vec3Comparator());

		glm::vec3 median_velocity = velocities[(velocities.size() + 1) / 2];
		

		//velocity /= other_robots.size();
		auto move_to_position = position_ + median_velocity * delta_time;
		move_to_position.y = 10.f;
		alignment_force_ = calculate_direction(move_to_position, alignment_constant_);
		alignment_force_.y = 0.f;
	}	
	
}

void Robot::calculate_perimeter_force(const std::vector<glm::vec3>& interior_cells) {

	// algorithm:
	// if we have adjacent perimeter, then we go along it
	// if not we have the explore force, that guides us towards perimeter
	// the repulsion force stays the same just in case we get too close
	// basically, a concentric circle model for perimeter exploration, and thereby, 3D reconstruction
	// making use of the boid model (sort of) and adapting it to  repulsion, explore and walk along
	

	// for perimeter, get the closest two
	perimeter_force_ = glm::vec3(0.f, 0.f, 0.f);

	if (interior_cells.size() > 1) {
		std::vector<PerimeterPos> perimeter_vector;
		glm::ivec3 explore_direction;

		for (auto itr = interior_cells.begin(); itr != interior_cells.end(); ++itr) {
			auto perimeter_position = *itr;

			//if (!occupancy_grid_->is_unexplored_perimeter(perimeter_position)
			//	&& !occupancy_grid_->has_explored(perimeter_position)) {
			//	std::vector<glm::ivec3> adjacent_cells;
			//	adjacent_cells.reserve(9);
			//	occupancy_grid_->get_adjacent_cells(perimeter_position, adjacent_cells, 1);
			//	for (auto& adjacent_cell : adjacent_cells) {
			//		if (occupancy_grid_->is_unexplored_perimeter(adjacent_cell)) {
			//			explore_direction = current_cell.grid_position_;
			//		}
			//	}
			//}

			std::vector<glm::ivec3> adjacent_cells;
			adjacent_cells.reserve(9);
			occupancy_grid_->get_adjacent_cells(occupancy_grid_->map_to_grid(perimeter_position), adjacent_cells, 1);
			bool found_non_interior_unexplored_position = false;
			for (auto& adjacent_cell : adjacent_cells) {
				if (!occupancy_grid_->is_interior(adjacent_cell) &&
					!occupancy_grid_->has_explored(adjacent_cell)) {
					found_non_interior_unexplored_position = true;
					break;
				}
			}
				
			if (found_non_interior_unexplored_position) {
				perimeter_vector.push_back(PerimeterPos(glm::length(glm::vec3(occupancy_grid_->map_to_grid(perimeter_position) -
					occupancy_grid_->map_to_grid(position_))), occupancy_grid_->map_to_grid(perimeter_position)));
			}

		}

		if (perimeter_vector.size() > 1) {

			std::sort(perimeter_vector.begin(), perimeter_vector.end());

			//if (perimeter_vector[0].distance_ > (1)) {
			//	// too far, implement this as zones
			//	return;
			//}

			//glm::vec3 perimeter_direction = occupancy_grid_->map_to_position(perimeter_vector[perimeter_vector.size() - 1].grid_position_) -
			//	occupancy_grid_->map_to_position(perimeter_vector[0].grid_position_);
			glm::vec3 perimeter_direction = occupancy_grid_->map_to_position(perimeter_vector[1].grid_position_) -
				occupancy_grid_->map_to_position(perimeter_vector[0].grid_position_);
			perimeter_direction.y = 10.f;

			// project perimeter with last resultant direction, proj (resultant direction) on to (perimeter) = proj_(p)r
			glm::vec3 walk_along_direction;
			if (glm::dot(perimeter_direction, resultant_direction_) > 1e-6) {
				walk_along_direction =
					perimeter_direction * glm::dot(perimeter_direction, resultant_direction_) / glm::dot(resultant_direction_, resultant_direction_);
			}
			else {
				walk_along_direction = perimeter_direction;
			}
			walk_along_direction.y = 0.f;


			perimeter_force_ = calculate_direction(position_ + walk_along_direction, perimeter_constant_);
		}
	}

	//try {
	//	auto current_cell = occupancy_grid_->map_to_grid(position_);
	//	glm::ivec3 perimeter_cell;
	//	bool something_to_explore = occupancy_grid_->find_closest_perimeter(current_cell, perimeter_cell);

	//	if (something_to_explore) {
	//		auto move_to_position = occupancy_grid_->map_to_position(perimeter_cell);

	//		// force to move to that cell (in 2D)
	//		move_to_position.y = 10.f;
	//		perimeter_force_ = calculate_direction(move_to_position, perimeter_constant_);
	//	} else {
	//		perimeter_force_ = glm::vec3(0.f);
	//	}

	//} catch (OutOfGridBoundsException& ex) {
	//	//std::cout << "Position is out of bounds : [" << position_.x << ", " << position_.y << ", " << position_.z << "]" << std::endl;
	//	perimeter_force_ = glm::vec3(0.f);
	//}
	
}

glm::vec3 Robot::calculate_resultant_direction(const std::vector<int>& other_robots, const std::vector<glm::vec3>& interior_cells) {

	// calculate important forces
	calculate_explore_force();
	calculate_separation_force(other_robots, interior_cells);
	calculate_cluster_force(other_robots);
	calculate_alignment_force(other_robots);
	calculate_perimeter_force(interior_cells);
	calculate_work_force();

	// add forces together to get direction of motion
	glm::vec3 actual_force = explore_force_ + separation_force_ + perimeter_force_ + alignment_force_ + cluster_force_ + work_force_;
#ifdef DEBUG
	if (actual_force.y > 0.f) {
		std::cout << "Robot : " << id_ << std::endl;
		SwarmUtils::print_vector("explore", explore_force_);
		SwarmUtils::print_vector("separation", separation_force_);
		SwarmUtils::print_vector("perimeter", perimeter_force_);
		SwarmUtils::print_vector("alignment", alignment_force_);
		SwarmUtils::print_vector("cluster", cluster_force_);
		std::cout << std::endl;
	}
#endif
	glm::vec3 force_direction;
	if (glm::length(actual_force) > 1e-6) {
		 force_direction = glm::normalize(actual_force);
	}
	return force_direction;

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

		// save old position for rollback if colliding
		glm::vec3 old_position = position_;
		position_ += position_delta;

		// check for collision
		if (is_colliding_with_interior(interior_cells) ||
			is_colliding_with_robots(robot_ids)) {
			position_ = old_position;
		} 



		// update visualization
		if (show_forces_) {
			update_force_visualization(0, explore_force_);
			update_force_visualization(1, separation_force_);
			update_force_visualization(3, perimeter_force_);
			update_force_visualization(4, cluster_force_);
			update_force_visualization(5, alignment_force_);
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
				occupancy_grid_->mark_explored_in_list(grid_position);
				overlay_->update_grid_position(grid_position);
			}
		} catch (OutOfGridBoundsException& exception) {
			// ignore
		}

		previous_position_ = position_;
	}

	last_updated_time_ = current_timestamp.count();
}

