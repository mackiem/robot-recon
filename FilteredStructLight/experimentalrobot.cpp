#include "experimentalrobot.h"
#include <chrono>
#include "swarmutils.h"


ExperimentalRobot::ExperimentalRobot(UniformLocations& locations, unsigned id, SwarmOccupancyTree* octree,
	SwarmCollisionTree* collision_tree, double explore_constant, double separation_constant, double alignment_constant,
	double cluster_constant, double perimeter_constant, double work_constant, Range explore_range,
	Range separation_range, Range alignment_range, Range cluster_range, Range perimeter_range,
	Range obstacle_avoidance_near_range, Range obstacle_avoidance_far_range, double sensor_range,
	int discovery_range, int neighborhood_count, double separation_distance, glm::vec3 position,
	QGLShaderProgram* shader, bool render, double magic_k, bool collide_with_robots)

	: Robot(locations,
	id, octree, collision_tree, explore_constant, separation_constant,
	alignment_constant, cluster_constant, perimeter_constant,
	work_constant,
	explore_range, separation_range, alignment_range, cluster_range, perimeter_range, obstacle_avoidance_near_range,
	obstacle_avoidance_far_range, sensor_range, discovery_range, neighborhood_count,
	separation_distance, position, shader, render, magic_k, collide_with_robots)
{
	random_direction_ = glm::vec3(1.f, 0.f, 0.f);
	max_velocity_ = 20.f;
}

ExperimentalRobot::~ExperimentalRobot()
{
}

glm::vec3 ExperimentalRobot::calculate_separation_velocity() {
	const float max_distance = 20.f;
	glm::vec3 c;
	for (auto& robot : robots_) {
		if (robot->get_id() != id_) {
			float distance_apart = glm::length(robot->get_position() - position_);
			if (distance_apart < max_distance) {
				auto move_away_vector = (robot->get_position() - position_);
				//c -= glm::vec3(move_away_vector.x * move_away_vector.x, move_away_vector.y * move_away_vector.y,
				//	move_away_vector.z * move_away_vector.z);
				auto inverse_normalized_dist = (max_distance - distance_apart) / max_distance;
				float normalize_constant = std::pow(inverse_normalized_dist, 2);
				c -= normalize_constant * move_away_vector;
			}
		}
	}
	return c;
}

glm::vec3 ExperimentalRobot::calculate_alignment_velocity() {
	glm::vec3 v;
	int count = 0;
	for (auto& robot : robots_) {
		if (robot->get_id() != id_) {
			v += robot->get_velocity();
			count++;
		}
	}
	if (count > 0) {
		v /= static_cast<float>(count);
	}
	return (v - velocity_) / 8.f;
}

glm::vec3 ExperimentalRobot::bounce_off_corners_velocity() {
	float x_max, y_max, z_max;
	float x_min, y_min, z_min;
	x_min = y_min = z_min = 0;
	x_max = y_max = z_max = occupancy_grid_->get_grid_cube_length() * occupancy_grid_->get_grid_resolution_per_side();
	
	const float opposite_vel = 10.f;
	glm::vec3 v;

	if (position_.x < x_min) {
		v.x = opposite_vel;
	} else if (position_.x > x_max) {
		v.x = -opposite_vel;
	}

	//if (position_.y < y_min) {
	//	v.y = opposite_vel;
	//} else if (position_.y > y_may) {
	//	v.y = -opposite_vel;
	//}
	
	if (position_.z < z_min) {
		v.z = opposite_vel;
	} else if (position_.z > z_max) {
		v.z = -opposite_vel;
	}
	return v;
}

glm::vec3 ExperimentalRobot::calculate_clustering_velocity() {
	glm::vec3 pc;
	int count = 0;
	for (auto& robot : robots_) {
		if (robot->get_id() != id_) {
			pc += robot->get_position();
			count++;
		}
	}
	if (count > 0) {
		pc /= static_cast<float>(count);
	}

	float normalizing_constant = std::pow(glm::length(pc - position_) / 600.f, 2);
	glm::vec3 cluster_velocity = normalizing_constant * (pc - position_) / 100.f;
	return cluster_velocity;
}

glm::vec3 ExperimentalRobot::calculate_random_direction() {
	glm::vec3 v;
	
	return v;
}

glm::vec3 ExperimentalRobot::calculate_explore_velocity() {
	glm::vec3 explore_velocity;

	try {
		auto current_cell = occupancy_grid_->map_to_grid(position_);
		glm::ivec3 explore_cell;
		bool something_to_explore = occupancy_grid_->next_cell_to_explore(current_cell, explore_cell, explore_range_.min_, explore_range_.max_);


		if (something_to_explore) {
			auto move_to_position = occupancy_grid_->map_to_position(explore_cell);
			float max_distance = occupancy_grid_->get_grid_resolution_per_side() * 1.414 * occupancy_grid_->get_grid_cube_length();
			float normalizing_constant = std::pow(glm::length(move_to_position - position_) / max_distance, 2);
			explore_velocity = normalizing_constant * (move_to_position - position_) / 10.f;
		}
	} catch (OutOfGridBoundsException& ex) {
		// ignore
	}

	return explore_velocity;
	
}

void ExperimentalRobot::update(int timestamp) {

	std::chrono::milliseconds current_timestamp = std::chrono::duration_cast<std::chrono::milliseconds>(
		std::chrono::system_clock::now().time_since_epoch());

	if (last_updated_time_ < 0) {
		last_updated_time_ = current_timestamp.count();
		return;
	}

	const float delta_time = 0.01; // in ms

	accumulator_ += current_timestamp.count() - last_updated_time_;

	//while (accumulator_ >= delta_time) {
		// do stuff


	glm::vec3 separation_velocity = calculate_separation_velocity();
	glm::vec3 alignment_velocity = calculate_alignment_velocity();
	glm::vec3 clustering_velocity = calculate_clustering_velocity();
	glm::vec3 explore_velocity = calculate_explore_velocity();

	//velocity_ += random_direction_;
	velocity_ += separation_velocity;
	velocity_ += alignment_velocity;
	velocity_ += clustering_velocity;
	velocity_ += bounce_off_corners_velocity();
	velocity_ += explore_velocity;

	velocity_.y = 0.f;

	// euler integration
	glm::vec3 old_position = position_;
	if (glm::length(velocity_) > max_velocity_) {
		auto normalized_velocity = glm::normalize(velocity_);
		velocity_ = max_velocity_ * normalized_velocity;
	}
	position_ += velocity_;

	//SwarmUtils::print_vector(std::to_string(id_) + " : ", clustering_velocity);

	// update grid data structure
	int explored = id_;
	try {
		glm::ivec3 visited_cell = occupancy_grid_->map_to_grid(position_);

		if (!occupancy_grid_->is_interior(visited_cell)) {
			if (render_) {
				explored_mutex_.lock();
				explored_cells_.push_back(visited_cell);
				explored_mutex_.unlock();
			}
			occupancy_grid_->set(visited_cell.x, visited_cell.z, explored);
			occupancy_grid_->mark_explored_in_perimeter_list(visited_cell);
			occupancy_grid_->mark_perimeter_covered_by_robot(occupancy_grid_->map_to_grid(visited_cell), timestamp, id_);
		}
	}
	catch (OutOfGridBoundsException& exception) {
		// ignore
	}

	previous_position_ = position_;
	accumulator_ -= delta_time;
	//}
}

