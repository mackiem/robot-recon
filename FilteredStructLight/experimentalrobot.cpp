#include "experimentalrobot.h"
#include <chrono>
#include "swarmutils.h"

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

ExperimentalRobot::ExperimentalRobot(UniformLocations& locations, unsigned id, SwarmOccupancyTree* octree,
	SwarmCollisionTree* collision_tree, Swarm3DReconTree* recon_tree, int cluster_id, double separation_constant, double alignment_constant,
	double cluster_constant, double explore_constant, double sensor_range,
	int discovery_range, double separation_distance, glm::vec3 position,
	double square_radius, double bounce_function_power, double bounce_function_multiplier, int max_time, 
	bool collide_with_robots, bool render, QGLShaderProgram* shader)

	: Robot(locations, id, octree, collision_tree,  separation_constant,
	alignment_constant, cluster_constant, explore_constant,sensor_range, discovery_range, separation_distance, position, render, shader),  
	square_radius_(square_radius), bounce_function_power_(bounce_function_power), 
	bounce_function_multiplier_(bounce_function_multiplier), recon_tree_(recon_tree), max_time_(max_time)
{
	random_direction_ = glm::vec3(1.f, 0.f, 0.f);
	max_velocity_ = 4.f;
	robot_radius_ = 11.85f;
	previous_no_of_explored_cells_ = -1;
	explore_range_ = Range(0, occupancy_grid_->get_grid_resolution_per_side() * 1.414);
	death_time_ = -1;
	dead_ = false;
	dead_color_changed_ = false;
	cluster_id_ = cluster_id;
	populate_occlusion_map();
}

void ExperimentalRobot::populate_occlusion_map() {
	measurement_time_step_ = 100;
	int no_of_divisions = max_time_ / measurement_time_step_;

	for (int i = 0; i < no_of_divisions; ++i) {
		occlusions_per_timestamp_map_[i * measurement_time_step_] = 0;
	}
}


void ExperimentalRobot::update_visualization_structs() {
	Robot::update_visualization_structs();
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

ExperimentalRobot::~ExperimentalRobot()
{
}

glm::vec3 ExperimentalRobot::calculate_separation_velocity() {
	const float max_distance = separation_distance_;
	glm::vec3 c;
	for (auto& robot_id : robot_ids_) {
		if (robot_id != id_) {
			float distance_apart = glm::length(robots_[robot_id]->get_position() - position_);
			if (distance_apart < max_distance) {
				auto move_away_vector = (robots_[robot_id]->get_position() - position_);
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
	for (auto& robot_id : robot_ids_) {
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
	return (v - velocity_) * alignment_constant_;
}

glm::vec3 ExperimentalRobot::bounce_off_corners_velocity() {
	float x_max, y_max, z_max;
	float x_min, y_min, z_min;
	float outer_boundary = 3.5f;
	x_min = y_min = z_min = occupancy_grid_->get_grid_cube_length() * outer_boundary;
	x_max = y_max = z_max = occupancy_grid_->get_grid_cube_length() * (occupancy_grid_->get_grid_resolution_per_side() - outer_boundary);

	float x_wall_max, y_wall_max, z_wall_max;
	float x_wall_min, y_wall_min, z_wall_min;

	float inner_boundary = 1.5f;
	x_wall_min = y_wall_min = z_wall_min = occupancy_grid_->get_grid_cube_length() * inner_boundary;
	x_wall_max = y_wall_max = z_wall_max = occupancy_grid_->get_grid_cube_length() * (occupancy_grid_->get_grid_resolution_per_side() - inner_boundary);
	
	const float opposite_vel = max_velocity_;
	const float power = 2.f;
	glm::vec3 v;

	float bounce_off_normalizing_constant = 0.001f;

	if (position_.x > x_max
		&& position_.z > z_max) {
			float normalizing_constant = 1.f / std::pow(position_.x - x_wall_max, power);
			v.x = bounce_off_normalizing_constant * -opposite_vel;
			v.z = normalizing_constant * -opposite_vel;

	} else if (position_.x < x_min
		&& position_.z < z_min) {
			float normalizing_constant = 1.f / std::pow(position_.z - z_wall_min, power);
			v.z = bounce_off_normalizing_constant * opposite_vel;
			v.x = normalizing_constant * opposite_vel;
		
	}
	else {


		if (position_.x < x_min) {
			float normalizing_constant = 1.f / std::pow(position_.x - x_wall_min, power);
			v.x = bounce_off_normalizing_constant * opposite_vel;
			// send in the normal direction
			v.z = normalizing_constant * opposite_vel;
		}
		else if (position_.x > x_max) {
			float normalizing_constant = 1.f / std::pow(position_.x - x_wall_max, power);
			// send in the normal direction
			v.x = bounce_off_normalizing_constant * -opposite_vel;
			v.z = normalizing_constant * -opposite_vel;
		}

		//if (position_.y < y_min) {
		//	v.y = opposite_vel;
		//} else if (position_.y > y_may) {
		//	v.y = -opposite_vel;
		//}

		if (position_.z < z_min) {
			float normalizing_constant = 1.f / std::pow(position_.z - z_wall_min, power);
			v.z = bounce_off_normalizing_constant * opposite_vel;
			v.x = normalizing_constant * -opposite_vel;
		}
		else if (position_.z > z_max) {
			float normalizing_constant = 1.f / std::pow(position_.z - z_wall_max, power);
			v.z = bounce_off_normalizing_constant * -opposite_vel;
			v.x = normalizing_constant * opposite_vel;
		}
	}
	return v;
}

glm::vec3 ExperimentalRobot::calculate_clustering_velocity() {
	glm::vec3 pc;
	int count = 0;

	for (auto& robot_id : robot_ids_) {
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

	float normalizing_constant = std::pow(glm::length(pc - position_) / (sensor_range_ * 1.414 * occupancy_grid_->get_grid_cube_length()), 2);
	//glm::vec3 cluster_velocity = normalizing_constant * (pc - position_) / 10.f;
	glm::vec3 cluster_velocity = normalizing_constant * (pc - position_) * cluster_constant_;
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

glm::vec3 ExperimentalRobot::calculate_explore_velocity() {
	glm::vec3 explore_velocity;

	auto explored_cells = occupancy_grid_->no_of_unexplored_cells();



	try {
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
			float max_distance = occupancy_grid_->get_grid_resolution_per_side() * 1.414 * occupancy_grid_->get_grid_cube_length();
			float normalizing_constant = std::pow(glm::length(move_to_position - position_) / max_distance, 2);
			//explore_velocity = normalizing_constant * (move_to_position - position_) / 2.5f;
			explore_velocity = normalizing_constant * (move_to_position - position_) * explore_constant_;

			//if (id_ == 0) {
			//	SwarmUtils::print_vector("explore", explore_cell);
			//}
		}
	} catch (OutOfGridBoundsException& ex) {
		// ignore
	}
	
	previous_no_of_explored_cells_ = explored_cells;


	for (auto& interior_center : interior_cells_) {

		float x_max, y_max, z_max;
		float x_min, y_min, z_min;

		float outer_boundary = 2.0f;

		x_min = interior_center.x - (occupancy_grid_->get_grid_cube_length() * outer_boundary);
		z_min = interior_center.z - (occupancy_grid_->get_grid_cube_length() * outer_boundary);

		x_max = interior_center.x + (occupancy_grid_->get_grid_cube_length() * outer_boundary);
		z_max = interior_center.z + (occupancy_grid_->get_grid_cube_length() * outer_boundary);
			
		float x_wall_max, y_wall_max, z_wall_max;
		float x_wall_min, y_wall_min, z_wall_min;

		float inner_boundary = 1.0f;
		x_wall_min = interior_center.x - (occupancy_grid_->get_grid_cube_length() * inner_boundary);
		z_wall_min = interior_center.z - (occupancy_grid_->get_grid_cube_length() * inner_boundary);

		x_wall_max = interior_center.x + (occupancy_grid_->get_grid_cube_length() * inner_boundary);
		z_wall_max = interior_center.z + (occupancy_grid_->get_grid_cube_length() * inner_boundary);

		glm::vec3 v;
		//float power = 3.0f;

		float normalizingConstant = (outer_boundary - inner_boundary) * occupancy_grid_->get_grid_cube_length();
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

		explore_velocity += v * static_cast<float>(bounce_function_multiplier_);

	}

	return explore_velocity;

	//float x_max, y_max, z_max;
	//float x_min, y_min, z_min;
	//float outer_boundary = 2.5f;
	//x_min = y_min = z_min = occupancy_grid_->get_grid_cube_length() * outer_boundary;
	//x_max = y_max = z_max = occupancy_grid_->get_grid_cube_length() * (occupancy_grid_->get_grid_resolution_per_side() - outer_boundary);

	//float x_wall_max, y_wall_max, z_wall_max;
	//float x_wall_min, y_wall_min, z_wall_min;

	//float inner_boundary = 1.5f;
	//x_wall_min = y_wall_min = z_wall_min = occupancy_grid_->get_grid_cube_length() * inner_boundary;
	//x_wall_max = y_wall_max = z_wall_max = occupancy_grid_->get_grid_cube_length() * (occupancy_grid_->get_grid_resolution_per_side() - inner_boundary);
	//

	//glm::vec3 v;
	////float power = 3.0f;

	//float normalizingConstant = (outer_boundary - inner_boundary) * occupancy_grid_->get_grid_cube_length();
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
	for (auto& adjacent : adjacent_cells_) {
		// this may slow things down, look into doing it when grid cell changes
		//auto points_3d = recon_tree_->get_3d_points(interior);
		//update_some_data_struct_to_render_points(points_3d);
		if (render_) {
			if (past_reconstructed_positions_.find(adjacent) == past_reconstructed_positions_.end()) {
				recon_mutex_.lock();
				reconstructed_positions_.push_back(adjacent);
				recon_mutex_.unlock();
				past_reconstructed_positions_.insert(adjacent);
			}
		}
		recon_tree_->update_multi_sampling_map(adjacent);
	}
}

std::vector<glm::vec3> ExperimentalRobot::get_corners(const glm::vec3& interior_cell) const {
	// 4 corners
	std::vector<glm::vec3> corners(4);
	float half_distance = occupancy_grid_->get_grid_cube_length() / 2.f;

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
		if (occlusion_per_timestamp.first <= final_timestamp_) {
			occlusion += occlusion_per_timestamp.second;
			entry_count++;
		}
	}
	if (entry_count > 0) {
		occlusion /= entry_count;
	}
	return occlusion;
	
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


void ExperimentalRobot::update(int timestamp) {

	//std::chrono::milliseconds current_timestamp = std::chrono::duration_cast<std::chrono::milliseconds>(
	//	std::chrono::system_clock::now().time_since_epoch());

	//if (last_updated_time_ < 0) {
	//	last_updated_time_ = current_timestamp.count();
	//	return;
	//}
	if (dead_) {
		return;
	}

	const float delta_time = 2500; // in ms

	if (death_time_ > 0 && timestamp > death_time_) {
		dead_ = true;
	}

	final_timestamp_ = timestamp;


	try {

		robot_ids_ = get_other_robots(adjacent_cells_);

		if (timestamp > 0 && ((timestamp % measurement_time_step_) == 0)) {
			occlusions_per_timestamp_map_[timestamp] = robot_ids_.size();
		}

		glm::vec3 separation_velocity = calculate_separation_velocity();
		glm::vec3 alignment_velocity = calculate_alignment_velocity();
		glm::vec3 clustering_velocity = calculate_clustering_velocity();
		glm::vec3 explore_velocity = calculate_explore_velocity();

		velocity_ += separation_velocity;
		velocity_ += alignment_velocity;
		velocity_ += clustering_velocity;
		velocity_ += explore_velocity;

		velocity_.y = 0.f;

		// euler integration
		glm::vec3 old_position = position_;
		if (glm::length(velocity_) > max_velocity_) {
			auto normalized_velocity = glm::normalize(velocity_);
			velocity_ = max_velocity_ * normalized_velocity;
		}

		position_ += velocity_ * delta_time / 1000.f / 10.f;

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

		//SwarmUtils::print_vector(std::to_string(id_) + " : ", bounce_velocity);
		//SwarmUtils::print_vector(std::to_string(id_) + " : ", position_);

		// update grid data structure
		int explored = id_;
		try {
			glm::ivec3 grid_position = occupancy_grid_->map_to_grid(position_);
			glm::ivec3 previous_grid_position = occupancy_grid_->map_to_grid(previous_position_);
			collision_grid_->update(id_, previous_grid_position, grid_position);
			update_adjacent_and_interior(previous_grid_position, grid_position);

			//reallocate_pools();
			//std::vector<glm::ivec3> sensored_cells = (*heap_pool_)[current_pool_count_++];
			//occupancy_grid_->get_adjacent_cells(grid_position, sensored_cells, discovery_range_);

			if (interior_updated_) {
				for (auto sensored_cell : adjacent_cells_) {
					float distance = glm::length(glm::vec3(grid_position - sensored_cell));

					if (distance <= discovery_range_) {

						if (!occupancy_grid_->is_interior(sensored_cell)) {
							if (render_) {
								explored_mutex_.lock();
								explored_cells_.push_back(sensored_cell);
								explored_mutex_.unlock();
							}
							occupancy_grid_->set(sensored_cell.x, sensored_cell.z, explored);
							occupancy_grid_->mark_explored_in_perimeter_list(sensored_cell);
						}
						else {
						}
					}
				}
				reconstruct_points();
			}
		}
		catch (OutOfGridBoundsException& exception) {
			// ignore
		}

		previous_nminus2_position_ = previous_position_;
		previous_position_ = position_;
		//	accumulator_ -= delta_time;
		//}
	} catch (OutOfGridBoundsException& ex) {
		std::cout << "Out of bounds - id : " << id_ << "\n";
	}
}

