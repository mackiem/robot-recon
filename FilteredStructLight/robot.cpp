#include "robot.h"
#include <chrono>
#include <glm/gtc/matrix_transform.hpp> // glm::translate, glm::rotate, glm::scale, glm::perspective
#include <cmath>
#include <queue>
#include "swarmutils.h"

int Robot::MAX_DEPTH = 10;

#define PI 3.14159


unsigned Robot::get_id() {
	return id_;
}

glm::vec3 Robot::get_position() {
	return position_;
}

glm::vec3 Robot::get_velocity() {
	return velocity_;
}

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

bool Range::within_range(float grid_distance) {
	return (min_ <= grid_distance && grid_distance < max_);
}

void Robot::change_state(RobotState state) {
	current_robot_state_ = state;
}

void Robot::change_state(SwarmingState state) {
	current_swarming_state_ = state;
}

void Robot::update_robots(const std::vector<Robot*>& robots) {
	robots_ = robots;
}

void Robot::update(glm::mat4 global_model) {
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

void Robot::update_adjacent_and_interior(const glm::vec3& previous_position, const glm::vec3& current_position) {
	if (current_position == previous_position) {
		return;
	}

	adjacent_cells_.clear();
	adjacent_cells_.reserve(std::pow(sensor_range_ * 2, 2));
	get_adjacent_cells(occupancy_grid_->map_to_position(current_position), adjacent_cells_);
	interior_cells_ = get_interior_cell_positions(adjacent_cells_);
}

void Robot::calculate_work_force() {
	work_force_ = glm::vec3(0.f, 0.f, 0.f);
}

Robot::Robot(UniformLocations& locations, unsigned int id, SwarmOccupancyTree* octree, SwarmCollisionTree* collision_tree, 
	double explore_constant, double separation_constant, double alignment_constant, double cluster_constant, double perimeter_constant, double work_constant,
	Range explore_range, Range separation_range, Range alignment_range, Range cluster_range, Range perimeter_range, Range obstacle_avoidance_near_range, 
	Range obstacle_avoidance_far_range, 
	double sensor_range, int discovery_range, int neighborhood_count,
	double separation_distance, glm::vec3 position, QGLShaderProgram* shader, bool render, double magic_k, bool collide_with_robots) :
	
	VisObject(locations), all_goals_explored_(false),
	accumulator_(0.f), id_(id), position_(position), timeout_(5000), last_timeout_(0),
	last_updated_time_(-1), 
	explore_range_(explore_range), separation_range_(separation_range), alignment_range_(alignment_range),
	perimeter_range_(perimeter_range), 
	obstacle_avoidance_near_range_(obstacle_avoidance_near_range),
	obstacle_avoidance_far_range_(obstacle_avoidance_far_range),
	cluster_range_(cluster_range), sensor_range_(sensor_range), discovery_range_(discovery_range), 
	explore_constant_(explore_constant), separation_constant_(separation_constant), alignment_constant_(alignment_constant),
	work_constant_(work_constant), perimeter_constant_(perimeter_constant),
	cluster_constant_(cluster_constant), neighborhood_count_(neighborhood_count),
	separation_distance_(separation_distance), occupancy_grid_(octree),  
	collision_grid_(collision_tree), shader_(shader), render_(render), collide_with_robots_(collide_with_robots) {

	position_.y = 10.f;

	collision_grid_->insert(id_, occupancy_grid_->map_to_grid(position_));
	previous_position_ = position_;

	robot_radius_ = 11.f;
	mass_ = 10;
	max_velocity_ = 50;

	current_neighborhood_count_ = 0;
	
	magic_k_ = occupancy_grid_->get_grid_cube_length() * magic_k;

	tick_tock_age_ = 0;

	//sensor_range_ = 5;

	//separation_range_ = Range(0, 1);
	//alignment_range_ = Range(1, 3);
	//cluster_range_ = Range(3, 5);

	//perimeter_range_ = Range(1, 4);
	//explore_range_ = Range(4, 10); // extends to infininty

	attraction_distance_threshold_ = separation_distance_ + 10;

	distance_to_goal_threshold_ = 5.f;

	travelling_in_bound_ = false;

	cv::Vec4f green(0.f, 1.f, 0.f, 1.f);
	cv::Vec4f red(1.f, 0.f, 0.f, 1.f);
	cv::Vec4f yellow(1.f, 1.f, 0.f, 1.f);
	cv::Vec4f blue(0.f, 0.f, 1.f, 1.f);
	cv::Vec4f black(0.f, 0.f, 0.f, 1.f);
	cv::Vec4f orange = (cv::Vec4f(255.f, 131.f, 0.f, 255.f)) / 255.f;
	cv::Vec4f cyan = cv::Vec4f(0.f, 255.f, 204.f, 255.f) / 255.f;


	if (render_) {
		init_force_visualization(0, explore_force_, blue);
		init_force_visualization(1, separation_force_, green);
		init_force_visualization(2, resultant_force_, black);
		init_force_visualization(3, perimeter_force_, yellow);
		init_force_visualization(4, cluster_force_, cyan);
		init_force_visualization(5, alignment_force_, orange);
	}

	init_coverage_map();


	// create pool
	heap_pool_ = new std::vector<std::vector<glm::ivec3>>();
	pool_size_ = 10000;
	current_pool_count_ = 10000;
	heap_pool_->resize(pool_size_);


	float separation_force_max = explore_constant * explore_range.max_ +
		perimeter_constant * perimeter_range.max_;
	float k_separation_constant = separation_force_max / separation_range_.max_;
	//std::cout << "separation constnat " << k_separation_constant << std::endl;

	// naive theoretical baseline
	auto no_of_perimeter_cells = occupancy_grid_->no_of_unexplored_cells();
	auto total_distance_of_perimeter_cells = no_of_perimeter_cells * occupancy_grid_->get_grid_cube_length();
	auto half_diagonal_length = 0.5 * 1.414 * occupancy_grid_->get_grid_resolution_per_side() * occupancy_grid_->get_grid_cube_length();
	
	std::vector<int> theroetical_no_of_robots;
	theroetical_no_of_robots.push_back(1);
	theroetical_no_of_robots.push_back(4);
	theroetical_no_of_robots.push_back(9);
	theroetical_no_of_robots.push_back(49);
	theroetical_no_of_robots.push_back(100);

	for (auto& no_of_robots : theroetical_no_of_robots) {
		auto no_of_time_steps = total_distance_of_perimeter_cells / magic_k_ / no_of_robots;
		//no_of_time_steps += half_diagonal_length / magic_k_;
		//std::cout << "theoretical baseline - no of robots " << no_of_robots << " no of steps : " << no_of_time_steps << std::endl;
	}
}

Robot::Robot(UniformLocations& locations, unsigned id, SwarmOccupancyTree* octree,
	SwarmCollisionTree* collision_tree, double separation_constant, double alignment_constant,
	double cluster_constant, double explore_constant, double sensor_range,
	int discovery_range, double separation_distance, glm::vec3 position): VisObject(locations),  id_(id), position_(position),  
	sensor_range_(sensor_range), discovery_range_(discovery_range), 
	explore_constant_(explore_constant), separation_constant_(separation_constant), alignment_constant_(alignment_constant),
	cluster_constant_(cluster_constant), separation_distance_(separation_distance), occupancy_grid_(octree),  
	collision_grid_(collision_tree), collide_with_robots_(false) {

	heap_pool_ = new std::vector<std::vector<glm::ivec3>>();
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
		separation_distance_ = other.separation_distance_;

		// exploration force requirements
		// location in grid for frontier calculation
		mass_ = other.mass_;
		max_velocity_ = other.max_velocity_;
	}
	return *this;
}

Robot::~Robot() {
	//heap_pool_->clear();
	delete heap_pool_;
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
	for (int i = 0; i < 5; ++i) {
		update_force_visualization(i, zero_force);
	}
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
	//glm::vec3 direction;
	//if (glm::length(move_to_vector) > 1e-6) {
	//	 direction = glm::normalize(move_to_vector);
	//}
	glm::vec3 constant_times_direction = constant * move_to_vector;
	return constant_times_direction;
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
		if (other_robot_id != id_) {
			auto separation_vector = position_ - robots_[other_robot_id]->position_;
			auto separation_length = glm::length(separation_vector);
			if (separation_length > 1e-6) {
				float length = separation_length / static_cast<float>(occupancy_grid_->get_grid_cube_length());
				if (separation_range_.within_range(length)) {
					float inverse_length = (separation_range_.max_ - length) * static_cast<float>(occupancy_grid_->get_grid_cube_length());
					auto inverse_separation_vector = inverse_length * glm::normalize(separation_vector);
					separation_force_ += calculate_direction(position_ + inverse_separation_vector, separation_constant_);
				}
			}
		}
	}

	//for (auto& interior_cell_position : interior_cells) {
	//	auto separation_vector = position_ - interior_cell_position;
	//	float length = glm::length(separation_vector) / static_cast<float>(occupancy_grid_->get_grid_cube_length());
	//	if (separation_range_.within_range(length)) {
	//		separation_force_ += calculate_direction(position_ + separation_vector, separation_constant_);
	//	}
	//}

	separation_force_.y = 0.f;
}

void Robot::calculate_alignment_force(const std::vector<int>& other_robots) {
	alignment_force_ = glm::vec3(0.f, 0.f, 0.f);

	if (other_robots.size() > 0) {
		float delta_time = 1;
		//std::vector<glm::vec3> velocities;

		int alignment_count = 0;

		glm::vec3 avg_velocities;
		glm::vec3 first_velocity;
		for (auto& other_robot_id : other_robots) {
			if (other_robot_id != id_) {
				auto robot_distance = 
					glm::length(glm::vec3(occupancy_grid_->map_to_grid(position_) - occupancy_grid_->map_to_grid(robots_[other_robot_id]->position_)));
				if (alignment_range_.within_range(robot_distance)) {
					//break;
					//velocities.push_back(robots_[other_robot_id]->resultant_force_);
					alignment_count++;
					avg_velocities += robots_[other_robot_id]->resultant_force_;
				}
			}
		}

		if (alignment_count < 1) {
			return;
		}

		glm::vec3 picked_velocity = avg_velocities / static_cast<float>(alignment_count);

		if (glm::length(picked_velocity) > 1e-5) {

			//std::sort(velocities.begin(), velocities.end(), Vec3Comparator());
			//glm::vec3 median_velocity = velocities[velocities.size() / 2];
			//glm::vec3 avg_velocity = avg_velocities / static_cast<float>(velocities.size());
			//explore_force_ = glm::vec3(0.f);
			//perimeter_force_ = glm::vec3(0.f);


			auto move_to_position = position_ + picked_velocity * delta_time;
			//auto move_to_position = position_ + median_velocity * delta_time;
			move_to_position.y = 10.f;
			alignment_force_ = calculate_direction(move_to_position, alignment_constant_);
		}
		//alignment_force_.y = 0.f;
	}	
	
}


void Robot::calculate_cluster_force(const std::vector<int>& other_robots) {

	cluster_force_ = glm::vec3(0.f, 0.f, 0.f);

	try {
		if (other_robots.size() > 0) {
			//// move to centroid within cluster range
			//std::vector<PerimeterPos> other_robot_distance_vector;
			//std::vector<int> robot_ids;
			//std::vector<float> neighborhood_counts;
			//for (auto itr = other_robots.begin(); itr != other_robots.end(); ++itr) {
			//	auto other_robot = *itr;
			//	float grid_length = glm::length(glm::vec3(occupancy_grid_->map_to_grid(robots_[other_robot]->position_) -
			//		occupancy_grid_->map_to_grid(position_)));
			//	if (cluster_range_.within_range(grid_length) && id_ != other_robot) {
			//		other_robot_distance_vector.push_back(PerimeterPos(grid_length, occupancy_grid_->map_to_grid(robots_[other_robot]->position_)));
			//		robot_ids.push_back(other_robot);
			//		neighborhood_counts.push_back(robots_[other_robot]->current_neighborhood_count_);
			//	}
			//}


			//// simply, all the robots within range
			//current_neighborhood_count_ = other_robot_distance_vector.size();

			//// if preferred count is lower, then move towards any neighbor

			//if (current_neighborhood_count_ < (neighborhood_count_)) {
			//	glm::vec3 centroid;
			//	int cluster_size = 0;
			//	for (int i = 0; i < other_robot_distance_vector.size(); ++i) {
			//		centroid += robots_[robot_ids[i]]->position_;
			//		cluster_size++;
			//	}

			//	if (cluster_size > 0) {
			//		centroid /= cluster_size;
			//		glm::vec3 move_towards_position = centroid;
			//		cluster_force_ = calculate_direction(move_towards_position, cluster_constant_);
			//		cluster_force_.y = 0.f;
			//	}
			//}

			// move to centroid only if within cluster range
			if (other_robots.size() > 0) {
				// simply, all the robots within range
				current_neighborhood_count_ = other_robots.size();

				// if preferred count is lower, then move towards any neighbor

				//if (current_neighborhood_count_ < (neighborhood_count_)) {
					glm::vec3 centroid = position_;
					for (auto& other_robot_id : other_robots) {
						centroid += robots_[other_robot_id]->position_;
					}
					centroid /= (other_robots.size() + 1);

					float grid_distance = glm::length(glm::vec3(occupancy_grid_->map_to_grid(centroid) -
						occupancy_grid_->map_to_grid(position_)));
					if (cluster_range_.within_range(grid_distance)) {
						glm::vec3 move_towards_position = centroid;
						cluster_force_ = calculate_direction(move_towards_position, cluster_constant_);
						cluster_force_.y = 0.f;
					}
				//}
			}

		}
	} catch (OutOfGridBoundsException& ex) {
		
	}
}


void Robot::calculate_perimeter_force(const std::vector<glm::vec3>& interior_cells) {

	// for perimeter, get the closest two
	perimeter_force_ = glm::vec3(0.f, 0.f, 0.f);

	// guide the robot towards empty space
	try {
		auto current_cell = occupancy_grid_->map_to_grid(position_);
		glm::ivec3 perimeter_cell;
		bool something_to_perimeter = occupancy_grid_->closest_perimeter(current_cell, perimeter_cell, perimeter_range_.min_, perimeter_range_.max_);

		if (something_to_perimeter) {
			auto move_to_position = occupancy_grid_->map_to_position(perimeter_cell);
			// force to move to that cell (in 2D)
			move_to_position.y = 10.f;

			perimeter_force_ = calculate_direction(move_to_position, perimeter_constant_);
			
		} else {
			perimeter_force_ = glm::vec3(0.f);
		}

	} catch (OutOfGridBoundsException& ex) {
		//std::cout << "Position is out of bounds : [" << position_.x << ", " << position_.y << ", " << position_.z << "]" << std::endl;
		perimeter_force_ = glm::vec3(0.f);
	}

	//auto static_perimeter_list = occupancy_grid_->get_static_perimeter_list();

	//std::set<PerimeterPos> perimeter_vector;

	//for (auto& perimeter_grid_position : static_perimeter_list) {
	//	auto robot_grid_position = occupancy_grid_->map_to_grid(position_);
	//	float distance = glm::length(glm::vec3(perimeter_grid_position - robot_grid_position));
	//	if (perimeter_range_.within_range(distance)) {
	//		perimeter_vector.insert(PerimeterPos(distance, perimeter_grid_position));
	//	}
	//}

	//if (perimeter_vector.size() > 0) {

	//	//std::stable_sort(perimeter_vector.begin(), perimeter_vector.end());

	//	int count = 0;
	//	std::vector<glm::ivec3> closest_perimeter_positions;
	//	for (auto itr = perimeter_vector.begin(); itr != perimeter_vector.end(); ++itr) {
	//		if (count == 1) {
	//			break;
	//		}
	//		closest_perimeter_positions.push_back(itr->grid_position_);
	//		count++;
	//	}

	//	glm::vec3 walk_along_direction;
	//	glm::vec3 perimeter_position = occupancy_grid_->map_to_position(closest_perimeter_positions[0]);
	//	perimeter_position.y = 10.f;
	//	walk_along_direction = perimeter_position - position_;

	//	perimeter_force_ = calculate_direction(position_ + walk_along_direction, perimeter_constant_);
	//}


}

void Robot::calculate_explore_force() {
	// guide the robot towards empty space
	try {
		auto current_cell = occupancy_grid_->map_to_grid(position_);
		glm::ivec3 explore_cell;
		bool something_to_explore = occupancy_grid_->next_cell_to_explore(current_cell, explore_cell, explore_range_.min_, explore_range_.max_);

		if (something_to_explore) {
			auto move_to_position = occupancy_grid_->map_to_position(explore_cell);
			// force to move to that cell (in 2D)
			move_to_position.y = 10.f;

			explore_force_ = calculate_direction(move_to_position, explore_constant_);
			
			//if (glm::length(perimeter_force_) > 1e-6
			//	&& glm::length(explore_force_) > 1e-6) {
			//	float dot_value = glm::dot(glm::normalize(perimeter_force_), glm::normalize(explore_force_));
			//	float perimeter_constant = std::pow((dot_value + 1) /  2.f, 0.5f);

			//	perimeter_force_ = perimeter_constant * perimeter_force_;
			//}
		} else {
			explore_force_ = glm::vec3(0.f);
		}

	} catch (OutOfGridBoundsException& ex) {
		//std::cout << "Position is out of bounds : [" << position_.x << ", " << position_.y << ", " << position_.z << "]" << std::endl;
		explore_force_ = glm::vec3(0.f);
	}

}

glm::vec3 Robot::calculate_obstacle_avoidance_direction(glm::vec3 resultant_force) {

	//auto interior_list = occupancy_grid_->get_interior_list();

	//std::vector<PerimeterPos> interior_vector;

	//for (auto& interior_grid_position : interior_list) {
	//	auto robot_grid_position = occupancy_grid_->map_to_grid(position_);
	//	float distance = glm::length(glm::vec3(interior_grid_position - robot_grid_position));
	//	//if (cluster_range_.within_range(distance)) {
	//		interior_vector.push_back(PerimeterPos(distance, interior_grid_position));
	//	//}
	//}

	//if (interior_vector.size() > 0) {

	//	std::stable_sort(interior_vector.begin(), interior_vector.end());

	//	int count = 0;
	//	std::vector<glm::ivec3> closest_interior_positions;
	//	for (auto itr = interior_vector.begin(); itr != interior_vector.end(); ++itr) {
	//		if (count == 2) {
	//			break;
	//		}
	//		if (obstacle_avoidance_near_range_.within_range((*itr).distance_)) {
	//			closest_interior_positions.push_back(itr->grid_position_);
	//			count++;
	//		}
	//	}


	try {
		reallocate_pools();
		std::vector<glm::ivec3> closest_interior_positions = (*heap_pool_)[current_pool_count_++];
		//std::vector<glm::ivec3> closest_interior_positions;
		//closest_interior_positions.reserve(2);

		glm::vec3 final_direction;

		auto current_cell = occupancy_grid_->map_to_grid(position_);

		bool close_interiors_found = occupancy_grid_->closest_2_interior_positions(current_cell, closest_interior_positions, 
			obstacle_avoidance_near_range_.min_, obstacle_avoidance_far_range_.max_);

		if (close_interiors_found) {
			float close_interior_distance = glm::length(glm::vec3(closest_interior_positions[0] - current_cell));
			if (close_interior_distance < obstacle_avoidance_near_range_.max_
				|| closest_interior_positions.size() == 1) {
				// just go there
				glm::vec3 closest_perimeter_position = occupancy_grid_->map_to_position(closest_interior_positions[0]);
				closest_perimeter_position.y = 10.f;
				glm::vec3 position_to_perimeter = closest_perimeter_position - position_;

				if (glm::dot(resultant_force, position_to_perimeter) < 0) {
					return resultant_force;
				}

				// away from the perimeter
				final_direction = position_ - closest_perimeter_position;

			} else  if (closest_interior_positions.size() > 1) {
				// figure out how to walk along
				glm::vec3 interior_direction = occupancy_grid_->map_to_position(closest_interior_positions[1]) -
					occupancy_grid_->map_to_position(closest_interior_positions[0]);
				interior_direction.y = 10.f;

				glm::vec3 closest_perimeter_position = occupancy_grid_->map_to_position(closest_interior_positions[0]);
				glm::vec3 position_to_perimeter = closest_perimeter_position - position_;

				if (glm::dot(resultant_force, position_to_perimeter) < 0) {
					return resultant_force;
				}

				// project perimeter with current resultant direction, proj (resultant direction) on to (perimeter) = proj_(p)r
				if (std::abs(glm::dot(interior_direction, resultant_force)) > 1e-6) {
					final_direction =
						interior_direction * glm::dot(interior_direction, resultant_force) / glm::dot(interior_direction, interior_direction);
				}
				else {
					// it's orthogonal, so just change direction to wall direction
					final_direction = interior_direction;
				}
				final_direction.y = 0.f;
			}
			if (glm::length(final_direction) > 1e-6) {
				//final_direction = glm::normalize(final_direction);
			}
			return final_direction;
			
			//if (glm::length(perimeter_force_) > 1e-6
			//	&& glm::length(final_direction_force_) > 1e-6) {
			//	float dot_value = glm::dot(glm::normalize(perimeter_force_), glm::normalize(final_direction_force_));
			//	float perimeter_constant = std::pow((dot_value + 1) /  2.f, 0.5f);

			//	perimeter_force_ = perimeter_constant * perimeter_force_;
			//}
		} else {
			return resultant_force;
		}

	} catch (OutOfGridBoundsException& ex) {
			return resultant_force;
	}

		//if (closest_interior_positions.size() > 0) {
		//	// check if resultant direction and perimeter are going in the same direction 


		//return final_direction;
		//}
		//else {
		//	return resultant_force;
		//}

	
}

glm::vec3 Robot::calculate_resultant_direction(const std::vector<int>& other_robots, const std::vector<glm::vec3>& interior_cells) {

	// calculate important forces
	calculate_separation_force(other_robots, interior_cells);
	calculate_cluster_force(other_robots);
	calculate_perimeter_force(interior_cells);
	calculate_explore_force();
	calculate_alignment_force(other_robots);

	// add forces together to get direction of motion
	glm::vec3 actual_force = separation_force_ + alignment_force_ + cluster_force_ + explore_force_ + perimeter_force_;

#ifdef DEBUG
	//if (actual_force.y > 0.f) {
		std::cout << "Robot : " << id_ << std::endl;
		SwarmUtils::print_vector("explore", explore_force_);
		SwarmUtils::print_vector("separation", separation_force_);
		SwarmUtils::print_vector("perimeter", perimeter_force_);
		SwarmUtils::print_vector("alignment", alignment_force_);
		SwarmUtils::print_vector("cluster", cluster_force_);
		std::cout << std::endl;
	//}
#endif
	glm::vec3 force_direction;
	//if (glm::length(actual_force) > 1e-6) {
	//	 force_direction = glm::normalize(actual_force);
	//}
	force_direction = actual_force;
	return force_direction;

}

void Robot::init_coverage_map() {
	occupancy_grid_->init_coverage_map(coverage_map_);
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

void Robot::update_visualization_structs() {
		// update visualization
		if (show_forces_) {
			update_force_visualization(0, explore_force_);
			update_force_visualization(1, separation_force_);
			update_force_visualization(3, perimeter_force_);
			update_force_visualization(4, cluster_force_);
			update_force_visualization(5, alignment_force_);
			update_force_visualization(2, 100.f * resultant_force_);
		}
		// update rendered mesh
		for (auto& render_entity : mesh_) {
			glm::mat4 translate_model = glm::translate(glm::mat4(1.f), position_);
			render_entity.set_model(translate_model * render_entity.get_initial_model());
		}
	
		explored_mutex_.lock();
		for (auto& adjacent_sensor_cell : explored_cells_) {
			overlay_->update_grid_position(adjacent_sensor_cell);
		}
		explored_cells_.clear();
		explored_mutex_.unlock();
}

void Robot::calculate_sampling_factor() {
	occupancy_grid_->calculate_multi_sampling_factor();
}

double Robot::calculate_coverage() {
	double avg_covered_cells = 0;
	for (auto& cell : coverage_map_) {
		if (cell.second > 0) {
			avg_covered_cells += 1;
		}
	}
	if (coverage_map_.size() > 0) {
		avg_covered_cells /= coverage_map_.size();
	}
	return avg_covered_cells;
}

void Robot::reallocate_pools() {
	// create a pool, to stop micro memory allocations all the time
	if (current_pool_count_ == pool_size_) {
		current_pool_count_ = 0;
		std::vector<glm::ivec3> sample_vector;
		sample_vector.reserve(std::pow(2 * (sensor_range_ + 1), 2));
		heap_pool_->clear();
		heap_pool_->resize(pool_size_);
		std::fill(heap_pool_->begin(), heap_pool_->end(), sample_vector);
	}
	
}

void Robot::update(int timestamp) {

	// algorithm:
	// if we have adjacent perimeter, then we go along it
	// if not we have the explore force, that guides us towards perimeter
	// the repulsion force stays the same just in case we get too close
	// basically, a concentric circle model for perimeter exploration, and thereby, 3D reconstruction
	// making use of the boid model (sort of) and adapting it to  repulsion, explore and walk along

	std::chrono::milliseconds current_timestamp = std::chrono::duration_cast<std::chrono::milliseconds>(
		std::chrono::system_clock::now().time_since_epoch());

	if (last_updated_time_ <= 0) {
		last_updated_time_ = current_timestamp.count();
	} else {

		auto delta_time = (current_timestamp.count() - last_updated_time_) / 1000.f;

		//float time_step_duration = 1.f / 30.f; // 60 hz 

		//accumulator_ += delta_time;

		// get next direction for motion
		reallocate_pools();
		std::vector<glm::ivec3> adjacent_cells = (*heap_pool_)[current_pool_count_++];
		//adjacent_cells.reserve(9);
		get_adjacent_cells(position_, adjacent_cells);
		auto robot_ids = get_other_robots(adjacent_cells);
		auto interior_cells = get_interior_cell_positions(adjacent_cells);

		// add hysteresis
		// monitor last 5 directions
		glm::vec3 last_direction = resultant_force_;

		if (tick_tock_age_ > 0) {
			resultant_force_ = last_direction;
			tick_tock_age_--;
		}
		else {
			glm::vec3 new_resultant_force = calculate_resultant_direction(robot_ids, interior_cells);
			new_resultant_force = calculate_obstacle_avoidance_direction(new_resultant_force);
			resultant_force_ = new_resultant_force;

			const int window_size = 3;
			if (last_resultant_directions_.size() == window_size) {
				int counter = 0;
				bool tick_tock_detected = false;
				int previous_result = 0;

				glm::vec3 test_previous_direction = new_resultant_force;

				for (auto& previous_direction : last_resultant_directions_) {
					float result = glm::dot(previous_direction, test_previous_direction);
					tick_tock_detected = (result) < 0;
					if (!tick_tock_detected) break;
					//previous_result = result;
					test_previous_direction = previous_direction;
					counter++;
				}
				if (tick_tock_detected) {
					resultant_force_ = last_direction;
					tick_tock_age_ = 10;
				}

				last_resultant_directions_.pop_front();
				last_resultant_directions_.push_back(new_resultant_force);
			}
			else {
				last_resultant_directions_.push_back(new_resultant_force);
			}
		}
		//std::stringstream ss;
		//ss << "Robot " << id_ << ": ";

		//SwarmUtils::print_vector(ss.str(), resultant_force_);


		//// if in opposite directions
		//if ((glm::length(resultant_force_) < 1e-5) || glm::dot(last_direction, new_resultant_direction) > 0) {
		//	resultant_force_ = new_resultant_direction;
		//} else {
		//	resultant_force_ = last_direction;
		//}

		// calculate new position
		//float small_dist = magic_k_;
		//glm::vec3 position_delta = resultant_force_ * small_dist;



		// save old position for rollback if colliding
		//glm::vec3 old_position = position_;
		//position_ += position_delta;


		// euler
		glm::vec3 old_position = position_;
		velocity_ += resultant_force_ * delta_time;
		if (glm::length(velocity_) > max_velocity_) {
			auto normalized_velocity = glm::normalize(velocity_);
			velocity_ = max_velocity_ * normalized_velocity;
		}
		position_ += velocity_ * delta_time;


		// check for collision

		bool is_colliding = false;
		if (collide_with_robots_) {
			is_colliding = is_colliding_with_robots(robot_ids);
		}
		is_colliding |= is_colliding_with_interior(interior_cells);

		if (is_colliding) {
			position_ = old_position;
		} 

		for (auto& interior_cell : interior_cells) {
			occupancy_grid_->mark_perimeter_covered_by_robot(occupancy_grid_->map_to_grid(interior_cell), timestamp, id_);
		}

		// update grid data structure
   		int explored = id_;
		try {
			glm::ivec3 grid_position = occupancy_grid_->map_to_grid(position_);
			glm::ivec3 previous_grid_position = occupancy_grid_->map_to_grid(previous_position_);
			collision_grid_->update(id_, previous_grid_position, grid_position);
	
			// update the whole range of the sensor
			std::vector<glm::ivec3> sensor_cells;
			//int sensor_range = (sensor_range_ - 2) <= 0 ? 1 : sensor_range_ - 2;
			occupancy_grid_->get_adjacent_cells(grid_position, sensor_cells, discovery_range_);

			for (auto adjacent_sensor_cell : sensor_cells) {
				if (!occupancy_grid_->is_interior(adjacent_sensor_cell)) {
					if (render_) {
						explored_mutex_.lock();
						explored_cells_.push_back(adjacent_sensor_cell);
						explored_mutex_.unlock();
					}
					occupancy_grid_->set(adjacent_sensor_cell.x, adjacent_sensor_cell.z, explored);
					occupancy_grid_->mark_explored_in_perimeter_list(adjacent_sensor_cell);
					//occupancy_grid_->mark_explored_in_empty_space_list(adjacent_sensor_cell);
					coverage_map_[adjacent_sensor_cell] = 1;
				} else {
					// is interior
					// mark interiors as samples
				}

			}

		} catch (OutOfGridBoundsException& exception) {
			// ignore
		}

		previous_position_ = position_;
	}

	last_updated_time_ = current_timestamp.count();
}



