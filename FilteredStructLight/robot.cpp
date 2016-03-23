#include "robot.h"
#include <chrono>
#include <glm/gtc/matrix_transform.hpp> // glm::translate, glm::rotate, glm::scale, glm::perspective
#include <cmath>
#include <queue>



void Robot::set_explore_constant(float constant) {
	explore_constant_ = constant;
}

void Robot::set_seperation_constant(float constant) {
	separation_constant_ = constant;
}

void Robot::set_work_constant(float constant) {
	work_constant_ = constant;
}

void Robot::change_state(RobotState state) {
	current_robot_state_ = state;
}

void Robot::change_state(SwarmingState state) {
	current_swarming_state_ = state;
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
	// use Hooke's law
	auto move_to_vector = (move_to_position - position_);
	glm::vec3 force = constant * move_to_vector;
	return force;
}

void Robot::calculate_explore_force() {
	// if there's a frontier (unexplore space around me) head towards that
	// naive algorithm - find the first

	try {
		auto current_cell = octree_->map_to_grid(position_);

		glm::ivec3 frontier_cell;
		bool something_to_explore = octree_->frontier_bread_first_search(current_cell, frontier_cell);
		if (something_to_explore) {
			auto move_to_position = octree_->map_to_position(frontier_cell);
			// only 2D
			move_to_position.y = 10.f;
			//explore_force_ = calculate_force(move_to_position);
			explore_force_ = calculate_force(move_to_position, explore_constant_);
			std::cout << "Frontier Cell : [" << frontier_cell.x << ", " << frontier_cell.y << ", " << frontier_cell.z << "]" << std::endl;
		}
		else {
			// we are done
			//goal_position_ = position_;
			explore_force_ = glm::vec3(0.f);
			std::cout << "Nothing to explore..." << std::endl;
		}
	}
	catch (OutOfGridBoundsException& ex) {
		//std::cout << "Position is out of bounds : [" << position_.x << ", " << position_.y << ", " << position_.z << "]" << std::endl;
		explore_force_ = glm::vec3(0.f);
	}
}

void Robot::calculate_separation_force() {

	separation_force_ = glm::vec3(0.f, 0.f, 0.f);
	for (auto& robot : robots_) {
		if (robot->id_ < id_) {
			// naively, just a set direction away from me
			auto seperation_vector = position_ - robot->position_;
			//std::cout << "Other robot : " << robot.id_ << " position : " << robot.position_.x << ", " << robot.position_.y << ", " << 
			//	robot.position_.z << std::endl;
			float distance = glm::length(seperation_vector);
			if (distance < separation_distance_threshold_) {
				// apply some force away from center of mass 

				// minimum seperation distance + some threshold??
				float distance_to_travel = (separation_distance_threshold_ - distance);
				glm::vec3 move_to_direction = glm::normalize(seperation_vector);
				//move_to_direction.y = 10.f;
				glm::vec3 move_to_position = position_ + distance_to_travel * move_to_direction;
				//std::cout << "distance to travel : " << distance_to_travel << std::endl;
			//std::cout << "robot : " << id_ << " move to direction : " << move_to_direction.x << ", " << move_to_direction.y << ", " << 
			//	move_to_direction.z << std::endl;

			//std::cout << "robot : " << id_ << " position : " << position_.x << ", " << position_.y << ", " << 
			//	position_.z << std::endl;

			//std::cout << "robot : " << id_ << " move to position : " << move_to_position.x << ", " << move_to_position.y << ", " << 
			//	move_to_position.z << std::endl;

				separation_force_ += calculate_force(move_to_position, separation_constant_);
			//	std::cout << "Robot : " << id_ << " separation distance : " << distance <<  
			//		" disance to travel : " <<  distance_to_travel << std::endl;
			//std::cout << "robot : " << id_ << " seperation force : " << separation_force_.x << ", " << separation_force_.y << ", " << 
			//	separation_force_.z << std::endl;
			}
		}
	}
	separation_force_.y = 0.f;
}

void Robot::calculate_work_force() {
	work_force_ = glm::vec3(0.f, 0.f, 0.f);

}

bool Robot::is_at_edge() {
	try {
		auto current_cell = octree_->map_to_grid(position_);
		if ((current_cell.x <= 0 || current_cell.x >= octree_->get_grid_resolution_per_side())
			|| (current_cell.z <= 0 || current_cell.z >= octree_->get_grid_resolution_per_side())) {
			return true;
		}
	return false;
	}
	catch (OutOfGridBoundsException& ex) {
		return true;
	}
}

void Robot::stop() {
	work_force_ = glm::vec3(0.f, 0.f, 0.f);
	explore_force_ = glm::vec3(0.f, 0.f, 0.f);
	separation_force_ = glm::vec3(0.f, 0.f, 0.f);
	resultant_force_ = glm::vec3(0.f, 0.f, 0.f);
	velocity_ = glm::vec3(0.f, 0.f, 0.f);
}

void Robot::nullify_forces() {
	explore_force_ = glm::vec3(0.f, 0.f, 0.f);
	separation_force_ = glm::vec3(0.f, 0.f, 0.f);
	stopping_force_ = glm::vec3(0.f, 0.f, 0.f);
	resultant_force_ = glm::vec3(0.f, 0.f, 0.f);

}

void Robot::calculate_out_of_bounds_force() {
	//float out_of_bounds_constant_ = 10 * explore_constant_;
	//try {
	//	if (current_swarming_state_ != GOING_IN_BOUND) {
	//		//change_state(GOING_IN_BOUND);
	//	}
	//} catch (OutOfGridBoundsException& ex) {
	//	if (current_swarming_state_ != GOING_IN_BOUND) {
	//		//out_of_bounds_force_ = -glm::normalize(velocity_) * 100.f;
	//		explore_force_ = glm::vec3(0.f, 0.f, 0.f);
	//		separation_force_ = glm::vec3(0.f, 0.f, 0.f);
	//		velocity_ = glm::vec3(0.f, 0.f, 0.f);
	//		change_state(EXPLORING);
	//	}
	//}
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


glm::vec3 Robot::calculate_resultant_force() {
	calculate_explore_force();
	calculate_separation_force();
	//calculate_work_force();
	//calculate_out_of_bounds_force();

	//glm::vec3 actual_force = explore_constant_ * explore_force_
	//	+ separation_force_ * separation_constant_+ work_constant_ * work_force_;

	glm::vec3 actual_force = explore_force_
		+ separation_force_ + work_force_;

	// we only want to get the force direction
	glm::vec3 force;
	if (glm::length(actual_force) > 1e-3) {
		glm::vec3 force_direction = glm::normalize(actual_force);
		force = force_direction * static_cast<float>(octree_->get_grid_cube_length());
		goal_position_ = position_ + force_direction * static_cast<float>(octree_->get_grid_cube_length());
	} else {
		goal_position_ = position_;
	}


	//glm::vec3 force =  explore_force_
	//	+ separation_force_ + work_force_ + stopping_force_;

	//if (force.y > 0.f) {
	std::cout << "Robot : " << id_ << std::endl;
	std::cout << "explore : [" << explore_force_.x << ", " << explore_force_.y << ", " << explore_force_.z << "]" << std::endl;
	std::cout << "seperation : [" << separation_force_.x << ", " << separation_force_.y << ", " << separation_force_.z << "]" << std::endl;
	std::cout << "out_of_bounds : [" << out_of_bounds_force_.x << ", " << out_of_bounds_force_.y << ", " << out_of_bounds_force_.z << "]" << std::endl;
	std::cout << std::endl;
	// }

	// ToDo : add some gaussian noise

	return force;
}

Robot::Robot(UniformLocations& locations, unsigned int id, std::shared_ptr<SwarmOctTree> octree, 
	double explore_constant, double seperation_constant, double work_constant,
	double seperation_distance, QGLShaderProgram* shader) : VisObject(locations), id_(id),
	timeout_(5000), last_timeout_(0), last_updated_time_(0), explore_constant_(explore_constant), separation_constant_(seperation_constant),
	work_constant_(work_constant), separation_distance_threshold_(seperation_distance), octree_(octree),
	shader_(shader) {

	std::random_device rd;
	rng_.seed(rd());
	velocity_generator_ = std::uniform_real_distribution<float>(-0.2, 0.2);
	position_generator_ = std::uniform_int_distribution<int>(0, octree->get_grid_cube_length() * octree_->get_grid_resolution_per_side());
	position_ = glm::vec3(position_generator_(rng_), 10.f, position_generator_(rng_));

	//set_random_velocity();

	mass_ = 10;
	max_velocity_ = 50;

	//explore_constant_ = work_constant_ = 1.f;
	//separation_constant_ = 10 * explore_constant_;
	
	//separation_distance_threshold_ = 50;
	//minimum_seperation_distance_ = 10;

	distance_to_goal_threshold_ = 5.f;

	travelling_in_bound_ = false;
	//change_state(EXPLORING);
	change_state(STOPPED);

	cv::Vec4f green(0.f, 1.f, 0.f, 1.f);
	cv::Vec4f red(0.f, 1.f, 0.f, 1.f);
	visualize_force(1, explore_force_, green, true);
	visualize_force(2, separation_force_, red, true);

}

void Robot::set_velocity(glm::vec3 velocity) {
	velocity_ = velocity;
}

void Robot::set_random_velocity() {
	glm::vec3 velocity(velocity_generator_(rng_), 0.f, velocity_generator_(rng_));
	set_velocity(velocity);
}

void Robot::update_explored(const glm::vec3& position) {
	int explored = id_;
	try {
		glm::ivec3 grid_position = octree_->map_to_grid(position);
		octree_->set(grid_position.x, grid_position.y, grid_position.z, explored);
	} catch (OutOfGridBoundsException& exception) {
		// ignore
	}
}

void Robot::update_robots(const std::vector<std::shared_ptr<Robot>>& robots) {
	robots_ = robots;
}

//void Robot::update_robots(const std::vector<Robot>& robots) {
//	robots_ = robots;
//}

void Robot::calculate_stopping_force() {
	glm::vec3 force_direction = velocity_;
	glm::vec3 displacement = goal_position_ - position_;
	float distance = glm::length(displacement);
	float acceleration = 0.f;
	if (distance > 1e-3) {
		acceleration = -std::pow(glm::length(velocity_), 2) / (2 * distance);
	}
	stopping_force_ = acceleration * force_direction * mass_;
	std::cout << "stopping : [" << stopping_force_.x << ", " << stopping_force_.y << ", " << stopping_force_.z << "]" << std::endl;
}

glm::vec3 Robot::calculate_inward_force() {
	glm::ivec3 mid_point(octree_->get_grid_resolution_per_side()/ 2, 0.f, octree_->get_grid_resolution_per_side()/ 2);
	glm::vec3 distance = 100.f * glm::normalize(octree_->map_to_position(mid_point) - position_);
	distance.y = 0.f;
	return distance;
}

bool Robot::is_velocity_non_zero() {
	float length = glm::length(velocity_);
	return (length > 5);

}

void Robot::handle_input() {
	//handle robot case
	//std::cout << current_robot_state_ << std::endl;
	switch (current_robot_state_) {

	case MOVING: {
		float distance_to_goal = glm::distance(goal_position_, position_);
		if (std::abs(distance_to_goal) < distance_to_goal_threshold_) {
			//calculate_stopping_force();
			//resultant_force_ = stopping_force_;
			nullify_forces();
			velocity_ = glm::vec3(0.f, 0.f, 0.f);
			change_state(STOPPING);
		}

		break;
	}
	case STOPPING: {
		if (!is_velocity_non_zero()) {
			nullify_forces();
			velocity_ = glm::vec3(0.f, 0.f, 0.f);
			change_state(STOPPED);
		}
		break;
	}
	case STOPPED: {
		nullify_forces();
		resultant_force_ = calculate_resultant_force();
		change_state(MOVING);
		break;
	}
	default: break;

	}


	//switch (current_swarming_state_) {
	//case EXPLORING:  {
	//	if (is_at_edge()) {
	//		//stop();
	//		change_state(AT_EDGE);
	//	}
	//	else {
	//		resultant_force_ = calculate_resultant_force();
	//	}
	//	break;
	//}
	//case AT_EDGE: {
	//	resultant_force_ = calculate_inward_force();
	//	change_state(GOING_INWARD);
	//	break;
	//}
	//case GOING_INWARD: {
	//	if (!is_at_edge()) {
	//		//stop();
	//		change_state(EXPLORING);
	//	}

	//}
	//default: break;

	//}

}


void Robot::update(glm::mat4 global_model) {

	std::chrono::milliseconds current_timestamp = std::chrono::duration_cast<std::chrono::milliseconds>(
		std::chrono::system_clock::now().time_since_epoch());

	if (last_updated_time_ <= 0) {
		last_updated_time_ = current_timestamp.count();
	} else {

		auto delta_time = (current_timestamp.count() - last_updated_time_) / 1000.f;
		delta_time *= 4;



		handle_input();

		// calculate position, velocity through euler integration

		glm::vec3 acceleration = resultant_force_ / mass_;
		position_ += (velocity_ * (static_cast<float>(delta_time)));
		velocity_ += acceleration * (static_cast<float>(delta_time));

		// calculate position, velocity through velocity verlet for 2nd order approximation
		//glm::vec3 acceleration = resultant_force_ / mass_;
		//// time += delta_time
		//position_ += delta_time * (velocity_ + (static_cast<float>(delta_time) * acceleration) / 2.f);

		////handle_input();
		//glm::vec3 new_acceleration = glm::vec3(0.f, 0.f, 0.f);

		//velocity_ += delta_time * (acceleration + new_acceleration) / 2.f;

		//// update position
		//for (auto& mesh : mesh_) {
		//	// integrate position


		//	position_ =  velocity_ * static_cast<float>(delta_time);
		//}

		cv::Vec4f green(0.f, 1.f, 0.f, 1.f);
		cv::Vec4f red(0.f, 1.f, 0.f, 1.f);
		cv::Vec4f blue(0.f, 0.f, 1.f, 1.f);
		visualize_force(0, explore_force_, blue, false);
		visualize_force(1, separation_force_, red, false);

		// update rendered mesh
		for (auto& render_entity : mesh_) {
			//glm::mat4 translate_model = glm::translate(render_entity.get_model(), position_);
			glm::mat4 translate_model = glm::translate(glm::mat4(1.f), position_);
			render_entity.set_model(translate_model * render_entity.get_initial_model());
		}

		last_updated_time_ = current_timestamp.count();

		//  update octtree visit
		update_explored(position_);


		//if (last_timeout_ <= 0) {
		//	last_timeout_ = last_updated_time_;
		//} else {
		//	auto delta_timeout = last_updated_time_ - last_timeout_;
		//	if (delta_timeout > timeout_) {
		//		set_random_velocity();
		//		last_timeout_ = last_updated_time_;
		//	}
		//}

	}
}

Robot& Robot::operator=(const Robot& other) {
	if (this != &other) {
		id_ = other.id_;
		octree_ = other.octree_;

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

		// seperation force requirements
		//glm::vec3 center_of_mass_;
		//std::vector<Robot> robots_ = other.robots_;
		minimum_seperation_distance_ = other.minimum_seperation_distance_;
		separation_distance_threshold_ = other.separation_distance_threshold_;

		// exploration force requirements
		// location in grid for frontier calculation
		mass_ = other.mass_;
		max_velocity_ = other.max_velocity_;
	}
	return *this;
}

//bool Robot::operator=(const Robot& other) {
//	return (other.id_ == id_);
//}
