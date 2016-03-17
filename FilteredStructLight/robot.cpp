#include "robot.h"
#include <chrono>
#include <glm/gtc/matrix_transform.hpp> // glm::translate, glm::rotate, glm::scale, glm::perspective
#include <cmath>
#include <queue>



void Robot::set_explore_constant(float constant) {
	explore_constant_ = constant;
}

void Robot::set_seperation_constant(float constant) {
	seperation_constant_ = constant;
}

void Robot::set_work_constant(float constant) {
	work_constant_ = constant;
}

void Robot::change_state(State state) {
	current_state_ = state;
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


	if (current_state_ == EXPLORING) {
		try {
			auto current_cell = octree_.map_to_grid(position_);

			glm::ivec3 frontier_cell;
			bool something_to_explore = octree_.frontier_bread_first_search(current_cell, frontier_cell);
			if (something_to_explore) {
				auto move_to_position = octree_.map_to_position(frontier_cell);
				// only 2D
				move_to_position.y = 10.f;
				//explore_force_ = calculate_force(move_to_position);
				explore_force_ = calculate_force(move_to_position, explore_constant_);
			}
			else {
				// we are done
				explore_force_ = glm::vec3(0.f);
			}
		} catch (OutOfGridBoundsException& ex) {
			std::cout << "Position is out of bounds : [" << position_.x << ", " << position_.y << ", " << position_.z << "]" << std::endl;
			explore_force_ = glm::vec3(0.f);
		}
	}
}

void Robot::calculate_seperation_force() {

	seperation_force_ = glm::vec3(0.f, 0.f, 0.f);
	for (auto& robot : robots_) {
		if (robot.id_ != id_) {
			// naively, just a set distance away from the center
			auto seperation_vector = robot.position_ - position_;
			float distance = glm::length(seperation_vector);
			if (distance < minimum_seperation_distance_) {
				// apply some force away from center of mass 

				// minimum seperation distance + some threshold??
				float distance_to_travel = (minimum_seperation_distance_ - distance) + seperation_distance_threshold_;
				glm::vec3 move_to_direction = glm::normalize(-seperation_vector);
				//move_to_direction.y = 10.f;
				glm::vec3 move_to_position = distance_to_travel * move_to_direction;

				seperation_force_ += calculate_force(move_to_position, seperation_constant_);
			}
		}
	}
	seperation_force_.y = 0.f;

}

void Robot::calculate_work_force() {
	work_force_ = glm::vec3(0.f, 0.f, 0.f);

}

void Robot::calculate_out_of_bounds_force() {
	float out_of_bounds_constant_ = 10 * explore_constant_;
	try {
		auto current_cell = octree_.map_to_grid(position_);
		if (current_state_ == GOING_IN_BOUND) {
			velocity_ = glm::vec3(0.f, 0.f, 0.f);
			out_of_bounds_force_ = glm::vec3(0.f, 0.f, 0.f);
			change_state(EXPLORING);
		}
	} catch (OutOfGridBoundsException& ex) {
		if (current_state_ != GOING_IN_BOUND) {
				
			out_of_bounds_force_ = -glm::normalize(velocity_) * 100.f;
			explore_force_ = glm::vec3(0.f, 0.f, 0.f);
			seperation_force_ = glm::vec3(0.f, 0.f, 0.f);
			velocity_ = glm::vec3(0.f);
			change_state(GOING_IN_BOUND);
		}
	}

}

glm::vec3 Robot::calculate_resultant_force() {
	calculate_explore_force();
	calculate_seperation_force();
	calculate_work_force();
	calculate_out_of_bounds_force();

	//glm::vec3 force = explore_constant_ * explore_force_
	//	+ seperation_force_ * seperation_constant_ + work_constant_ * work_force_;

	glm::vec3 force =  explore_force_
		+ seperation_force_ + work_force_ + out_of_bounds_force_;

	if (force.y > 0.f) {
		std::cout << "explore : [" << explore_force_.x << ", " << explore_force_.y << ", " << explore_force_.z << "]" << std::endl;
		std::cout << "seperation : [" << seperation_force_.x << ", " << seperation_force_.y << ", " << seperation_force_.z << "]" << std::endl;
		std::cout << "out_of_bounds : [" << out_of_bounds_force_.x << ", " << out_of_bounds_force_.y << ", " << out_of_bounds_force_.z << "]" << std::endl;
	}

	// ToDo : add some gaussian noise

	return force;
}

Robot::Robot(UniformLocations& locations, unsigned int id, SwarmOctTree& octree) : VisObject(locations), id_(id),
	timeout_(5000), last_timeout_(0), last_updated_time_(0), octree_(octree) {
	std::random_device rd;
	rng_.seed(rd());
	velocity_generator_ = std::uniform_real_distribution<float>(-0.2, 0.2);
	position_generator_ = std::uniform_int_distribution<int>(0, 500);
	position_ = glm::vec3(position_generator_(rng_), 10.f, position_generator_(rng_));

	//set_random_velocity();

	mass_ = 10;
	max_velocity_ = 50;

	explore_constant_ = work_constant_ = 1.f;
	seperation_constant_ = 10 * explore_constant_;
	
	seperation_distance_threshold_ = 50;
	minimum_seperation_distance_ = 10;

	travelling_in_bound_ = false;
	change_state(EXPLORING);

}

void Robot::set_velocity(glm::vec3 velocity) {
	velocity_ = velocity;
}

void Robot::set_random_velocity() {
	glm::vec3 velocity(velocity_generator_(rng_), 0.f, velocity_generator_(rng_));
	set_velocity(velocity);
}

void Robot::update_explored(const glm::vec3& position) {
	int explored = 1;
	try {
		glm::ivec3 grid_position = octree_.map_to_grid(position);
		octree_.set(grid_position.x, grid_position.y, grid_position.z, explored);
	} catch (OutOfGridBoundsException& exception) {
		// ignore
	}
}

void Robot::update_robots(const std::vector<Robot>& robots) {
	robots_ = robots;
}

void Robot::update(glm::mat4 global_model) {

	std::chrono::milliseconds current_timestamp = std::chrono::duration_cast<std::chrono::milliseconds>(
		std::chrono::system_clock::now().time_since_epoch());

	if (last_updated_time_ <= 0) {
		last_updated_time_ = current_timestamp.count();
	} else {

		auto delta_time = (current_timestamp.count() - last_updated_time_) / 1000.f;

		glm::vec3 resultant_force = calculate_resultant_force();

		// calculate position, velocity through velocity verlet for 2nd order approximation
		glm::vec3 acceleration = resultant_force / mass_;
		// time += delta_time
		position_ += delta_time * (velocity_ + (static_cast<float>(delta_time) * acceleration) / 2.f);

		glm::vec3 new_resultant_force = calculate_resultant_force();
		glm::vec3 new_acceleration = new_resultant_force / mass_;

		velocity_ += delta_time * (acceleration + new_acceleration) / 2.f;

		//// update position
		//for (auto& mesh : mesh_) {
		//	// integrate position


		//	position_ =  velocity_ * static_cast<float>(delta_time);
		//}

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
		seperation_constant_ = other.seperation_constant_;
		work_constant_ = other.work_constant_;

		explore_force_ = other.explore_force_;
		seperation_force_ = other.seperation_force_;
		work_force_ = other.work_force_;

		// seperation force requirements
		//glm::vec3 center_of_mass_;
		//std::vector<Robot> robots_ = other.robots_;
		minimum_seperation_distance_ = other.minimum_seperation_distance_;
		seperation_distance_threshold_ = other.seperation_distance_threshold_;

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
