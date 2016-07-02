#include "simulatorthread.h"
#include "swarmutils.h"

#define PI 3.14159265

const std::string SimulatorThread::DEFAULT_INTERIOR_MODEL_FILENAME = "interior/l-shape-floor-plan.obj";

const int SimulatorThread::DEFAULT_NO_OF_ROBOTS = 3;
const std::string SimulatorThread::OCCUPANCY_GRID_NAME = "occupancy_grid";
const std::string SimulatorThread::OCCUPANCY_GRID_OVERLAY_NAME = "occupancy_grid_overlay";
const int SimulatorThread::OCCUPANCY_GRID_HEIGHT = 2;

void SimulatorThread::create_robots() {
	//m_shader.bind();



	std::vector<glm::vec3> robot_positions = create_starting_formation(static_cast<Formation>(formation_));
	assert(no_of_robots_ == robot_positions.size());

	RenderMesh robot_mesh;
	cv::Vec4f color(0.f, 1.f, 1.f, 1.f);
	//create_robot_model(robot_mesh, color);
		
	for (int i = 0; i < no_of_robots_; ++i) {
		std::random_device rd;
		std::mt19937 rng;
		rng.seed(rd());
		//std::uniform_real_distribution<float> color_generator(0.f, 1.f);
		//cv::Vec4f color(color_generator(rng), color_generator(rng), color_generator(rng), 1.f);
		//RenderMesh robot_mesh;
		////cv::Vec4f color(0.f, 1.f, 1.f, 1.f);
		//create_robot_model(robot_mesh, color);

		//robot_color_map_[i] = color;


		//Robot robot(uniform_locations_, i, occupancy_grid_, explore_constant_, separation_constant_, goto_work_constant_, separation_distance_, &m_shader);
		//robot.mesh_.push_back(robot_mesh[0]);
		//robots_.push_back(robot);

		Robot* robot = new ExperimentalRobot(uniform_locations_, 
			i, occupancy_grid_, collision_grid_,  separation_constant_, alignment_constant_, cluster_constant_, explore_constant_,
			sensor_range_, discovery_range_, 
			separation_distance_, robot_positions[i], square_radius_, bounce_function_power_, bounce_function_multiplier_, false);

		//robot->mesh_.push_back(robot_mesh[0]);
		robots_.push_back(robot);
	}
}

void BridgeObject::update_sim_results(int group_id, int thread_id, int iteration, 
	double separation_constant, double alignment_constant, double cluster_constant, double explore_constant, 
	double separation_distance, double simultaneous_sampling, double time_taken, double occlusion, double coverage) {
	//emit send_sim_results(group_id, thread_id, iteration, separation_constant, alignment_constant, cluster_constant, explore_constant,
	//	separation_distance, simultaneous_sampling, time_taken, occlusion, coverage);
	lock_.lock();
	emit send_sim_results(group_id, thread_id, iteration);
	lock_.unlock();
}

void BridgeObject::start_thread(SimulatorThread* sim_thread) {
		sim_thread->reset_sim();
		thread_pool_.start(sim_thread);
}

SimulatorThread::SimulatorThread(BridgeObject* bridge, int group_id, int thread_id, int no_of_robots, unsigned int grid_resolution, float grid_length, 
		std::string interior_filename, double interior_scale, glm::vec3 interior_offset, glm::mat4 interior_rotation, 
		int max_time_taken, 
		double separation_constant, double alignment_constant,
		double cluster_constant, double explore_constant, double sensor_range,
		int discovery_range, double separation_distance, Formation formation, 
		double square_radius, double bounce_function_power, double bounce_function_multipler, int iteration) :

		separation_constant_(separation_constant), alignment_constant_(alignment_constant), cluster_constant_(cluster_constant), explore_constant_(explore_constant), 
		separation_distance_(separation_distance), sensor_range_(sensor_range), discovery_range_(discovery_range), 
		max_time_taken_(max_time_taken), square_radius_(square_radius),
		bounce_function_power_(bounce_function_power), 
		bounce_function_multiplier_(bounce_function_multipler), formation_(static_cast<Formation>(formation)),
		grid_resolution_(grid_resolution), grid_length_(grid_length), no_of_robots_(no_of_robots),
		occupancy_grid_(nullptr), collision_grid_(nullptr),
		interior_scale_(interior_scale), interior_offset_(interior_offset),
		model_rotation_(interior_rotation), interior_model_filename_(interior_filename), group_id_(group_id), thread_id_(thread_id), iteration_(iteration), bridge_(bridge)
{
	//reset_sim();
}


SimulatorThread::~SimulatorThread()
{
	cleanup();
}

std::vector<glm::vec3> SimulatorThread::create_starting_formation(Formation type) {
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

//void SimulatorThread::create_light_model(RenderMesh& light_mesh) {
//	VertexBufferData vb_data;
//	SwarmUtils::load_obj("bulb/bulb.obj", vb_data);
//
//	cv::Vec4f color(253.f, 184.f, 19.f, 255.f);
//	color /= 255.f;
//	for (auto& vertice : vb_data.positions) {
//		vb_data.colors.push_back(color);
//	}
//
//	RenderEntity light_model(GL_TRIANGLES, &m_shader);
//	light_model.upload_data_to_gpu(vb_data);
//
//	light_model.set_model(glm::scale(glm::mat4(1.f), glm::vec3(10.f, 10.f, 10.f)));
//	light_mesh.push_back(light_model);
//}

//void SimulatorThread::create_robot_model(RenderMesh& robot_mesh, cv::Vec4f color) {
//	VertexBufferData vb_data;
//	SwarmUtils::load_obj("PYROS/pyros-obj.obj", vb_data);
//
//	//cv::Vec4f color(253.f, 184.f, 19.f, 255.f);
//	//color /= 255.f;
//
//	vb_data.colors.resize(vb_data.positions.size());
//	for (auto& color_holder : vb_data.colors) {
//		color_holder = color;
//	}
//
//	RenderEntity robot_model(GL_TRIANGLES, &m_shader);
//	robot_model.upload_data_to_gpu(vb_data);
//	robot_model.set_initial_model(glm::scale(glm::mat4(1.f), glm::vec3(.1f, .1f, .1f)));
//
//	//robot_model.set_model(glm::scale(glm::mat4(1.f), glm::vec3(10.f, 10.f, 10.f)));
//	robot_mesh.push_back(robot_model);
//}

//void SimulatorThread::create_lights() {
//
//	std::random_device rd;
//	std::mt19937 rng(rd());
//	std::uniform_real_distribution<float> color(0.5, 1); // uniform, unbiased
//	std::uniform_int_distribution<int> position(0, occupancy_grid_->get_grid_cube_length() 
//		* occupancy_grid_->get_grid_resolution_per_side());
//
//	m_shader.bind();
//
//	RenderMesh light_bulb;
//	create_light_model(light_bulb);
//
//	for (size_t i = 0; i < NO_OF_LIGHTS; ++i) {
//		Light* light = new Light(uniform_locations_);
//
//		std::stringstream light_name;
//		light_name << "lights[" << i << "]";
//
//		std::stringstream light_position_name;
//		light_position_name << light_name.str() << ".position";
//		light->light_position_location_ = m_shader.uniformLocation(light_position_name.str().c_str());
//
//		std::stringstream light_color_name;
//		light_color_name << light_name.str() << ".color";
//		light->light_color_location_ = m_shader.uniformLocation(light_color_name.str().c_str());
//
//		//glm::vec3 light_position(0, 0, -100);
//		glm::vec3 light_position(position(rng), 500.f, position(rng));
//		glm::vec3 light_color(color(rng), color(rng), color(rng));
//
//		light->color_ = light_color;
//		light->position_ = light_position;
//
//		glUniform3fv(light->light_color_location_, 1, glm::value_ptr(light_color));
//
//		glm::mat4 RT = glm::translate(glm::mat4(1.f), light_position);
//		RenderMesh light_mesh = light_bulb;
//		update_model(light_mesh, RT);
//		//default_scene_.push_back(light);
//		 
//		light->mesh_ = light_mesh;
//
//		lights_.push_back(light);
//	}
//}

void SimulatorThread::derive_floor_plan(const VertexBufferData& bufferdata, float scale, const glm::vec3& offset) {
	std::vector<std::vector<cv::Vec3f>> triangle_list;
	cv::Vec3f vertex_offset(offset.x, offset.y, offset.z);
	glm::mat3 rot_mat = glm::mat3(model_rotation_);
	cv::Mat rotation_mat = SwarmUtils::convert_mat(rot_mat);

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

bool SimulatorThread::intersect(const cv::Vec3f& n, float d, 
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


void SimulatorThread::load_interior_model() {
	cv::Mat unit_matrix = cv::Mat::eye(4, 4, CV_64F);

	std::string model_filename = (interior_model_filename_.compare("") == 0) ? DEFAULT_INTERIOR_MODEL_FILENAME
		: interior_model_filename_;

	VertexBufferData* vertex_buffer_data = new VertexBufferData();
	SwarmUtils::load_obj(model_filename,  *vertex_buffer_data);

	float scale = interior_scale_;

	//if (render_) {
	//	cv::Vec4f color(0.8f, 0.5f, 0.5f, 1.f);
	//	vertex_buffer_data->colors = std::vector<cv::Vec4f>(vertex_buffer_data->positions.size(), color);

	//	RenderEntity interior_model(GL_TRIANGLES, &m_shader);
	//	//interior_model.upload_data_to_gpu(vertices, colors, normals, uvs, indices, count, offset, base_index);
	//	interior_model.upload_data_to_gpu(*vertex_buffer_data);


	//	RenderMesh interior_model_mesh;
	//	interior_model_mesh.push_back(interior_model);

	//	VisObject* interior_model_object = new VisObject(uniform_locations_);
	//	interior_model_object->mesh_ = interior_model_mesh;


	//	glm::mat4 model = glm::scale(glm::mat4(1.f), glm::vec3(scale, scale, scale));
	//	model = glm::translate(model, interior_offset_);
	//	model = model * model_rotation_;
	//	update_model(interior_model_object->mesh_, model);

	//	default_vis_objects_.push_back(interior_model_object);
	//}


	if (interior_model_filename_.compare("") > 0) {
		derive_floor_plan(*vertex_buffer_data, scale, interior_offset_);
	}

	delete vertex_buffer_data;
}

//void SimulatorThread::change_to_top_down_view() {
//	look_at_z_ = 0.f;
//	look_at_x_ = 0.f;
//	look_at_y_ = 0.f;
//
//	m_xRot = 0.f;
//	m_yRot = 0.f;
//	m_zRot = 0.f;
//
//	camera_distance_ = 0.f;
//	float length = grid_resolution_per_side_ * grid_length_;
//	glm::vec3 mid_point(length / 2.f, 0.f, length/2.f);
//
//	center_ = mid_point;
//	eye_ = mid_point;
//	float distance = (length / 2.f) / std::tan(glm::radians(fovy_ /2.f));
//	eye_.y = distance;
//
//	up_ = glm::vec3(1.f, 0.f, 0.f);
//	
//}
//
//void SimulatorThread::update_perimiter_positions_in_overlay() {
//	auto perimeter_list = occupancy_grid_->get_unexplored_perimeter_list();
//	cv::Vec4f dark_green(60.f, 179.f, 113.f, 255.f);
//	dark_green /= 255.f;
//	for (auto& perimeter_pos : perimeter_list) {
//		overlay_->update_grid_position(perimeter_pos, dark_green);
//	}
//}

void SimulatorThread::reset_sim() {
	//makeCurrent();
	//shutdown_worker();
	cleanup();

	//set_model_rotation(-90.f, 0.f, -90.f);
	//sim_results_updated_ = false;
	time_step_count_ = 0;

	//render_ = gui_render_;

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
		//if (render_) {
		//	robot->set_show_forces(show_forces_);
		//}
	}




	// graphics setup
	//change_to_top_down_view();
	//if (render_) {
	//	create_occupancy_grid(grid_resolution_per_side_, grid_length_);
	//	robot_color_map_[occupancy_grid_->get_interior_mark()] = cv::Vec4f(1.f, 1.f, 1.f, 1.f);
	//	create_lights();

	//	overlay_ = new GridOverlay(uniform_locations_,
	//		occupancy_grid_, grid_resolution_per_side_, grid_length_, robot_color_map_, &m_shader);
	//	update_perimiter_positions_in_overlay();
	//	reset_vis_objects_.push_back(overlay_);
	//	for (auto& robot : robots_) {
	//		robot->set_grid_overlay(overlay_);
	//	}
	//}

	//robot_worker_ = new RobotWorker();
	//robot_worker_->moveToThread(&robot_update_thread_);
	//robot_worker_->init();
	//connect(this, &SimulatorThread::physics_thread_pause, robot_worker_, &RobotWorker::pause);
	//connect(this, &SimulatorThread::physics_thread_resume, robot_worker_, &RobotWorker::resume);
	//connect(this, &SimulatorThread::physics_thread_step, robot_worker_, &RobotWorker::step);
	//connect(robot_worker_, &RobotWorker::update_time_step_count, this, &SimulatorThread::update_time_step_count);
	//connect(robot_worker_, &RobotWorker::update_sim_results, this, &SimulatorThread::update_sim_results);
	//connect(robot_worker_, &RobotWorker::update_sampling, this, &SimulatorThread::update_sampling);
	//connect(&robot_update_thread_, &QThread::started, robot_worker_, &RobotWorker::do_work);

	//robot_worker_->set_robots(robots_);
	//robot_worker_->set_occupancy_tree(occupancy_grid_);
	//robot_worker_->set_slow_down(slow_down_);
	//robot_worker_->set_max_time_taken(max_time_taken_);
	//robot_update_thread_.start();
}

void SimulatorThread::finish_work() {
	double simultaneous_sampling = occupancy_grid_->calculate_multi_sampling_factor();
	double occlusion = 0.0;
	//double coverage = calculate_coverage();
	double coverage = 0.0;
	//bridge_->update_sim_results(group_id_, thread_id_, separation_constant_, alignment_constant_, cluster_constant_, explore_constant_,
	//	separation_distance_, simultaneous_sampling, time_step_count_, occlusion, coverage, iteration_);

	std::random_device rd;
	std::mt19937 mt(rd());
	std::uniform_real_distribution<> uniform_real_distribution(0, 10);

	time_step_count_ = uniform_real_distribution(mt);

	//emit send_sim_results(group_id_, thread_id_, iteration_);
	emit send_sim_results(group_id_, thread_id_, iteration_, separation_constant_, alignment_constant_, cluster_constant_, explore_constant_,
		separation_distance_, simultaneous_sampling, time_step_count_, occlusion, coverage);

	abort();
}
	

void SimulatorThread::run() {
	while (!aborted_) {
		if (time_step_count_ > max_time_taken_) {
			finish_work();
		}
		if (occupancy_grid_->no_of_unexplored_cells() > 0) {

		}
		else {
			finish_work();
		}
		for (auto& robot : robots_) {
			robot->update(time_step_count_);
		}
		time_step_count_++;
		//QCoreApplication::processEvents();
	}
	std::cout << "Ending : " << group_id_ << " " << thread_id_ << " " << iteration_ << "\n";
	finish_work();
}

void SimulatorThread::abort() {
	aborted_ = true;
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
	
}

void SimulatorThread::set_no_of_robots(int no_of_robots) {
	no_of_robots_ = no_of_robots;
}

void SimulatorThread::set_separation_distance(double distance) {
	separation_distance_ = distance;
}

void SimulatorThread::set_grid_resolution(int grid_resolution) {
	//grid_resolution_ = std::pow(std::pow(2, grid_resolution), 3);
	grid_resolution_ = grid_resolution;
}

void SimulatorThread::set_grid_length(int grid_length) {
	grid_length_ = grid_length;
}

void SimulatorThread::set_interior_scale(double scale) {
	interior_scale_ = scale;
}

void SimulatorThread::set_interior_offset(glm::vec3 offset) {
	interior_offset_ = offset;
}

void SimulatorThread::set_exploration_constant(double constant) {
	explore_constant_ = constant;
}

void SimulatorThread::set_separation_constant(double constant) {
	separation_constant_ = constant;
}

void SimulatorThread::set_alignment_constant(double constant) {
	alignment_constant_ = constant;
}

void SimulatorThread::set_cluster_constant(double constant) {
	cluster_constant_ = constant;
}

void SimulatorThread::set_model_rotation(double x_rotation, double y_rotation, double z_rotation) {
	glm::mat4 rotateX = glm::rotate(glm::mat4(1.f), glm::radians((float)x_rotation), glm::vec3(1.f, 0.f, 0.f));
	glm::mat4 rotateY = glm::rotate(rotateX, glm::radians((float)y_rotation), glm::vec3(0.f, 1.f, 0.f));
	glm::mat4 rotateZ = glm::rotate(rotateY,  glm::radians((float)z_rotation), glm::vec3(0.f, 0.f, 1.f));
	
	model_rotation_ = rotateZ;
}

void SimulatorThread::set_square_formation_radius(double radius) {
	square_radius_ = radius;
}

void SimulatorThread::set_bounce_function_power(double bounce_function_power) {
	bounce_function_power_ = bounce_function_power;
}

void SimulatorThread::set_bounce_function_multiplier(double bounce_function_multiplier) {
	bounce_function_multiplier_ = bounce_function_multiplier;
}

void SimulatorThread::set_sensor_range(double sensor_range) {
	sensor_range_ = sensor_range;
}

void SimulatorThread::set_discovery_range(int discovery_range) {
	discovery_range_ = discovery_range;
}

void SimulatorThread::set_formation(int formation) {
	formation_ = formation;
}

//void SimulatorThread::set_show_forces(int show) {
//	show_forces_ = show;
//	for (auto& robot : robots_) {
//		robot->set_show_forces(show_forces_);
//	}
//}

void SimulatorThread::set_model_filename(QString filename) {
	interior_model_filename_ = filename.toStdString();
}
