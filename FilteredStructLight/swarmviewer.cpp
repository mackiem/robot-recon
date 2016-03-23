#include "swarmviewer.h"
#include <random>
#include <glm/vec3.hpp> // glm::vec3
#include <glm/vec4.hpp> // glm::vec4
#include <glm/mat4x4.hpp> // glm::mat4
#include <glm/gtc/matrix_transform.hpp> // glm::translate, glm::rotate, glm::scale, glm::perspective
#include <glm/gtc/type_ptr.hpp>
#include <chrono>

const std::string SwarmViewer::DEFAULT_INTERIOR_MODEL_FILENAME = "interior/house interior.obj";
const int SwarmViewer::DEFAULT_NO_OF_ROBOTS = 3;

#define NO_OF_LIGHTS 5
#define NO_OF_ROBOTS 10

const std::string SwarmViewer::OCCUPANCY_GRID_NAME = "occupancy_grid";
const std::string SwarmViewer::OCCUPANCY_GRID_OVERLAY_NAME = "occupancy_grid_overlay";


void SwarmViewer::Light::update(glm::mat4 global_model) {
	glm::vec3 initial_position(0.f);
	glm::mat4 model = glm::translate(glm::mat4(1.f), (position_));
	glm::vec4 transformed_position = global_model * model * glm::vec4(initial_position, 1.f);
	
	glm::vec3 world_position =
		glm::vec3(transformed_position.x, transformed_position.y, transformed_position.z) / transformed_position.w;

	glUniform3fv(light_position_location_, 1, glm::value_ptr(world_position));
}

SwarmViewer::GridOverlay::GridOverlay(UniformLocations& locations, std::shared_ptr<SwarmOctTree> octree, 
	unsigned grid_resolution, float grid_length, std::map<int, cv::Vec4f> robot_color_map, QGLShaderProgram* shader) : 
	VisObject(locations), octree_(octree), grid_resolution_per_side_(grid_resolution), grid_length_(grid_length), robot_color_map_(robot_color_map), shader_(shader)  {
	
	create_mesh(true);
}

void SwarmViewer::GridOverlay::create_mesh(bool initialize) {
	int grid_length_per_side = grid_resolution_per_side_ / 2;
	int y = 2.f;

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

			cv::Vec3f pt_1  = cv::Vec3f(x * grid_length_, y, z * grid_length_);
			cv::Vec3f pt_2  = cv::Vec3f(x * grid_length_, y, (z+1) * grid_length_);
			cv::Vec3f pt_3  = cv::Vec3f((x+1) * grid_length_, y, (z+1) * grid_length_);
			cv::Vec3f pt_4  = cv::Vec3f((x+1) * grid_length_, y, z * grid_length_);
		
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
			int explored_robot = octree_->explored_by(current_position);

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

	} else {
		mesh_[0].upload_data_to_gpu(bufferdata, true);
	}

}

void SwarmViewer::GridOverlay::update(glm::mat4 global_model) {
	create_mesh(false);
}

void SwarmViewer::load_inital_models() {

	load_interior_model();
	reset_sim();
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

void SwarmViewer::load_interior_model() {
	cv::Mat unit_matrix = cv::Mat::eye(4, 4, CV_64F);

	std::vector<cv::Vec3f> vertices;
	std::vector<cv::Vec3f> normals;
	std::vector<cv::Vec4f> colors;
	std::vector<cv::Vec2f> uvs;
	std::vector<unsigned int> indices;
	std::vector<int> count;
	std::vector<int> offset;
	std::vector<int> base_index;


	std::string model_filename = (interior_model_filename_.compare("") == 0) ? DEFAULT_INTERIOR_MODEL_FILENAME
		: interior_model_filename_;

	load_obj(model_filename,  vertices, normals, uvs, indices, count, offset, base_index);

	cv::Vec4f color(0.8f, 0.5f, 0.5f, 1.f);
	for (auto& vertice : vertices) {
		colors.push_back(color);
	}


	RenderEntity interior_model(GL_TRIANGLES, &m_shader);
	interior_model.upload_data_to_gpu(vertices, colors, normals, uvs, indices, count, offset, base_index);

	//GLenum error = glGetError();

	RenderMesh interior_model_mesh;
	interior_model_mesh.push_back(interior_model);
	
	std::shared_ptr<VisObject> interior_model_object = std::make_shared<VisObject>(uniform_locations_);
	interior_model_object->mesh_ = interior_model_mesh;
	float scale = 2.f;
	glm::mat4 model = glm::scale(glm::mat4(1.f), glm::vec3(scale, scale, scale));
	update_model(interior_model_object->mesh_, model);


	default_vis_objects_.push_back(interior_model_object);

	//default_scene_.push_back(interior_model_mesh);

	create_lights();
}

void SwarmViewer::custom_init_code() {
	timer_ = new QTimer(this);
	connect(timer_, SIGNAL(timeout()), this, SLOT(update()));
	timer_->start(20);

	//glEnable(GL_CULL_FACE);

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

	// assumption - global position of other robots are known
	for (auto& robot : robots_) {
		robot->update_robots(robots_);
	}

	for (auto& robot : robots_) {
		robot->update(model_);
		robot->draw(model_, camera_, projection_);
	}

	for (auto& light : lights_) {
		light.update(model_);
		//light.draw(model_, camera_, projection_);
	}

	// draw grid
	//create_occupancy_grid_overlay(grid_resolution_per_side_, grid_length_);
	//grid_overlay_.draw(model_, camera_, projection_);
	//grid_.draw(model_, camera_, projection_);

}

void SwarmViewer::draw_mesh(RenderMesh& mesh) {
}

void SwarmViewer::reset_sim() {

	robot_color_map_.clear();
	reset_vis_objects_.clear();
	robots_.clear();

	grid_resolution_per_side_ = std::pow(grid_resolution_, 1.f / 3.f);
	octree_ = std::make_shared<SwarmOctTree>(grid_length_, grid_resolution_);

	create_robots();
	create_occupancy_grid(grid_resolution_per_side_, grid_length_);
	robot_color_map_[octree_->get_interior_mark()] = cv::Vec4f(1.f, 1.f, 1.f, 1.f);

	std::shared_ptr<GridOverlay> overlay(new GridOverlay(uniform_locations_, 
		octree_, grid_resolution_per_side_, grid_length_, robot_color_map_, &m_shader));
	reset_vis_objects_.push_back(overlay);

}

SwarmViewer::SwarmViewer(const QGLFormat& format, QWidget* parent) : RobotViewer(format, parent)
{


	//grid_ = VisObject(uniform_locations_);
	//grid_overlay_ = VisObject(uniform_locations_);
	//octree_ = SwarmOctTree(grid_length_, grid_resolution_per_side_);
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
	for (auto& vertice : vb_data.positions) {
		vb_data.colors.push_back(color);
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
	std::uniform_int_distribution<int> position(0, 300);

	m_shader.bind();

	RenderMesh light_bulb;
	create_light_model(light_bulb);

	for (size_t i = 0; i < NO_OF_LIGHTS; ++i) {
		Light light(uniform_locations_);

		std::stringstream light_name;
		light_name << "lights[" << i << "]";

		std::stringstream light_position_name;
		light_position_name << light_name.str() << ".position";
		light.light_position_location_ = m_shader.uniformLocation(light_position_name.str().c_str());

		std::stringstream light_color_name;
		light_color_name << light_name.str() << ".color";
		light.light_color_location_ = m_shader.uniformLocation(light_color_name.str().c_str());

		//glm::vec3 light_position(0, 0, -100);
		glm::vec3 light_position(position(rng), position(rng), position(rng));
		glm::vec3 light_color(color(rng), color(rng), color(rng));

		light.color_ = light_color;
		light.position_ = light_position;

		glUniform3fv(light.light_color_location_, 1, glm::value_ptr(light_color));

		glm::mat4 RT = glm::translate(glm::mat4(1.f), light_position);
		RenderMesh light_mesh = light_bulb;
		update_model(light_mesh, RT);
		//default_scene_.push_back(light);
		 
		light.mesh_ = light_mesh;

		lights_.push_back(light);
	}
}

void SwarmViewer::create_robots() {
	m_shader.bind();


	for (int i = 0; i < no_of_robots_; ++i) {
		RenderMesh robot_mesh;
		std::random_device rd;
		std::mt19937 rng;
		rng.seed(rd());
		std::uniform_real_distribution<float> color_generator(0.f, 1.f);
		cv::Vec4f color(color_generator(rng), color_generator(rng), color_generator(rng), 1.f);

		robot_color_map_[i] = color;

		create_robot_model(robot_mesh, color);

		//Robot robot(uniform_locations_, i, octree_, explore_constant_, separation_constant_, goto_work_constant_, separation_distance_, &m_shader);
		//robot.mesh_.push_back(robot_mesh[0]);
		//robots_.push_back(robot);

		std::shared_ptr<Robot> robot(new Robot(uniform_locations_, 
			i, octree_, explore_constant_, separation_constant_, 
			goto_work_constant_, separation_distance_, &m_shader));
		robot->mesh_.push_back(robot_mesh[0]);
		robots_.push_back(robot);
	}
}



void SwarmViewer::create_occupancy_grid(int grid_resolution_per_side, int grid_size) {
	makeCurrent();

	//prepareShaderProgram(":/FilteredStructLight/model_vert.glsl", ":/FilteredStructLight/model_frag.glsl", line_shader_);

	cv::Vec4f green(0.f, 1.f, 0.f, 1.f);

	float y = 2.f;
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

	std::shared_ptr<VisObject> grid = std::make_shared<VisObject>(uniform_locations_);
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
	grid_resolution_ = std::pow(std::pow(2, grid_resolution), 3);
}

void SwarmViewer::set_grid_length(int grid_length) {
	grid_length_ = grid_length;
}

void SwarmViewer::set_interior_scale(int scale) {
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

void SwarmViewer::set_goto_work_constant(double constant) {
	goto_work_constant_ = constant;
}

void SwarmViewer::set_show_interior(int show) {
	show_interior_ = show;
}

