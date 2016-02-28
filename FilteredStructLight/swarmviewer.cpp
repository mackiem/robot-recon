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

#define NO_OF_LIGHTS 1
#define NO_OF_ROBOTS 10

SwarmViewer::Robot::Robot() : timeout_(5000), last_updated_time_(0), last_timeout_(0) {
	std::random_device rd;
	rng_.seed(rd());
	velocity_generator_ = std::uniform_real_distribution<float>(-0.2, 0.2);
	position_generator_ = std::uniform_int_distribution<int>(0, 500);
	update_velocity();
	position_ = glm::vec3(position_generator_(rng_), 30.f, position_generator_(rng_));
}

void SwarmViewer::Robot::set_velocity(glm::vec3 velocity) {
	velocity_ = velocity;
}

void SwarmViewer::Robot::update_velocity() {
		glm::vec3 velocity(velocity_generator_(rng_), 0.f, velocity_generator_(rng_));
		set_velocity(velocity);
}

void SwarmViewer::Robot::move() {
	std::chrono::milliseconds current_timestamp = std::chrono::duration_cast<std::chrono::milliseconds>(
		std::chrono::system_clock::now().time_since_epoch()
		);
	if (last_updated_time_ <= 0) {
		last_updated_time_ = current_timestamp.count();
	} else {
		auto delta_time = current_timestamp.count() - last_updated_time_;
		for (auto& mesh : mesh_) {
			position_ =  velocity_ * static_cast<float>(delta_time);
		}
		last_updated_time_ = current_timestamp.count();
		for (auto& render_entity : mesh_) {
			glm::mat4 translate_model = glm::translate(render_entity.get_model(), position_);
			render_entity.set_model(translate_model);
		}


		if (last_timeout_ <= 0) {
			last_timeout_ = last_updated_time_;
		} else {
			auto delta_timeout = last_updated_time_ - last_timeout_;
			if (delta_timeout > timeout_) {
				update_velocity();
				last_timeout_ = last_updated_time_;
			}
		}

	}
}

void SwarmViewer::Robot::draw() {
}

void SwarmViewer::load_inital_models() {
	cv::Mat unit_matrix = cv::Mat::eye(4, 4, CV_64F);
	draw_robot(unit_matrix);

	// create 10 robots
	int no_of_robots = (no_of_robots_ <= 0) ? DEFAULT_NO_OF_ROBOTS : no_of_robots_;

	std::random_device rd;
	std::mt19937 rng(rd());
	std::uniform_int_distribution<int> gen(0, 1000); // uniform, unbiased

	//for (int i = 0; i < no_of_robots; ++i) {
	//	RenderMesh robot = assets_[ROBOT];
	//	glm::mat4 init_model = glm::translate(glm::mat4(1.f), glm::vec3(gen(rng), gen(rng), 0.f));
	//	update_model(robot, init_model);
	//	default_scene_.push_back(robot);
	//}

	load_interior_model();
	create_robots();
}

void SwarmViewer::initialize_position() {

	std::random_device rd;
	std::mt19937 rng(rd());
	std::uniform_int_distribution<int> gen(0, 100); // uniform, unbiased
	int r = gen(rng);
	
	for (int i = 0; i < default_scene_.size(); ++i) {
		default_scene_.push_back(assets_[ROBOT]);
	}
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

	default_scene_.push_back(interior_model_mesh);

	create_lights();
}

void SwarmViewer::custom_init_code() {
	timer_ = new QTimer(this);
	connect(timer_, SIGNAL(timeout()), this, SLOT(update()));
	timer_->start(20);
}

void SwarmViewer::custom_draw_code() {
	glm::mat4 mvp;

	draw_scene(default_scene_);

	for (auto& robot : robots_) {
		robot.move();
		draw_mesh(robot.mesh_);
	}
}

void SwarmViewer::draw_mesh(RenderMesh& mesh) {

	GLint model_loc = m_shader.uniformLocation("model");
	GLint inverse_transpose_loc = m_shader.uniformLocation("inverse_transpose_model");
	GLint mvp_loc = m_shader.uniformLocation("mvp");

	GLint view_position_loc = m_shader.uniformLocation("view_position");
	glUniform3fv(view_position_loc, 1, glm::value_ptr(eye_));


	for (auto& entity : mesh) {
		// model
		glm::mat4 transformed_model = model_ * entity.model_;
		glUniformMatrix4fv(model_loc, 1, GL_FALSE, glm::value_ptr(transformed_model));

		// normal matrix
		glm::mat3 inverse_transpose_model = glm::transpose(glm::inverse(glm::mat3(transformed_model)));
		glUniformMatrix3fv(inverse_transpose_loc, 1, GL_FALSE, glm::value_ptr(inverse_transpose_model));


		//glm::mat4 mvp_matrix = projection_ * camera_ * transformed_model;
		glm::mat4 mvp_matrix = projection_ * camera_ * transformed_model;
		glUniformMatrix4fv(mvp_loc, 1, GL_FALSE, glm::value_ptr(mvp_matrix));


		entity.draw();
	}
}

SwarmViewer::SwarmViewer(const QGLFormat& format, QWidget* parent) : RobotViewer(format, parent) {
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

void SwarmViewer::create_robot_model(RenderMesh& robot_mesh) {
	VertexBufferData vb_data;
	load_obj("PYROS/pyros-obj.obj", vb_data);

	cv::Vec4f color(253.f, 184.f, 19.f, 255.f);
	color /= 255.f;
	for (auto& vertice : vb_data.positions) {
		vb_data.colors.push_back(color);
	}

	RenderEntity robot_model(GL_TRIANGLES, &m_shader);
	robot_model.upload_data_to_gpu(vb_data);
	robot_model.set_model(glm::scale(glm::mat4(1.f), glm::vec3(.1f, .1f, .1f)));

	//robot_model.set_model(glm::scale(glm::mat4(1.f), glm::vec3(10.f, 10.f, 10.f)));
	robot_mesh.push_back(robot_model);
}

void SwarmViewer::create_lights() {

	std::random_device rd;
	std::mt19937 rng(rd());
	std::uniform_real_distribution<float> color(0.5, 1); // uniform, unbiased
	std::uniform_int_distribution<int> position(0, 500);

	m_shader.bind();

	RenderMesh light_bulb;
	create_light_model(light_bulb);

	for (size_t i = 0; i < NO_OF_LIGHTS; ++i) {
		std::stringstream light_name;
		light_name << "lights[" << i << "]";

		std::stringstream light_position_name;
		light_position_name << light_name.str() << ".position";
		GLint light_position_loc = m_shader.uniformLocation(light_position_name.str().c_str());

		//glm::vec3 light_position(0, 0, -100);
		glm::vec3 light_position(position(rng), position(rng), position(rng));
		glUniform3fv(light_position_loc, 1, glm::value_ptr(light_position));

		glm::mat4 RT = glm::translate(glm::mat4(1.f), light_position);
		RenderMesh light = light_bulb;
		update_model(light, RT);
		default_scene_.push_back(light);
		 
		GLenum error = glGetError();

		std::stringstream light_color;
		light_color << light_name.str() << ".color";
		GLint light_color_loc = m_shader.uniformLocation(light_color.str().c_str());
		glUniform3fv(light_color_loc, 1, glm::value_ptr(glm::vec3(color(rng), color(rng), color(rng))));
	}
}


void SwarmViewer::create_robots() {


	m_shader.bind();

	RenderMesh robot_original_mesh;
	create_robot_model(robot_original_mesh);

	for (size_t i = 0; i < NO_OF_ROBOTS; ++i) {

		RenderMesh robot_mesh_copy = robot_original_mesh;

		Robot robot;
		robot.mesh_ = robot_mesh_copy;

		robots_.push_back(robot);
	}
}
