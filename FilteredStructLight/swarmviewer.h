#pragma once
#include "robotviewer.h"
#include <random>
#include "robot.h"
#include <cmath>
#include <memory>


class SwarmViewer : public RobotViewer {

struct Light : public VisObject {

protected:

public:
	explicit Light(UniformLocations& locations)
		: VisObject(locations) {
	}
	int light_color_location_;
	int light_position_location_;
	glm::vec3 position_;
	glm::vec3 color_;
	virtual void update(glm::mat4 global_model);
};

struct GridOverlay : public VisObject {
	std::shared_ptr<SwarmOctTree> octree_;
	unsigned int grid_resolution_per_side_;
	float grid_length_;
	QGLShaderProgram* shader_;
	std::map<int, cv::Vec4f> robot_color_map_;
	GridOverlay(UniformLocations& locations, std::shared_ptr<SwarmOctTree> octree, unsigned int grid_resolution, 
		float grid_length, std::map<int, cv::Vec4f> robot_color_map, QGLShaderProgram* shader);

	void create_mesh(bool initialize);
	void update(glm::mat4 global_model) override;
};

private:
	std::string interior_model_filename_;
	GLint model_loc_;
	GLint inverse_transpose_loc_;
	GLint mvp_loc_;
	GLint view_position_loc_;
	int grid_resolution_per_side_;
	static const std::string DEFAULT_INTERIOR_MODEL_FILENAME;
	static const int DEFAULT_NO_OF_ROBOTS;
	static const std::string OCCUPANCY_GRID_NAME;
	static const std::string OCCUPANCY_GRID_OVERLAY_NAME;
	std::vector<std::shared_ptr<Robot>> robots_;
	std::vector<Light> lights_;
	QTimer* timer_;
	UniformLocations uniform_locations_;

	std::vector<std::shared_ptr<VisObject>> reset_vis_objects_;
	
	std::vector<std::shared_ptr<VisObject>> default_vis_objects_;

	std::map<int, cv::Vec4f> robot_color_map_;


	int no_of_robots_;
	float explore_constant_;
	float separation_constant_;
	float goto_work_constant_;

	float separation_distance_;
	
	//robots_spinbox_->setValue(settings.value(ROBOTS_NO_LABEL, "10").toInt());
	//exploration_constant_->setValue(settings.value(EXPLORATION_CONSTANT_LABEL, "1").toDouble());
	//separation_constant_->setValue(settings.value(SEPERATION_CONSTANT_LABEL, "1").toDouble());
	//goto_work_constant_->setValue(settings.value(GOTO_WORK_CONSTANT_LABEL, "1").toDouble());

	unsigned int grid_resolution_;
	float grid_length_;
	//grid_resolution_spin_box_->setValue(settings.value(GRID_RESOLUTION_LABEL, "4096").toInt());
	//grid_length_spin_box_->setValue(settings.value(GRID_LENGTH_LABEL, "20").toInt());

	float interior_scale_;
	glm::vec3 interior_offset_;
	bool show_interior_;
	//scale_spinbox_->setValue(settings.value(BUILDING_INTERIOR_SCALE_LABEL, "2").toInt());
	//x_spin_box_->setValue(settings.value(BUILDING_OFFSET_X_LABEL, "0").toInt());
	//y_spin_box_->setValue(settings.value(BUILDING_OFFSET_Y_LABEL, "0").toInt());
	//z_spin_box_->setValue(settings.value(BUILDING_OFFSET_Z_LABEL, "0").toInt());

	//show_interior_->setCheckState(show_interior);
	std::shared_ptr<SwarmOctTree> octree_;
	QGLShaderProgram line_shader_;

	//std::vector<VisObject> default_objects_;
	//VisObject grid_;
	//VisObject grid_overlay_;

protected:
	void load_inital_models() override;
	void initialize_position();
	virtual void set_shaders() override;
	void load_interior_model();
	virtual void custom_init_code() override;
	virtual void custom_draw_code() override;
	virtual void draw_mesh(RenderMesh& mesh) override;

	
public:
	SwarmViewer(const QGLFormat& format, QWidget* parent = 0);
	void create_light_model(RenderMesh& light_mesh);
	void create_robot_model(RenderMesh& light_mesh, cv::Vec4f color);
	void create_lights();
	void create_robots();
	//void create_occupancy_grid_overlay(int grid_resolution, int grid_size, bool initialize = false);
	void create_occupancy_grid(int grid_resolution, int grid_size);

public slots:
void set_no_of_robots(int no_of_robots);

void set_separation_distance(float distance);

void set_grid_resolution(int grid_resolution);
void set_grid_length(int grid_length);

void set_interior_scale(int scale);
void set_interior_offset(glm::vec3 offset);

void set_exploration_constant(double constant);
void set_separation_constant(double constant);
void set_goto_work_constant(double constant);

void set_show_interior(int show);

void reset_sim();
};
