#pragma once
#include "robotviewer.h"
#include <random>
#include "robot.h"
#include <cmath>


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

private:
	int no_of_robots_;
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
	std::vector<Robot> robots_;
	std::vector<Light> lights_;
	QTimer* timer_;
	UniformLocations uniform_locations_;

	std::vector<VisObject> default_vis_objects_;

	unsigned int grid_resolution_;
	float grid_size_;

	SwarmOctTree octree_;
	QGLShaderProgram line_shader_;

	VisObject grid_;
	VisObject grid_overlay_;

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
	void create_robot_model(RenderMesh& light_mesh);
	void create_lights();
	void create_robots();
	void create_occupancy_grid_overlay(int grid_resolution, int grid_size, bool initialize = false);
	void create_occupancy_grid(int grid_resolution, int grid_size);
};
