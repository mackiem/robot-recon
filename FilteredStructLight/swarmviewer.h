#pragma once
#include "robotviewer.h"
#include <random>


class SwarmViewer : public RobotViewer
{

struct Robot {
	RenderMesh mesh_;
	glm::vec3 velocity_;
	glm::vec3 position_;
	long long timeout_;
	long long last_timeout_;
	long long last_updated_time_;
	std::mt19937 rng_;
	std::uniform_real_distribution<float> velocity_generator_;
	std::uniform_int_distribution<int> position_generator_;

	Robot();
	void set_velocity(glm::vec3 velocity);
	void update_velocity();
	void move();
	void draw();
};

private:
	int no_of_robots_;
	std::string interior_model_filename_;
	static const std::string DEFAULT_INTERIOR_MODEL_FILENAME;
	static const int DEFAULT_NO_OF_ROBOTS;
	std::vector<Robot> robots_;
	QTimer* timer_;

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
};
