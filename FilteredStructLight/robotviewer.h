#pragma once

#include "renderentity.h"
#include "fsl_common.h"
#include <QTimer>
#include <QWheelEvent>
#include <opencv2/video/background_segm.hpp>
#include <unordered_map>



class RobotViewer : public QGLWidget
{
	Q_OBJECT


protected:


	virtual void initializeGL();
	virtual void paintGL();
	virtual void resizeGL(int w, int h);
	virtual void wheelEvent(QWheelEvent *event);
	virtual void draw_mesh(RenderMesh& mesh);
	virtual void draw_scene(Scene& scene);
	void mouseMoveEvent(QMouseEvent* event);
	void mouseReleaseEvent(QMouseEvent* event);
	void keyPressEvent(QKeyEvent *event);

	virtual void load_inital_models();
	virtual void set_shaders();
	virtual void init_camera_params();
	virtual void custom_init_code();
	virtual void update_camera();
	virtual void custom_draw_code();

	void set_shader_paths(const char* vertex_shader_path, const char* fragment_shader_path);

	// names of elements
	const static std::string CAMERA;
	const static std::string IMAGE_PLANE;
	const static std::string ROBOT;
	const static std::string FLOOR_GRID;

	Assets assets_;
	Scene default_scene_;
	QGLShaderProgram m_shader;
	const char* vertex_shader_path_;
	const char* fragment_shader_path_;
	glm::mat4 camera_;
	glm::mat4 projection_;
	glm::mat4 model_;

	glm::vec3 up_;
	glm::vec3 eye_;
	glm::vec3 center_;

	float look_at_x_;
	float look_at_y_;
	float look_at_z_;

	float fovy_;
	float camera_distance_;

	bool prepareShaderProgram(const QString& vertexShaderPath,
		const QString& fragmentShaderPath);
	bool prepareShaderProgram(const QString& vertexShaderPath, const QString& fragmentShaderPath, QGLShaderProgram& shader);

private:

	QGLBuffer m_vertexBuffer;

	//GLuint *tex;
	GLuint vao_;
	GLuint vbo_[3];
	int angle_;
	int no_of_pts_;
	QTimer timer_;
	GLuint texture_id_;
	bool is_draw_triangles_;
	GLint active_texture_;
	bool is_texture_on_;
	bool mouse_down_;
	GLuint vao_pts_;
	GLuint vbo_pts_[3];
	GLint no_of_triangles_;
	float scale_;
	bool draw_points_;
	bool draw_planes_;
	bool draw_lines_;
	bool draw_default_;
	glm::vec3 point_centroid_;
	std::vector<cv::Vec3f> points_;
	void gen_texture(GLuint& texture_id, cv::Mat& remapped_img_for_texture);

	float m_xRot;
	float m_yRot;
	float m_zRot;
	QPoint m_lastPos;

	void setXRotation(float angle);
	void setYRotation(float angle);
	void setZRotation(float angle);

	std::vector<RenderEntity> entities_;

	bool draw_frame_by_frame_;
	std::vector<std::vector<RenderMesh>> frames_;
	int current_frame_to_draw_;

public:
	RobotViewer(const QGLFormat& format, QWidget* parent = 0);
	virtual ~RobotViewer(void);
	QSize sizeHint() const override;
	void change_world_pts(WPt& world_pts);

	void load_obj(std::string filename, std::vector<cv::Vec3f>& vertices, std::vector<cv::Vec3f>& normals);
	void load_obj(std::string filename, std::vector<cv::Vec3f>& vertices,
		std::vector<cv::Vec3f>& normals, std::vector<cv::Vec2f>& uvs, std::vector<unsigned int>& indices,
		std::vector<int>& count, std::vector<int>& offset, std::vector<int>& base_index);
	void load_obj(std::string filename, VertexBufferData& vertex_buffer_data);
	void draw_camera(cv::Mat& model);
	void draw_table_top(cv::Mat& model);
	void draw_robot(cv::Mat& model);

	void clear_models();
	glm::mat4 convert(cv::Mat& mat);

	RenderEntity get_line_entity(cv::Vec3f a, cv::Vec3f b, cv::Vec4f line_color, 
		cv::Mat& model, RenderEntity::Type type = RenderEntity::Lines);
	RenderEntity get_line_entity(cv::Vec3f a, cv::Vec3f b, cv::Vec4f line_color, 
		 RenderEntity::Type type = RenderEntity::Lines);
	RenderEntity get_plane_entity(cv::Vec3f normal, double d, cv::Vec4f plane_color);
	RenderEntity get_points_entity(std::vector<cv::Vec3f> points_3d, cv::Vec4f point_color);

	void update_model(RenderMesh& mesh, cv::Mat RT);
	void update_model(RenderMesh& mesh, glm::mat4 RT);


	public slots:
	void create_plane_with_points_and_lines(std::vector<cv::Vec3f> points_3d,
		cv::Vec3f line_a, cv::Vec3f line_b, cv::Vec3f normal, double d);
	void create_line(cv::Vec3f a, cv::Vec3f b, cv::Vec4f line_color, cv::Mat& model, RenderEntity::Type type = RenderEntity::Lines);
	void create_plane(cv::Vec3f normal, double d, cv::Vec4f plane_color);
	void create_points(std::vector<cv::Vec3f> points_3d, cv::Vec4f point_color);
	void toggle_draw_points(int check_state);
	void toggle_draw_planes(int check_state);
	void toggle_draw_lines(int check_state);
	void toggle_draw_default(int check_state);
	void toggle_frame_by_frame(int check_state);
	void update_lines_3d(WPts world_pts);
	void update_model_with_triangles(WPts world_pts, 
		WPts world_pt_colors, WPt triangles, IPt texture_coords, cv::Mat texture_img);
	void draw_triangles();
	void draw_points();
	void draw_colors();
	void draw_texture();
	void reset_view();
	void center_view();
	void top_view();
	void set_scale(int value);


	void start_reconstruction_sequence();
	void end_reconstruction_sequence();

	void create_reconstruction_frame(std::vector<cv::Vec3f> points_3d,
		cv::Vec3f line_a, cv::Vec3f line_b, cv::Vec3f normal, double d, cv::Mat RT);
	void set_current_frame_to_draw(int frame_no);


	void create_calibration_frame(std::vector<cv::Vec3f> points_3d,
		cv::Vec3f line_a, cv::Vec3f line_b, cv::Vec3f normal, double d);

	void create_final_calibration_frame(cv::Vec3f normal, double d);
	//void set_camera_pair();
};

