#pragma once
#include "gl_core_3_3.h"
#include <QGLWidget>

#include <QGLBuffer>
#include <QGLShaderProgram>
#include "fsl_common.h"
#include <QTimer>
#include <QWheelEvent>
#include <opencv2/video/background_segm.hpp>

class RenderEntity {

private:
	GLuint vao_;
	GLuint vbo_[2];
	GLenum primitive_;
	GLint count_;
	QGLShaderProgram &shader_;

public:
	RenderEntity(GLenum primitive, QGLShaderProgram& shader);

	void upload_data_to_gpu(std::vector<cv::Vec3f>& vertices, std::vector<cv::Vec3f>& colors);
	void draw();
};

class RobotViewer : public QGLWidget
{
	Q_OBJECT

protected:
	virtual void initializeGL();
	virtual void paintGL();
	virtual void resizeGL(int w, int h);
	virtual void wheelEvent(QWheelEvent *event);
	void mouseMoveEvent(QMouseEvent* event);
	void mouseReleaseEvent(QMouseEvent* event);
	void keyPressEvent(QKeyEvent *event);


private:
	bool prepareShaderProgram(const QString& vertexShaderPath,
		const QString& fragmentShaderPath);

	QGLShaderProgram m_shader;
	QGLBuffer m_vertexBuffer;

	//GLuint *tex;
	GLuint vao_;
	GLuint vbo_[3];
	int angle_;
	int no_of_pts_;
	QTimer timer_;
	float zoom_;
	GLuint texture_id_;
	bool is_draw_triangles_;
	GLint active_texture_;
	bool is_texture_on_;
	float camera_distance_;
	bool mouse_down_;
	float look_at_x_;
	float look_at_y_;
	GLuint vao_pts_;
	GLuint vbo_pts_[3];
	GLint no_of_triangles_;
	float scale_;
	void gen_texture(GLuint& texture_id, cv::Mat& remapped_img_for_texture);

	float m_xRot;
	float m_yRot;
	float m_zRot;
	QPoint m_lastPos;

	void setXRotation(float angle);
	void setYRotation(float angle);
	void setZRotation(float angle);

	std::vector<RenderEntity> entities_;


public:
	RobotViewer(const QGLFormat& format, QWidget* parent = 0);
	virtual ~RobotViewer(void);
	QSize sizeHint() const override;
	void change_world_pts(WPt& world_pts);

	public slots:
	void create_plane_with_points_and_lines(std::vector<cv::Vec3f> points_3d,
		cv::Vec3f line_a, cv::Vec3f line_b, cv::Vec3f normal, double d);
	void create_line(cv::Vec3f a, cv::Vec3f b, cv::Vec3f line_color);
	void create_plane(cv::Vec3f normal, double d, cv::Vec3f plane_color);
	void create_points(std::vector<cv::Vec3f> points_3d, cv::Vec3f point_color);


	void update_lines_3d(WPts world_pts);
	void update_model_with_triangles(WPts world_pts, WPts world_pt_colors, WPt triangles, IPt texture_coords, cv::Mat texture_img);
	void draw_triangles();
	void draw_points();
	void draw_colors();
	void draw_texture();
	void reset_view();
	void set_scale(int value);
	//void set_camera_pair();
};

