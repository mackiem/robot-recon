#pragma once
#include "gl_core_3_3.h"
#include <QGLWidget>

#include <QGLBuffer>
#include <QGLShaderProgram>
#include "fsl_common.h"
#include <QTimer>
#include <QWheelEvent>

class ModelViewer : public QGLWidget
{
	Q_OBJECT

protected:
	virtual void initializeGL();
	virtual void paintGL();
	virtual void resizeGL(int w, int h);
	virtual void wheelEvent(QWheelEvent *event);
	void mouseMoveEvent(QMouseEvent* event);

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
	void gen_texture(GLuint& texture_id, cv::Mat& remapped_img_for_texture);

	float m_xRot;
	float m_yRot;
	float m_zRot;
	QPoint m_lastPos;
	void setXRotation(float angle);
	void setYRotation(float angle);
	void setZRotation(float angle);


public:
	ModelViewer(const QGLFormat& format, QWidget* parent = 0);
	virtual ~ModelViewer(void);
	QSize sizeHint() const override;
	void change_world_pts(WPts& world_pts);

	public slots:
	void update_model(WPts world_pts);
	void update_model_with_triangles(WPts triangles, IPts texture_coords, cv::Mat texture_img);
	void draw_triangles();
	void draw_points();
	void draw_colors();
	void draw_texture();
	//void set_camera_pair();
};

