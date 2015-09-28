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

private:
	bool prepareShaderProgram(const QString& vertexShaderPath,
		const QString& fragmentShaderPath);

	QGLShaderProgram m_shader;
	QGLBuffer m_vertexBuffer;

	//GLuint *tex;
	GLuint vao_;
	GLuint vbo_[2];
	int angle_;
	int no_of_pts_;
	QTimer timer_;
	float zoom_;
public:
	ModelViewer(const QGLFormat& format, QWidget* parent = 0);
	virtual ~ModelViewer(void);
	QSize sizeHint() const override;

	public slots:
	void change_world_pts(WPts& world_pts);
	void update_model(WPts world_pts);
};

