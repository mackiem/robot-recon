#pragma once


#include "gl_core_3_3.h"
#include <QGLWidget>

#include <QGLBuffer>
#include <QGLShaderProgram>
#include "FlyCapture2.h"

using namespace FlyCapture2;


class GLWidget : public QGLWidget
{
	Q_OBJECT

protected:
	virtual void initializeGL();
	virtual void resizeGL(int w, int h);
	virtual void paintGL();

	//virtual void keyPressEvent(QKeyEvent* e);

private:
	bool prepareShaderProgram(const QString& vertexShaderPath,
		const QString& fragmentShaderPath);

	QGLShaderProgram m_shader;
	QGLBuffer m_vertexBuffer;

	GLuint *tex;
	GLuint vao;

	int no_of_cams_;
	float threshold_;
	int is_thresholding_on_;

public slots:
void display_image(FlyCapture2::Image img, int cam_no);
void set_threshold(int value);
void toggle_thresholding(int value);

public:
	GLWidget(const int no_of_cams, const QGLFormat& format, QWidget* parent = 0);
	virtual ~GLWidget();
	QSize sizeHint() const;
	void set_no_of_cams(int no_of_cams);
};