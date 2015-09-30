#include "cameradisplaywidget.h"

#include <glm/glm.hpp>
#include <glm/vec3.hpp> // glm::vec3
#include <glm/vec4.hpp> // glm::vec4
#include <glm/mat4x4.hpp> // glm::mat4
#include <glm/gtc/matrix_transform.hpp> // glm::translate, glm::rotate, glm::scale, glm::perspective
#include <glm/gtc/type_ptr.hpp>
#include <iostream>
#include <opencv2/opencv.hpp>

GLWidget::GLWidget(const int no_of_cams, const QGLFormat& format, QWidget* parent)
	: no_of_cams_(no_of_cams), QGLWidget(format, parent),
	m_vertexBuffer(QGLBuffer::VertexBuffer), threshold_(0.075), is_thresholding_on_(0)
{
}

GLWidget::~GLWidget() {
	if (tex) {
		delete[] tex;
	}
}

void GLWidget::initializeGL()
{
	QGLFormat glFormat = QGLWidget::format();
	if (!glFormat.sampleBuffers())
		qWarning() << "Could not enable sample buffers";

	// Set the clear color to black
	glClearColor(0.0f, 0.0f, 0.0f, 1.0f);

	glGenVertexArrays(1, &vao);
	glBindVertexArray(vao);

	// Prepare a complete shader program?
	if (!prepareShaderProgram(":/FilteredStructLight/camera_vert.glsl", ":/FilteredStructLight/camera_frag.glsl"))
		return;

	// We need us some vertex data. Start simple with a triangle ;-)
	float points[] = { 
		-1.0f, -1.0f, 0.0f, 1.0f, 0.f, 1.f,
		1.0f, -1.0f, 0.0f, 1.0f, 1.f, 1.f,
		1.0f, 1.0f, 0.0f, 1.0f, 1.f, 0.f,

		1.0f, 1.0f, 0.0f, 1.0f, 1.f, 0.f,
		-1.0f, 1.f, 0.0f, 1.0f, 0.0f, 0.0f,
		-1.0f, -1.0f, 0.0f, 1.0f, 0.f, 1.f,
	};


	// Create vertex buffer

	GLuint vbo;
	glGenBuffers(1, &vbo);
	glBindBuffer(GL_ARRAY_BUFFER, vbo);
	glBufferData(GL_ARRAY_BUFFER, sizeof(points), points, GL_STATIC_DRAW);

	if (!m_shader.bind())
	{
		qWarning() << "Could not bind shader program to context";
		return;
	}

	// Specify vertex format
	//GLuint imgAttrPos = glGetAttribLocation(m_shader., "pos");
	//GLuint imgAttrTc = glGetAttribLocation(imgShader, "tc");

	GLuint imgAttrPos = m_shader.attributeLocation("vertex");
	GLuint imgAttrTc = m_shader.attributeLocation("vertCoord");

	glEnableVertexAttribArray(imgAttrPos);
	glVertexAttribPointer(imgAttrPos, 4, GL_FLOAT, GL_FALSE, sizeof(GLfloat) * 6 , NULL);
	glEnableVertexAttribArray(imgAttrTc);
	glVertexAttribPointer(imgAttrTc, 2, GL_FLOAT, GL_FALSE, sizeof(GLfloat) * 6, (GLvoid*)(sizeof(GLfloat) * 4));

	GLuint tex_location = m_shader.uniformLocation("tex");
	glUniform1i(tex_location, 0);

	// gen texture
	tex = new GLuint[no_of_cams_];
	glGenTextures(no_of_cams_, tex);

	for (int i = 0; i < no_of_cams_; ++i) {
		glBindTexture(GL_TEXTURE_2D, tex[i]);
		//glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA32F, 1, 1, 0, GL_RGBA, GL_FLOAT, NULL); 
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
		glBindTexture(GL_TEXTURE_2D, NULL);
	}

	// Cleanup state
	glBindVertexArray(NULL);
	glBindBuffer(GL_ARRAY_BUFFER, NULL);

	//m_shader.

	//m_vertexBuffer.create();
	//m_vertexBuffer.setUsagePattern(QGLBuffer::StaticDraw);
	//if (!m_vertexBuffer.bind())
	//{
	//	qWarning() << "Could not bind vertex buffer to the context";
	//	return;
	//}
	//m_vertexBuffer.allocate(points, 3 * 4 * sizeof(float));

	//// Bind the shader program so that we can associate variables from
	//// our application to the shaders
	//if (!m_shader.bind())
	//{
	//	qWarning() << "Could not bind shader program to context";
	//	return;
	//}

	//// Enable the "vertex" attribute to bind it to our currently bound
	//// vertex buffer.
	//m_shader.setAttributeBuffer("vertex", GL_FLOAT, 0, 4);
	//m_shader.enableAttributeArray("vertex");
}

bool GLWidget::prepareShaderProgram(const QString& vertexShaderPath,
	const QString& fragmentShaderPath)
{
	// First we load and compile the vertex shader?
	bool result = m_shader.addShaderFromSourceFile(QGLShader::Vertex, vertexShaderPath);
	if (!result)
		qWarning() << m_shader.log();

	// …now the fragment shader?
	result = m_shader.addShaderFromSourceFile(QGLShader::Fragment, fragmentShaderPath);
	if (!result)
		qWarning() << m_shader.log();

	// …and finally we link them to resolve any references.
	result = m_shader.link();
	if (!result)
		qWarning() << "Could not link shader program:" << m_shader.log();

	return result;
}

void GLWidget::resizeGL(int w, int h)
{
	// Set the viewport to window dimensions
	glViewport(0, 0, w, qMax(h, 1));
}

void GLWidget::paintGL()
{
	// Clear the buffer with the current clearing color
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	if (!m_shader.bind())
	{
		qWarning() << "Could not bind shader program to context";
		return;
	}

	for (int i = 0; i < no_of_cams_; ++i) {
		glBindVertexArray(vao);
		glActiveTexture(GL_TEXTURE0 + 0);
		glBindTexture(GL_TEXTURE_2D, tex[i]);

		GLuint model_loc = m_shader.uniformLocation("model");
		float translate_amount = 1.f;
		float translate_x = (i % 2 == 0) ? -1.f * translate_amount : translate_amount;
		float translate_y = (i < (no_of_cams_ / 2)) ? translate_amount : -1.f * translate_amount;
		//float translate_x = translate_amount;
		//float translate_y = translate_amount;
		
		//std::cout << translate_x << " " << translate_y << std::endl;

		glm::mat4 model = glm::translate(glm::scale(glm::mat4(1.f), glm::vec3(0.5f, 0.5f, 1.f)), glm::vec3(translate_x, translate_y, 0.f));

		GLuint threshold_location = m_shader.uniformLocation("threshold");

		GLuint toggle_threshold_location = m_shader.uniformLocation("toggle_threshold");

		glUniformMatrix4fv(model_loc, 1, GL_FALSE, glm::value_ptr(model));

		glUniform1i(toggle_threshold_location, is_thresholding_on_);

		glUniform1f(threshold_location, threshold_);

		glDrawArrays(GL_TRIANGLES, 0, 6);
	}


	// Draw stuff
	glBindVertexArray(NULL);
}

QSize GLWidget::sizeHint() const {
	return QSize(768, 768);
}


void GLWidget::set_no_of_cams(int no_of_cams) {
	no_of_cams_ = no_of_cams;
}

void GLWidget::display_image(Image image, int cam_no) {

	makeCurrent();
	cv::Mat test(image.GetRows(), image.GetCols(), CV_8UC1, image.GetData(), image.GetStride());
	cv::Mat test2;
	//cv::cvtColor(test, colored, CV_BayerBG2BGR);
	cv::cvtColor(test, test2, CV_BayerBG2GRAY);
	// Set stride for unpacking pixels
	glPixelStorei(GL_UNPACK_ROW_LENGTH, image.GetStride());
	// Replace current texture with new image
	glBindTexture(GL_TEXTURE_2D, tex[cam_no]);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RED, image.GetCols(), image.GetRows(), 0,
		GL_RED, GL_UNSIGNED_BYTE, test2.ptr());
	FlyCapture2::PixelFormat format = image.GetPixelFormat();
	glBindTexture(GL_TEXTURE_2D, NULL);

	//cv::Mat test(image.GetRows(), image.GetCols(), CV_8UC1, image.GetData(), image.GetStride());

	//std::string img_filename = std::string("test_") + std::to_string(cam_no) + std::string(".png");
	//cv::imwrite(img_filename, test);
	

	if (cam_no == no_of_cams_ - 1) {
		//glViewport(0, 0, width(), qMax(height(), 1));
		updateGL();
	}

}

void GLWidget::set_threshold(int value) {
	threshold_ = (float)(value) / (float)(1000);
}

void GLWidget::toggle_thresholding(int value) {
	is_thresholding_on_ = (value == 0) ? 0 : 1;
}
