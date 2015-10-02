#include "modelviewer.h"
#include <glm/glm.hpp>
#include <glm/vec3.hpp> // glm::vec3
#include <glm/vec4.hpp> // glm::vec4
#include <glm/mat4x4.hpp> // glm::mat4
#include <glm/gtc/matrix_transform.hpp> // glm::translate, glm::rotate, glm::scale, glm::perspective
#include <glm/gtc/type_ptr.hpp>



ModelViewer::ModelViewer(const QGLFormat& format, QWidget* parent)
	:  QGLWidget(format, parent),
	m_vertexBuffer(QGLBuffer::VertexBuffer), no_of_pts_(0), angle_(0)
{

	// Configure the timer_
    connect(&timer_, SIGNAL(timeout()), this, SLOT(updateGL()));
    if(this->format().swapInterval() == -1)
    {
        // V_blank synchronization not available (tearing likely to happen)
        qDebug("Swap Buffers at v_blank not available: refresh at approx 60fps.");
        timer_.setInterval(17);
    }
    else
    {
        // V_blank synchronization available
        timer_.setInterval(0);
    }
    timer_.start();
}

ModelViewer::~ModelViewer() {
	//if (tex) {
	//	delete[] tex;
	//}
}

void ModelViewer::initializeGL()
{
	QGLFormat glFormat = QGLWidget::format();
	if (!glFormat.sampleBuffers())
		qWarning() << "Could not enable sample buffers";

	// Set the clear color to black
	glClearColor(0.0f, 0.0f, 0.0f, 1.0f);

	glGenVertexArrays(1, &vao_);
	glBindVertexArray(vao_);

	// Prepare a complete shader program?
	if (!prepareShaderProgram(":/FilteredStructLight/model_vert.glsl", ":/FilteredStructLight/model_frag.glsl"))
		return;


	// create model and view p


	// Create vertex buffer
	glGenBuffers(2, vbo_);


	if (!m_shader.bind())
	{
		qWarning() << "Could not bind shader program to context";
		return;
	}

	zoom_ = 50.f;

	glm::mat4 projection = glm::perspective(zoom_, static_cast<float>(sizeHint().width())/static_cast<float>(sizeHint().height()), 0.1f, 1000.0f);

	glm::mat4 camera = glm::lookAt(glm::vec3(0,0,100), glm::vec3(0,0,0), glm::vec3(0,1,0));

	GLint projection_location = m_shader.uniformLocation("projection");
	glUniformMatrix4fv(projection_location, 1, GL_FALSE, glm::value_ptr(projection));

	GLint camera_location = m_shader.uniformLocation("camera");
	glUniformMatrix4fv(camera_location, 1, GL_FALSE, glm::value_ptr(camera));

	glEnable(GL_PROGRAM_POINT_SIZE);
	glPointSize(5.0);
	//GLuint tex_location = m_shader.uniformLocation("tex");
	//glUniform1i(tex_location, 0);

	//// gen texture
	//tex = new GLuint[no_of_cams_];
	//glGenTextures(no_of_cams_, tex);

	//for (int i = 0; i < no_of_cams_; ++i) {
	//	glBindTexture(GL_TEXTURE_2D, tex[i]);
	//	//glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA32F, 1, 1, 0, GL_RGBA, GL_FLOAT, NULL); 
	//	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
	//	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	//	glBindTexture(GL_TEXTURE_2D, NULL);
	//}

	// Cleanup state
	glBindVertexArray(NULL);
	glBindBuffer(GL_ARRAY_BUFFER, NULL);

	WPts test_pts;
	for (int x = -1; x <= 1; ++x) {
		for (int y = -1; y <= 1; ++y) {
			for (int z = -1; z <= 1; ++z) {
				test_pts.push_back(cv::Vec3f(x, y, z));
			}
		}
	}
	update_model(test_pts);
}

bool ModelViewer::prepareShaderProgram(const QString& vertexShaderPath,
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

void ModelViewer::resizeGL(int w, int h)
{
	// Set the viewport to window dimensions
	glViewport(0, 0, w, qMax(h, 1));
}

void ModelViewer::wheelEvent(QWheelEvent* event) {
	  QPoint numPixels = event->pixelDelta();
    QPoint numDegrees = event->angleDelta() / 8;

	if (!numDegrees.isNull()) {
		zoom_ += (float)(numDegrees.y()) / (float)(15 * 4);
	}
}

void ModelViewer::paintGL()
{
	// Clear the buffer with the current clearing color
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	glBindVertexArray(vao_);
	//glActiveTexture(GL_TEXTURE0 + 0);
	//glBindTexture(GL_TEXTURE_2D, tex[i]);

	if (!m_shader.bind())
	{
		qWarning() << "Could not bind shader program to context";
		return;
	}

	GLuint model_loc = m_shader.uniformLocation("model");

	//std::cout << translate_x << " " << translate_y << std::endl;

	glm::mat4 model = glm::rotate(glm::scale(glm::mat4(1.f), glm::vec3(0.1, 0.1, 0.1)), 0.1f * static_cast<float>(angle_++), glm::vec3(0.f, 1.f, 0.f));

	glUniformMatrix4fv(model_loc, 1, GL_FALSE, glm::value_ptr(model));

	glm::mat4 projection = glm::perspective(zoom_, static_cast<float>(size().width())/static_cast<float>(size().height()), 0.1f, 1000.0f);
	float ratio = static_cast<float>(size().width())/static_cast<float>(size().height());
	GLint projection_location = m_shader.uniformLocation("projection");
	glUniformMatrix4fv(projection_location, 1, GL_FALSE, glm::value_ptr(projection));

	glDrawArrays(GL_POINTS, 0, no_of_pts_);


	// Draw stuff
	glBindVertexArray(NULL);
}

QSize ModelViewer::sizeHint() const {
	return QSize(768, 768);
}

void ModelViewer::change_world_pts(WPts& world_pts) {
	
	cv::Vec3f distance(0.f);
	for (auto i = 0u; i < world_pts.size(); ++i) {
		distance += cv::Vec3f(0.f) - cv::Vec3f(world_pts[i]);
	}
	cv::Vec3f avg_distance = distance / (float)(world_pts.size());

	// Copy all scene points into a local array
	WPts pts(world_pts.size());

	for (int p = 0; p < world_pts.size(); p++) {
		pts[p] = world_pts[p] + avg_distance;
	}

	world_pts = pts;
}

void ModelViewer::update_model(WPts world_pts) {
	makeCurrent();

	if (world_pts.size() < 1) {
		std::cout << "No points to display" << std::endl;
		return;
	}

	WPts color_pts;
	cv::Vec3f white(1.f, 1.f, 1.f);
	for (auto i = 0u; i < world_pts.size(); ++i) {
		color_pts.push_back(white);
	}

	change_world_pts(world_pts);


	if (!m_shader.bind())
	{
		qWarning() << "Could not bind shader program to context";
		return;
	}

	glBindVertexArray(vao_);

	glBindBuffer(GL_ARRAY_BUFFER, vbo_[0]);
	glBufferData(GL_ARRAY_BUFFER, world_pts.size() * sizeof(cv::Vec3f), &world_pts[0], GL_STATIC_DRAW);

	GLuint vert_pos_attr = m_shader.attributeLocation("vertex");

	glEnableVertexAttribArray(vert_pos_attr);
	glVertexAttribPointer(vert_pos_attr, 3, GL_FLOAT, GL_FALSE, sizeof(cv::Vec3f), NULL);

	glBindBuffer(GL_ARRAY_BUFFER, vbo_[1]);
	glBufferData(GL_ARRAY_BUFFER, color_pts.size() * sizeof(cv::Vec3f), &color_pts[0], GL_STATIC_DRAW);

	GLuint vert_color_attr = m_shader.attributeLocation("vertColor");

	glEnableVertexAttribArray(vert_color_attr);
	glVertexAttribPointer(vert_color_attr, 3, GL_FLOAT, GL_FALSE, sizeof(cv::Vec3f), NULL);

	no_of_pts_ = world_pts.size();

	glBindVertexArray(NULL);

	setFocus();
}
