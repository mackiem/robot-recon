#include "robotviewer.h"
#include <glm/glm.hpp>
#include <glm/vec3.hpp> // glm::vec3
#include <glm/vec4.hpp> // glm::vec4
#include <glm/mat4x4.hpp> // glm::mat4
#include <glm/gtc/matrix_transform.hpp> // glm::translate, glm::rotate, glm::scale, glm::perspective
#include <glm/gtc/type_ptr.hpp>
#include "objloader.hpp"
#include <random>


RobotViewer::RobotViewer(const QGLFormat& format, QWidget* parent)
	:  QGLWidget(format, parent),
	m_vertexBuffer(QGLBuffer::VertexBuffer), no_of_pts_(0), angle_(0), is_draw_triangles_(true), is_texture_on_(false),
	m_xRot(0), m_yRot(0), m_zRot(0), mouse_down_(false), scale_(1.f)
{

	setFocusPolicy(Qt::StrongFocus);

	//// Configure the timer_
 //   connect(&timer_, SIGNAL(timeout()), this, SLOT(updateGL()));
 //   if(this->format().swapInterval() == -1)
 //   {
 //       // V_blank synchronization not available (tearing likely to happen)
 //       qDebug("Swap Buffers at v_blank not available: refresh at approx 60fps.");
 //       timer_.setInterval(17);
 //   }
 //   else
 //   {
 //       // V_blank synchronization available
 //       timer_.setInterval(0);
 //   }
 //   timer_.start();
}

RobotViewer::~RobotViewer() {
	//if (tex) {
	//	delete[] tex;
	//}
}

void RobotViewer::initializeGL()
{
	QGLFormat glFormat = QGLWidget::format();
	if (!glFormat.sampleBuffers())
		qWarning() << "Could not enable sample buffers";

	// Set the clear color to black
	glClearColor(0.0f, 0.0f, 0.0f, 1.0f);


	// Prepare a complete shader program?
	if (!prepareShaderProgram(":/FilteredStructLight/model_vert.glsl", ":/FilteredStructLight/model_frag.glsl"))
		return;


	// create model and view p


	// Create vertex buffer


	if (!m_shader.bind())
	{
		qWarning() << "Could not bind shader program to context";
		return;
	}

	zoom_ = 45.f;

	camera_distance_ = 100.f;

	look_at_x_ = look_at_y_ = 0.f;

	glm::mat4 projection = glm::perspective(zoom_, static_cast<float>(sizeHint().width())/static_cast<float>(sizeHint().height()), 0.1f, 1000.0f);

	glm::mat4 camera = glm::lookAt(glm::vec3(0.f,0.f,camera_distance_), glm::vec3(0.f,0.f,0.f), glm::vec3(0.f,1.f,0.f));

	GLint projection_location = m_shader.uniformLocation("projection");
	glUniformMatrix4fv(projection_location, 1, GL_FALSE, glm::value_ptr(projection));

	GLint camera_location = m_shader.uniformLocation("camera");
	glUniformMatrix4fv(camera_location, 1, GL_FALSE, glm::value_ptr(camera));

	glEnable(GL_PROGRAM_POINT_SIZE);
	glPointSize(2.0);


	glGenTextures(1, &texture_id_);
	active_texture_ = 0;
	GLuint tex_location = m_shader.uniformLocation("tex");
	glUniform1i(tex_location, active_texture_);

	glBindTexture(GL_TEXTURE_2D, texture_id_);
	glActiveTexture(GL_TEXTURE0 + active_texture_);
	////use fast 4-byte alignment (default anyway) if possible
	//glPixelStorei(GL_UNPACK_ALIGNMENT, (image.step & 3) ? 1 : 4);

	////set length of one complete row in data (doesn't need to equal image.cols)
	//glPixelStorei(GL_UNPACK_ROW_LENGTH, image.step / image.elemSize());

	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

	// Set texture clamping method
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
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

	glGenVertexArrays(1, &vao_pts_);
	glBindVertexArray(vao_pts_);

	glGenBuffers(3, vbo_pts_);

	// Cleanup state
	glBindVertexArray(NULL);
	glBindBuffer(GL_ARRAY_BUFFER, NULL);

	WPts test_pts;
	//for (int x = -1; x <= 1; ++x) {
	//	for (int y = -1; y <= 1; ++y) {
	//		for (int z = -1; z <= 1; ++z) {
	//			test_pts.push_back(cv::Vec3f(x, y, z));
	//		}
	//	}
	//}

	std::vector<glm::vec3> out_vertices;
	std::vector<glm::vec2> out_uvs;
	std::vector<glm::vec3> out_normals;
	loadOBJ("deer-obj.obj", out_vertices, out_uvs, out_normals);

	test_pts.resize(1);

	for (auto& out_vertice : out_vertices) {
		test_pts[0].push_back(cv::Vec3f(out_vertice[0], out_vertice[1], out_vertice[2]));
	}

	update_lines_3d(test_pts);
}

bool RobotViewer::prepareShaderProgram(const QString& vertexShaderPath,
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

void RobotViewer::resizeGL(int w, int h)
{
	// Set the viewport to window dimensions
	glViewport(0, 0, w, qMax(h, 1));
}

void RobotViewer::wheelEvent(QWheelEvent* event) {
	  QPoint numPixels = event->pixelDelta();
    QPoint numDegrees = event->angleDelta() / 8;

	if (!numDegrees.isNull()) {
		//zoom_ += (float)(numDegrees.y()) / (float)(15 * 10);
		camera_distance_ += (float)(numDegrees.y()) / (float)(1);
	}
	update();
}

void RobotViewer::paintGL()
{
	// Clear the buffer with the current clearing color
	glEnable(GL_DEPTH_TEST);
    glEnable(GL_CULL_FACE);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	glBindTexture(GL_TEXTURE_2D, texture_id_);
	glActiveTexture(GL_TEXTURE0 + active_texture_);

	if (!m_shader.bind())
	{
		qWarning() << "Could not bind shader program to context";
		return;
	}

	GLuint model_loc = m_shader.uniformLocation("model");

	//std::cout << translate_x << " " << translate_y << std::endl;

	//glm::mat4 model = glm::rotate(glm::scale(glm::mat4(1.f), glm::vec3(0.1, 0.1, 0.1)), 0.1f * static_cast<float>(angle_++), glm::vec3(0.f, 1.f, 0.f));

	glm::mat4 scale_mat = glm::scale(glm::mat4(1.f), glm::vec3(scale_, scale_, scale_));
	glm::mat4 rotateX = glm::rotate(scale_mat, (float)m_xRot, glm::vec3(1.f, 0.f, 1.f));
	glm::mat4 rotateY = glm::rotate(rotateX, (float)m_yRot, glm::vec3(0.f, 1.f, 0.f));
	glm::mat4 rotateZ = glm::rotate(rotateY, (float)m_zRot, glm::vec3(0.f, 0.f, 1.f));
	//glm::mat4 rotateX = glm::rotate(glm::mat4(1.f), (float)1.f, glm::vec3(1.f, 0.f, 1.f));
	//glm::mat4 rotateY = glm::rotate(rotateX, (float)1.f, glm::vec3(0.f, 1.f, 0.f));
	//glm::mat4 rotateZ = glm::rotate(rotateY, (float)1.f, glm::vec3(0.f, 0.f, 1.f));
	
	glm::mat4 model = rotateZ;

	glUniformMatrix4fv(model_loc, 1, GL_FALSE, glm::value_ptr(model));

	glm::mat4 projection = glm::perspective(zoom_, 
		static_cast<float>(size().width())/static_cast<float>(size().height()), 0.1f, 10000.0f);
	//glm::mat4 projection = glm::perspective(zoom_, 1.f, 0.1f, 1000.0f);
	float ratio = static_cast<float>(size().width())/static_cast<float>(size().height());
	GLint projection_location = m_shader.uniformLocation("projection");
	glUniformMatrix4fv(projection_location, 1, GL_FALSE, glm::value_ptr(projection));

	glm::mat4 camera = glm::lookAt(glm::vec3(look_at_x_, look_at_y_, camera_distance_), glm::vec3(look_at_x_, look_at_y_, 0.f), glm::vec3(0.f,1.f,0.f));

	GLint camera_location = m_shader.uniformLocation("camera");
	glUniformMatrix4fv(camera_location, 1, GL_FALSE, glm::value_ptr(camera));


	GLint texture_used_loc = m_shader.uniformLocation("is_texture_used");
	glUniform1i(texture_used_loc, is_texture_on_);
	//if (is_texture_on_) {
	//} else {
	//	glUniform1i(texture_used_loc, is_texture_on_);
	//}

	for (auto& entity : entities_) {
		entity.draw();
	}


	// Draw stuff
	glBindVertexArray(NULL);
	glBindTexture(GL_TEXTURE_2D, NULL);
}

QSize RobotViewer::sizeHint() const {
	return QSize(1024, 768);
}

void RobotViewer::change_world_pts(WPt& world_pts) {
	
	cv::Vec3f distance(0.f);
	for (auto i = 0u; i < world_pts.size(); ++i) {
		distance += cv::Vec3f(0.f) - cv::Vec3f(world_pts[i]);
	}
	cv::Vec3d avg_distance = distance / (double)(world_pts.size());

	// Copy all scene points into a local array
	WPt pts(world_pts.size());

	for (int p = 0; p < world_pts.size(); p++) {
		pts[p] = world_pts[p] + avg_distance;
	}

	world_pts = pts;
}


void RobotViewer::create_plane(cv::Vec3f normal, double d, cv::Vec3f plane_color) {
	makeCurrent();
	int dist_x = 500, dist_y = 500;

	std::vector<cv::Vec3f> plane_points;
	std::vector<cv::Vec3f> plane_colors;
	for (int x = 0; x < 2; ++x)  {
		for (int y = 0; y < 2; ++y) {
			float x_mul = (x % 2 == 0) ? 1.f : -1.f;
			float y_mul = (y % 2 == 0) ? 1.f : -1.f;

			float plane_x = x_mul * dist_x;
			float plane_y = y_mul * dist_y;
			cv::Vec3f calc_point(plane_x, plane_y, 0);

			float z = 0.f;
			if (normal[2] > 1e-5) {
				z = (d - normal.dot(calc_point)) / normal[2];
			}

			cv::Vec3f plane_point(plane_x, plane_y, z);
			plane_points.push_back(plane_point);
			plane_colors.push_back(plane_color);
		}
	}

	RenderEntity plane(GL_TRIANGLE_STRIP, m_shader);
	plane.upload_data_to_gpu(plane_points, plane_colors);
	entities_.push_back(plane);

}


void RobotViewer::create_points(std::vector<cv::Vec3f> points_3d, cv::Vec3f point_color) {
	makeCurrent();

	std::vector<cv::Vec3f> point_colors;
	for (int i = 0; i < points_3d.size(); ++i) {
		point_colors.push_back(point_color);
	}

	RenderEntity points(GL_POINTS, m_shader);
	points.upload_data_to_gpu(points_3d, point_colors);
	entities_.push_back(points);
	
}

void RobotViewer::create_line(cv::Vec3f a, cv::Vec3f b, cv::Vec3f line_color) {
	makeCurrent();

	std::vector<cv::Vec3f> line_colors;
	for (int i = 0; i < 2; ++i) {
		line_colors.push_back(line_color);
	}

	std::vector<cv::Vec3f> line_points;
	line_points.push_back(a);
	line_points.push_back(b);

	RenderEntity points(GL_LINES, m_shader);
	points.upload_data_to_gpu(line_points, line_colors);
	entities_.push_back(points);
	
}

void RobotViewer::create_plane_with_points_and_lines(std::vector<cv::Vec3f> points_3d, cv::Vec3f line_a, cv::Vec3f line_b, cv::Vec3f normal, double d) {

	std::random_device rd;
	std::mt19937 e2(rd());
	std::uniform_real_distribution<> dist(0, 1);

	cv::Vec3f plane_color(dist(e2), dist(e2), dist(e2));

	create_plane(normal, d, plane_color);


	cv::Vec3f point_color = cv::Vec3f(1.f, 1.f, 1.f) - plane_color;

	cv::Vec3f line_color = cv::Vec3f(1.f, 1.f, 1.f) - plane_color;

	create_line(line_a, line_b, line_color);

	create_points(points_3d, point_color);

}

RenderEntity::RenderEntity(GLenum primitive, QGLShaderProgram& shader) : primitive_(primitive), shader_(shader), vao_(0) {
}


void RenderEntity::upload_data_to_gpu(std::vector<cv::Vec3f>& vertices, std::vector<cv::Vec3f>& colors) {

	shader_.bind();

	glGenVertexArrays(1, &vao_);
	glBindVertexArray(vao_);

	glBindVertexArray(vao_);

	glBindBuffer(GL_ARRAY_BUFFER, vbo_[0]);
	glBufferData(GL_ARRAY_BUFFER, vertices.size() * sizeof(cv::Vec3f), &vertices[0], GL_STATIC_DRAW);

	glGenBuffers(2, vbo_);

	GLuint vert_pos_attr = shader_.attributeLocation("vertex");

	glEnableVertexAttribArray(vert_pos_attr);
	glVertexAttribPointer(vert_pos_attr, 3, GL_FLOAT, GL_FALSE, sizeof(cv::Vec3f), NULL);

	glBindBuffer(GL_ARRAY_BUFFER, vbo_[1]);
	glBufferData(GL_ARRAY_BUFFER, colors.size() * sizeof(cv::Vec3d), &colors[0], GL_STATIC_DRAW);

	GLuint vert_color_attr = shader_.attributeLocation("vertColor");

	glEnableVertexAttribArray(vert_color_attr);
	glVertexAttribPointer(vert_color_attr, 3, GL_FLOAT, GL_FALSE, sizeof(cv::Vec3d), NULL);

	count_ = vertices.size();

	//is_draw_triangles_ = false;
	//GLint texture_used_loc = m_shader.uniformLocation("is_texture_used");
	//glUniform1i(texture_used_loc, is_draw_triangles_);

	glBindVertexArray(NULL);
	
}

void RenderEntity::draw() {
	shader_.bind();
	glBindVertexArray(vao_);
	glDrawArrays(primitive_, 0, count_);
}


//void RobotViewer::update_lines_3d(WPts world_pnts) {
//	makeCurrent();
//
//	if (world_pnts.size() < 1) {
//		std::cout << "No points to display" << std::endl;
//		return;
//	}
//
//	WPt world_pt_single_array;
//	for (auto img = 0u; img < world_pnts.size(); ++img) {
//		for (auto i = 0u; i < world_pnts[img].size(); ++i) {
//			world_pt_single_array.push_back(world_pnts[img][i]);
//		}
//	}
//
//	WPt color_pts;
//	cv::Vec3f white(1.f, 1.f, 1.f);
//	for (auto i = 0u; i < world_pt_single_array.size(); ++i) {
//		color_pts.push_back(white);
//	}
//
//	change_world_pts(world_pt_single_array);
//

//	if (!m_shader.bind())
//	{
//		qWarning() << "Could not bind shader program to context";
//		return;
//	}
//
//	glBindVertexArray(vao_);
//
//	glBindBuffer(GL_ARRAY_BUFFER, vbo_[0]);
//	glBufferData(GL_ARRAY_BUFFER, world_pt_single_array.size() * sizeof(cv::Vec3f), &world_pt_single_array[0], GL_STATIC_DRAW);
//
//	GLuint vert_pos_attr = m_shader.attributeLocation("vertex");
//
//	glEnableVertexAttribArray(vert_pos_attr);
//	glVertexAttribPointer(vert_pos_attr, 3, GL_FLOAT, GL_FALSE, sizeof(cv::Vec3f), NULL);
//
//	glBindBuffer(GL_ARRAY_BUFFER, vbo_[1]);
//	glBufferData(GL_ARRAY_BUFFER, color_pts.size() * sizeof(cv::Vec3f), &color_pts[0], GL_STATIC_DRAW);
//
//	GLuint vert_color_attr = m_shader.attributeLocation("vertColor");
//
//	glEnableVertexAttribArray(vert_color_attr);
//	glVertexAttribPointer(vert_color_attr, 3, GL_FLOAT, GL_FALSE, sizeof(cv::Vec3f), NULL);
//
//	no_of_triangles_ = world_pt_single_array.size();
//
//	//is_draw_triangles_ = false;
//	//GLint texture_used_loc = m_shader.uniformLocation("is_texture_used");
//	//glUniform1i(texture_used_loc, is_draw_triangles_);
//
//	glBindVertexArray(NULL);
//
//	setFocus();
//}

void RobotViewer::update_lines_3d(WPts world_pnts) {
	makeCurrent();

	if (world_pnts.size() < 1) {
		std::cout << "No points to display" << std::endl;
		return;
	}

	WPt world_pt_single_array;
	WPt color_pts;
	cv::Vec3f white(1.f, 1.f, 1.f);

	// account for camera
	//world_pt_single_array.push_back(cv::Vec3f(0.f, 0.f, 0.f));
	//color_pts.push_back(white);


	for (auto img = 0u; img < world_pnts.size(); ++img) {
		for (auto i = 0u; i < world_pnts[img].size(); ++i) {
			world_pt_single_array.push_back(world_pnts[img][i]);
		}
	}


	WPt colors;
	cv::Vec3f red(1.f, 0.f, 0.f);
	cv::Vec3f blue(0.f, 0.f, 1.f);
	cv::Vec3f green(0.f, 1.f, 0.f);

	colors.push_back(red);
	colors.push_back(green);
	colors.push_back(blue);


	for (auto i = 0u; i < world_pt_single_array.size(); ++i) {
		color_pts.push_back(white);
		//color_pts.push_back(colors[i % colors.size()]);
	}

	//change_world_pts(world_pt_single_array);


	if (!m_shader.bind())
	{
		qWarning() << "Could not bind shader program to context";
		return;
	}


	setFocus();
}


void RobotViewer::gen_texture(GLuint& texture_id, cv::Mat& remapped_img_gray) {

	makeCurrent();

	cv::Mat remapped_img_for_texture;
	cv::cvtColor(remapped_img_gray, remapped_img_for_texture, CV_GRAY2BGR);
	glBindTexture(GL_TEXTURE_2D, texture_id);

	glActiveTexture(GL_TEXTURE0 + active_texture_);
	glTexImage2D(GL_TEXTURE_2D,     // Type of texture
		0,
		GL_RGB,
		remapped_img_for_texture.cols,
		remapped_img_for_texture.rows,
		0,
		GL_BGR,
		GL_UNSIGNED_BYTE,
		remapped_img_for_texture.ptr());


	glBindTexture(GL_TEXTURE_2D, 0);

}

void RobotViewer::draw_triangles() {
	is_draw_triangles_ = true;
	update();
}

void RobotViewer::draw_points() {
	is_draw_triangles_ = false;
	update();
}

void RobotViewer::draw_colors() {
	is_texture_on_ = false;
	update();
}

void RobotViewer::draw_texture() {
	is_texture_on_ = true;
	update();
}

void RobotViewer::reset_view() {
	look_at_x_ = look_at_y_ = 0.f;
	update();
}

void RobotViewer::set_scale(int value) {
	scale_ = 0.01f * value;
	update();
}

void RobotViewer::update_model_with_triangles(WPts world_pnts, WPts world_pt_colrs, WPt triangles, IPt texture_coords,
											  cv::Mat texture_img) {
	makeCurrent();

	if (triangles.size() < 1) {
		std::cout << "No triangles to display" << std::endl;
//		return;
	}

	WPt world_pt_single_array;
	for (auto img = 0u; img < world_pnts.size(); ++img) {
		for (auto i = 0u; i < world_pnts[img].size(); ++i) {
			world_pt_single_array.push_back(cv::Vec3d(world_pnts[img][i][0], world_pnts[img][i][1], world_pnts[img][i][2]));
		}
	}

	std::vector<glm::vec3> world_pt_colors_single_array;
	for (auto img = 0u; img < world_pt_colrs.size(); ++img) {
		for (auto i = 0u; i < world_pt_colrs[img].size(); ++i) {
			world_pt_colors_single_array.push_back(glm::vec3(world_pt_colrs[img][i][0], world_pt_colrs[img][i][1], 
				world_pt_colrs[img][i][2]));
		}
	}

	WPt color_pts;
	cv::Vec3f white(1.f, 0.f, 0.f);
	for (auto i = 0u; i < triangles.size(); ++i) {
		color_pts.push_back(white);
	}

	change_world_pts(triangles);


	if (!m_shader.bind())
	{
		qWarning() << "Could not bind shader program to context";
		return;
	}

	GLuint vert_pos_attr = m_shader.attributeLocation("vertex");
	GLuint vert_color_attr = m_shader.attributeLocation("vertColor");
	GLuint vert_tex_attr = m_shader.attributeLocation("vertCoords");

	std::vector<glm::vec3> glm_world_pts;

	for (auto& triangle : triangles) {
		glm::vec3 vec(triangle[0], triangle[1], triangle[2]);
		glm_world_pts.push_back(vec);
	}

	std::vector<glm::vec2> glm_tex_coords;
	for (auto& coord : texture_coords) {
		glm::vec2 vec2(coord[0], coord[1]);
		glm_tex_coords.push_back(vec2);
	}


	if (triangles.size() > 0) {
		glBindVertexArray(vao_);

		glBindBuffer(GL_ARRAY_BUFFER, vbo_[0]);
		//glBufferData(GL_ARRAY_BUFFER, triangles.size() * sizeof(cv::Vec3f), &triangles[0], GL_STATIC_DRAW);
		glBufferData(GL_ARRAY_BUFFER, glm_world_pts.size() * sizeof(glm::vec3), &glm_world_pts[0], GL_STATIC_DRAW);


		glEnableVertexAttribArray(vert_pos_attr);
		glVertexAttribPointer(vert_pos_attr, 3, GL_FLOAT, GL_FALSE, sizeof(glm::vec3), NULL);

		glBindBuffer(GL_ARRAY_BUFFER, vbo_[1]);
		glBufferData(GL_ARRAY_BUFFER, color_pts.size() * sizeof(cv::Vec3d), &color_pts[0], GL_STATIC_DRAW);


		glEnableVertexAttribArray(vert_color_attr);
		glVertexAttribPointer(vert_color_attr, 3, GL_FLOAT, GL_FALSE, sizeof(cv::Vec3d), NULL);

		glBindBuffer(GL_ARRAY_BUFFER, vbo_[2]);
		glBufferData(GL_ARRAY_BUFFER, texture_coords.size() * sizeof(glm::vec2), &glm_tex_coords[0], GL_STATIC_DRAW);


		glEnableVertexAttribArray(vert_tex_attr);
		glVertexAttribPointer(vert_tex_attr, 2, GL_FLOAT, GL_FALSE, sizeof(glm::vec2), NULL);

		no_of_triangles_ = triangles.size();
	}
	
	gen_texture(texture_id_, texture_img);

	//is_draw_triangles_ = true;
	//GLint texture_used_loc = m_shader.uniformLocation("is_texture_used");
	//glUniform1i(texture_used_loc, is_draw_triangles_);

	if (world_pt_single_array.size() < 1) {
		std::cout << "No points to display" << std::endl;
		return;
	}

	change_world_pts(world_pt_single_array);

	std::vector<glm::vec3> glm_world_pt_single_array;
		for (auto i = 0u; i < world_pt_single_array.size(); ++i) {
			glm_world_pt_single_array.push_back(glm::vec3(world_pt_single_array[i][0], world_pt_single_array[i][1], 
				world_pt_single_array[i][2]));
		}


	glBindVertexArray(vao_pts_);

	glBindBuffer(GL_ARRAY_BUFFER, vbo_pts_[0]);
	glBufferData(GL_ARRAY_BUFFER, glm_world_pt_single_array.size() * sizeof(glm::vec3), &glm_world_pt_single_array[0], GL_STATIC_DRAW);


	glEnableVertexAttribArray(vert_pos_attr);
	glVertexAttribPointer(vert_pos_attr, 3, GL_FLOAT, GL_FALSE, sizeof(glm::vec3), NULL);

	glBindBuffer(GL_ARRAY_BUFFER, vbo_pts_[1]);
	glBufferData(GL_ARRAY_BUFFER, world_pt_colors_single_array.size() * sizeof(glm::vec3), &world_pt_colors_single_array[0], GL_STATIC_DRAW);


	glEnableVertexAttribArray(vert_color_attr);
	glVertexAttribPointer(vert_color_attr, 3, GL_FLOAT, GL_FALSE, sizeof(glm::vec3), NULL);

	no_of_pts_ = world_pt_single_array.size();

	glBindVertexArray(NULL);
	
}


static void qNormalizeAngle(float &angle)
{
    while (angle < 0)
        //angle += 360 * 16;
        angle += 360;
    while (angle > 360 * 16)
    //while (angle > 360 * 16)
        //angle -= 360 * 16;
        angle -= 360;
}

void RobotViewer::setXRotation(float angle)
{
    qNormalizeAngle(angle);
    if (angle != m_xRot) {
        m_xRot = angle;
        update();
    }
}

void RobotViewer::setYRotation(float angle)
{
    qNormalizeAngle(angle);
    if (angle != m_yRot) {
        m_yRot = angle;
        update();
    }
}

void RobotViewer::setZRotation(float angle)
{
    qNormalizeAngle(angle);
    if (angle != m_zRot) {
        m_zRot = angle;
        update();
    }
}


void RobotViewer::mouseMoveEvent(QMouseEvent *event)
{
	if (mouse_down_) {
		int dx = event->x() - m_lastPos.x();
		int dy = event->y() - m_lastPos.y();

		float mouse_drag_sensitivity = 0.1;
		if (event->buttons() & Qt::LeftButton) {
			setXRotation(m_xRot + mouse_drag_sensitivity * dy);
			setYRotation(m_yRot + mouse_drag_sensitivity * dx);
		} else if (event->buttons() & Qt::RightButton) {
			setXRotation(m_xRot + mouse_drag_sensitivity * dy);
			setZRotation(m_zRot + mouse_drag_sensitivity * dx);
		}
		m_lastPos = event->pos();
	} else {
		m_lastPos = event->pos();
		mouse_down_ = true;
	}
}

void RobotViewer::mouseReleaseEvent(QMouseEvent* event) {
	mouse_down_ = false;
}

void RobotViewer::keyPressEvent(QKeyEvent* event) {
	float multiplier = 1.f;
	if (event->key() == Qt::Key_D) {
		look_at_x_ += multiplier;
	} else if (event->key() == Qt::Key_A) {
		look_at_x_ -= multiplier;
	} else if (event->key() == Qt::Key_W) {
		look_at_y_ += multiplier;
	} else if (event->key() == Qt::Key_S) {
		look_at_y_ -= multiplier;
	}
	update();
}