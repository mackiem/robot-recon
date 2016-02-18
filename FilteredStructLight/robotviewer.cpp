#include "robotviewer.h"
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


	// enable alpha
	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    //glEnable(GL_CULL_FACE);
	glEnable(GL_DEPTH_TEST);

	// Set the clear color to black
	glClearColor(0.1f, 0.1f, 0.1f, 1.f);


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

	camera_distance_ = -100.f;

	look_at_x_ = 45.f;
	look_at_y_ = -145.f;

	glm::mat4 projection = glm::perspective(zoom_, static_cast<float>(sizeHint().width())/static_cast<float>(sizeHint().height()), 0.1f, 1000.0f);

	up_ = glm::vec3(0.f,-1.f,0.f);

	eye_ = glm::vec3(look_at_x_, look_at_y_, camera_distance_);
	center_ = glm::vec3(0, 0, 0.f);

	glm::mat4 camera = glm::lookAt(eye_, center_, up_);

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

	draw_camera();
	draw_table_top();
	draw_robot();

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
	glm::mat4 rotateX = glm::rotate(scale_mat, ((float)m_xRot), glm::vec3(1.f, 0.f, 1.f));
	glm::mat4 rotateY = glm::rotate(rotateX, ((float)m_yRot), glm::vec3(0.f, 1.f, 0.f));
	glm::mat4 rotateZ = glm::rotate(rotateY, ((float)m_zRot), glm::vec3(0.f, 0.f, 1.f));
	//glm::mat4 rotateX = glm::rotate(glm::mat4(1.f), (float)1.f, glm::vec3(1.f, 0.f, 1.f));
	//glm::mat4 rotateY = glm::rotate(rotateX, (float)1.f, glm::vec3(0.f, 1.f, 0.f));
	//glm::mat4 rotateZ = glm::rotate(rotateY, (float)1.f, glm::vec3(0.f, 0.f, 1.f));
	
	model_ = rotateZ;

	//glm::vec3 up = glm::mat3(model_) * up_;
	glm::vec3 up = up_;
	glm::vec3 zoom = camera_distance_ * glm::normalize(center_ - eye_);
	glm::vec3 eye = eye_ - zoom;

	eye += glm::vec3(look_at_x_, look_at_y_, 0.f);
	glm::vec3 center = center_ + glm::vec3(look_at_x_, look_at_y_, 0.f);

	//direction = glm::mat3(model_) * direction;
	//direction = glm::mat3(model_) * direction;

	//glUniformMatrix4fv(model_loc, 1, GL_FALSE, glm::value_ptr(model));

	glm::mat4 projection = glm::perspective(zoom_, 
		static_cast<float>(size().width())/static_cast<float>(size().height()), 0.1f, 10000.0f);
	//glm::mat4 projection = glm::perspective(zoom_, 1.f, 0.1f, 1000.0f);
	float ratio = static_cast<float>(size().width())/static_cast<float>(size().height());
	GLint projection_location = m_shader.uniformLocation("projection");
	glUniformMatrix4fv(projection_location, 1, GL_FALSE, glm::value_ptr(projection));



	glm::mat4 camera = glm::lookAt(eye, 
		center, 
		up);

	//glm::mat4 camera = glm::lookAt(glm::vec3(look_at_x_, look_at_y_, camera_distance_), 
	//	glm::vec3(m_xRot, m_yRot, m_zRot), glm::vec3(0.f, -1.f, 0.f));

	GLint camera_location = m_shader.uniformLocation("camera");
	glUniformMatrix4fv(camera_location, 1, GL_FALSE, glm::value_ptr(camera));


	GLint texture_used_loc = m_shader.uniformLocation("is_texture_used");
	glUniform1i(texture_used_loc, is_texture_on_);
	//if (is_texture_on_) {
	//} else {
	//	glUniform1i(texture_used_loc, is_texture_on_);
	//}

	for (auto& entity : entities_) {
		RenderEntity::Type entity_type = entity.get_type();
		bool draw = true;
		switch (entity_type) {
		case RenderEntity::Plane: {
			draw = draw_planes_;
			break;
		}
		case RenderEntity::Points: draw = draw_points_; break;
		case RenderEntity::Lines: draw = draw_lines_; break;
		case RenderEntity::Default: draw = draw_default_; break;
		default: break;
		}

		if (draw) {
			glm::mat4 transformed_model = model_ * entity.model_;
			glUniformMatrix4fv(model_loc, 1, GL_FALSE, glm::value_ptr(transformed_model));
			//entity.model_ = entity.model_ * model_ ;
			entity.draw();
		}
	}


	// Draw stuff
	glBindVertexArray(NULL);
	glBindTexture(GL_TEXTURE_2D, NULL);
}

QSize RobotViewer::sizeHint() const {
	return QSize(2048, 768);
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


void RobotViewer::create_plane(cv::Vec3f normal, double d, cv::Vec4f plane_color) {
	makeCurrent();
	int dist_x = 200, dist_y = 200;

	std::vector<cv::Vec3f> plane_points;
	std::vector<cv::Vec4f> plane_colors;
	for (int x = 0; x < 2; ++x)  {
		for (int y = 0; y < 2; ++y) {
			float x_mul = (x % 2 == 0) ? 1.f : -1.f;
			float y_mul = (y % 2 == 0) ? 1.f : -1.f;

			float plane_x = x_mul * dist_x;
			float plane_y = y_mul * dist_y;
			cv::Vec3f calc_point(plane_x, plane_y, 0);

			float z = 0.f;
			float c_component = normal[2];
			if (std::abs(c_component) > 1e-6) {
				z = (d - normal.dot(calc_point)) / c_component;
			}

			cv::Vec3f plane_point(plane_x, plane_y, z);
			plane_points.push_back(plane_point);
			plane_colors.push_back(plane_color);
		}
	}

	RenderEntity plane(GL_TRIANGLE_STRIP, m_shader);
	plane.set_type(RenderEntity::Plane);
	std::vector<cv::Vec3f> normals;
	plane.upload_data_to_gpu(plane_points, plane_colors, normals);
	entities_.push_back(plane);

}


void RobotViewer::create_points(std::vector<cv::Vec3f> points_3d, cv::Vec4f point_color) {
	makeCurrent();

	std::vector<cv::Vec4f> point_colors;
	for (int i = 0; i < points_3d.size(); ++i) {
		point_colors.push_back(point_color);
	}

	RenderEntity points(GL_POINTS, m_shader);
	points.set_type(RenderEntity::Points);
	std::vector<cv::Vec3f> normals;
	points.upload_data_to_gpu(points_3d, point_colors, normals);
	entities_.push_back(points);
	
}

void RobotViewer::toggle_draw_points(int check_state) {
	draw_points_ = (check_state == 0) ? false : true;
	update();
}

void RobotViewer::toggle_draw_planes(int check_state) {
	draw_planes_ = (check_state == 0) ? false : true;
	update();
}

void RobotViewer::toggle_draw_lines(int check_state) {
	draw_lines_ = (check_state == 0) ? false : true;
	update();
}

void RobotViewer::toggle_draw_default(int check_state) {
	draw_default_ = (check_state == 0) ? false : true;
	update();
}

void RobotViewer::create_line(cv::Vec3f a, cv::Vec3f b, cv::Vec4f line_color, RenderEntity::Type type) {
	makeCurrent();

	std::vector<cv::Vec4f> line_colors;
	for (int i = 0; i < 2; ++i) {
		line_colors.push_back(line_color);
	}

	std::vector<cv::Vec3f> line_points;
	line_points.push_back(a);
	line_points.push_back(b);

	RenderEntity points(GL_LINES, m_shader);
	points.set_type(type);
	std::vector<cv::Vec3f> normals;
	points.upload_data_to_gpu(line_points, line_colors, normals);
	entities_.push_back(points);
	
}

void RobotViewer::create_plane_with_points_and_lines(std::vector<cv::Vec3f> points_3d, cv::Vec3f line_a, cv::Vec3f line_b, cv::Vec3f normal, double d) {

	std::random_device rd;
	std::mt19937 e2(rd());
	std::uniform_real_distribution<> dist(0.5, 1);

	cv::Vec4f plane_color(dist(e2), dist(e2), dist(e2), 0.1f);

	create_plane(normal, d, plane_color);


	cv::Vec4f point_color = cv::Vec4f(1.f, 1.f, 1.f, 1.f) - plane_color;
	cv::Vec4f line_color = point_color;

	//cv::Vec3f point_color =  plane_color;
	//cv::Vec3f line_color = point_color;


	create_line(line_a, line_b, line_color);

	create_points(points_3d, point_color);

}

RenderEntity::RenderEntity(GLenum primitive, QGLShaderProgram& shader) : primitive_(primitive), shader_(shader), vao_(0) {
	model_ = glm::mat4(1.f);
	model_loc_ = shader_.uniformLocation("model");
}

void RenderEntity::set_type(Type type) {
	type_ = type;
}

RenderEntity::Type RenderEntity::get_type() {
	return type_;
}

void RenderEntity::set_model(glm::mat4 model) {
	model_ = model;
}

void RenderEntity::upload_data_to_gpu(std::vector<cv::Vec3f>& vertices, std::vector<cv::Vec4f>& colors,
	std::vector<cv::Vec3f>& normals) {

	shader_.bind();

	glGenVertexArrays(1, &vao_);
	glBindVertexArray(vao_);


	glGenBuffers(2, vbo_);

	glBindBuffer(GL_ARRAY_BUFFER, vbo_[0]);
	glBufferData(GL_ARRAY_BUFFER, vertices.size() * sizeof(cv::Vec3f), &vertices[0], GL_STATIC_DRAW);


	GLuint vert_pos_attr = shader_.attributeLocation("vertex");

	glEnableVertexAttribArray(vert_pos_attr);
	glVertexAttribPointer(vert_pos_attr, 3, GL_FLOAT, GL_FALSE, sizeof(cv::Vec3f), NULL);

	glBindBuffer(GL_ARRAY_BUFFER, vbo_[1]);
	glBufferData(GL_ARRAY_BUFFER, colors.size() * sizeof(cv::Vec4f), &colors[0], GL_STATIC_DRAW);

	GLuint vert_color_attr = shader_.attributeLocation("vertColor");

	glEnableVertexAttribArray(vert_color_attr);
	glVertexAttribPointer(vert_color_attr, 4, GL_FLOAT, GL_FALSE, sizeof(cv::Vec4f), NULL);

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

void RobotViewer::load_obj(std::string filename, std::vector<cv::Vec3f>& vertices, std::vector<cv::Vec3f>& normals) {
	std::vector<glm::vec3> glm_vertices;
	std::vector<glm::vec2> uvs;
	std::vector<glm::vec3> glm_normals;
	loadOBJ(filename.c_str(), glm_vertices, uvs, glm_normals);

	
	for (auto& out_vertice : glm_vertices) {
		vertices.push_back(cv::Vec3f(static_cast<float>(out_vertice[0]), static_cast<float>(out_vertice[1])
			, static_cast<float>(out_vertice[2])));
	}

	for (auto& out_vertice : glm_normals) {
		normals.push_back(cv::Vec3f(static_cast<float>(out_vertice[0]), static_cast<float>(out_vertice[1])
			, static_cast<float>(out_vertice[2])));
	}

}

void RobotViewer::draw_camera() {
	makeCurrent();
	RenderEntity camera(GL_TRIANGLES, m_shader);
	camera.set_type(RenderEntity::Default);

	std::vector<cv::Vec3f> vertices;
	std::vector<cv::Vec4f> colors;
	std::vector<cv::Vec3f> normals;

	cv::Vec4f grey(0.8f, 0.8f, 0.8f, 0.5f);

	load_obj("Camera/camera.obj", vertices, normals);

	for (auto& vertice : vertices) {
		colors.push_back(grey);
	}

	float scale = 50.f;
	glm::mat4 model = glm::scale(glm::mat4(1.f), glm::vec3(scale, scale, scale));
	camera.set_model(model);
	camera.upload_data_to_gpu(vertices, colors, normals);
	entities_.push_back(camera);

	// draw x, y, z lines
	cv::Vec3f origin(0.f, 0.f, 0.f);

	const float multiplier = 100.f;
	cv::Vec3f x_point(multiplier * 1.f, 0.f, 0.f);
	cv::Vec4f x_color(1.f, 0.f, 0.f, 1.f );

	cv::Vec3f y_point(0.f, multiplier * 1.f, 0.f);
	cv::Vec4f y_color(0.f, 1.f, 0.f, 1.f);

	cv::Vec3f z_point(0.f, 0.f, multiplier * 1.f);
	cv::Vec4f z_color(0.f, 0.f, 1.f, 1.f);

	create_line(origin, x_point, x_color, RenderEntity::Default);
	create_line(origin, y_point, y_color, RenderEntity::Default);
	create_line(origin, z_point, z_color, RenderEntity::Default);

}

void RobotViewer::draw_table_top() {
	makeCurrent();

	cv::Vec4f green(0.f, 1.f, 0.f, 1.f);

	float y = 160.f;
	float space = 10.f;
	int start = 10;
	float max_value = 200.f;

	std::vector<cv::Vec3f> points;
	std::vector<cv::Vec4f> colors;
	std::vector<cv::Vec3f> normals;

	for (int i = -20; i < 21; ++i) {
		// x lines
		cv::Vec3f start_x(i * space, y, -1 * max_value);
		cv::Vec3f end_x(i * space, y, max_value);

		// z lines
		cv::Vec3f start_z(-1 * max_value,  y, i * space);
		cv::Vec3f end_z(max_value, y, i * space);

		// points
		points.push_back(start_x);
		points.push_back(end_x);
		points.push_back(start_z);
		points.push_back(end_z);

		// colors
		colors.push_back(green);
		colors.push_back(green);
		colors.push_back(green);
		colors.push_back(green);
	}

	RenderEntity grid_floor(GL_LINES, m_shader);
	grid_floor.set_type(RenderEntity::Default);
	grid_floor.upload_data_to_gpu(points, colors, normals);

	entities_.push_back(grid_floor);
}

void RobotViewer::draw_robot() {
	makeCurrent();
	std::vector<cv::Vec3f> vertices;
	std::vector<cv::Vec4f> colors;
	std::vector<cv::Vec3f> normals;

	cv::Vec4f color(0.2f, 0.5f, 0.5f, 0.8f);
	//cv::Vec4f color(1.f, 1.f, 1.f, 1.f);



	load_obj("cube.obj.txt", vertices, normals);
	for (auto& vertice : vertices) {
		colors.push_back(color);
	}

	RenderEntity robot(GL_TRIANGLES, m_shader);
	robot.set_type(RenderEntity::Default);
	float scale = 50.f;
	glm::mat4 model;
	model = glm::translate(model, glm::vec3(0.f, 60.f, 0.f));
	model = glm::scale(model, glm::vec3(scale, scale, scale));
	robot.set_model(model);

	robot.upload_data_to_gpu(vertices, colors, normals);
	entities_.push_back(robot);
}

void RobotViewer::clear_models() {
	entities_.clear();
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

		float mouse_drag_sensitivity = 0.01;
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
		//m_lastPos = QPoint(0, 0);
		mouse_down_ = true;
	}
}

void RobotViewer::mouseReleaseEvent(QMouseEvent* event) {
	mouse_down_ = false;
}

void RobotViewer::keyPressEvent(QKeyEvent* event) {
	float multiplier = 10.f;
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