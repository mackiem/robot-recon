#include "robotviewer.h"
#include <glm/vec3.hpp> // glm::vec3
#include <glm/vec4.hpp> // glm::vec4
#include <glm/mat4x4.hpp> // glm::mat4
#include <glm/gtc/matrix_transform.hpp> // glm::translate, glm::rotate, glm::scale, glm::perspective
#include <glm/gtc/type_ptr.hpp>
#include "objloader.hpp"
#include <random>
#include <tiny_obj_loader.h>


const std::string RobotViewer::CAMERA = "camera";
const std::string RobotViewer::IMAGE_PLANE = "image_plane";
const std::string RobotViewer::ROBOT = "robot";
const std::string RobotViewer::FLOOR_GRID = "floor_grid";


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

void RobotViewer::set_shader_paths(const char* vertex_shader_path, const char* fragment_shader_path) {
	vertex_shader_path_ = vertex_shader_path;
	fragment_shader_path_ = fragment_shader_path;
}

void RobotViewer::set_shaders() {
	set_shader_paths(":/FilteredStructLight/model_vert.glsl", ":/FilteredStructLight/model_frag.glsl");
}

void RobotViewer::init_camera_params() {

	fovy_ = 45.f;

	camera_distance_ = -100.f;

	look_at_z_ = -500.f;
	look_at_x_ = -245.f;
	look_at_y_ = 245.f;


	up_ = glm::vec3(0.f,1.f,0.f);

	eye_ = glm::vec3(look_at_x_, look_at_y_, look_at_z_);
	center_ = glm::vec3(0, 0, 0.f);


}

void RobotViewer::custom_init_code() {
	glm::mat4 camera = glm::lookAt(eye_, center_, up_);
	glm::mat4 projection = glm::perspective(fovy_, static_cast<float>(sizeHint().width())
		/static_cast<float>(sizeHint().height()), 0.1f, 1000.0f);

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


	set_shaders();

	// Prepare a complete shader program?
	if (!prepareShaderProgram(vertex_shader_path_, fragment_shader_path_))
		return;


	init_camera_params();

	if (!m_shader.bind())
	{
		qWarning() << "Could not bind shader program to context";
		return;
	}

	custom_init_code();

	load_inital_models();
}

bool RobotViewer::prepareShaderProgram(const QString& vertexShaderPath,
	const QString& fragmentShaderPath)
{
	bool result = prepareShaderProgram(vertexShaderPath, fragmentShaderPath, m_shader);

	return result;
}

bool RobotViewer::prepareShaderProgram(const QString& vertexShaderPath,
	const QString& fragmentShaderPath, QGLShaderProgram& shader)
{
	// First we load and compile the vertex shader?
	bool result = shader.addShaderFromSourceFile(QGLShader::Vertex, vertexShaderPath);
	if (!result)
		qWarning() << shader.log();

	// …now the fragment shader?
	result = shader.addShaderFromSourceFile(QGLShader::Fragment, fragmentShaderPath);
	if (!result)
		qWarning() << shader.log();

	// …and finally we link them to resolve any references.
	result = shader.link();
	if (!result)
		qWarning() << "Could not link shader program:" << shader.log();

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
		//fovy_ += (float)(numDegrees.y()) / (float)(15 * 10);
		camera_distance_ += (float)(numDegrees.y()) / (float)(1);
	}
	update();
}

void RobotViewer::draw_mesh(RenderMesh& mesh) {
	GLuint model_loc = m_shader.uniformLocation("model");
	for (auto& entity : mesh) {
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
}

void RobotViewer::draw_scene(Scene& scene) {
	for (auto& rendermesh : scene) {
		draw_mesh(rendermesh);
	}
}

void RobotViewer::update_camera() {
	GLuint model_loc = m_shader.uniformLocation("model");

	glm::mat4 scale_mat = glm::scale(glm::mat4(1.f), glm::vec3(scale_, scale_, scale_));
	glm::mat4 rotateX = glm::rotate(scale_mat, glm::radians((float)m_xRot), glm::vec3(1.f, 0.f, 0.f));
	glm::mat4 rotateY = glm::rotate(rotateX, glm::radians((float)m_yRot), glm::vec3(0.f, 1.f, 0.f));
	glm::mat4 rotateZ = glm::rotate(rotateY,  glm::radians((float)m_zRot), glm::vec3(0.f, 0.f, 1.f));
	//glm::mat4 rotateX = glm::rotate(glm::mat4(1.f), (float)1.f, glm::vec3(1.f, 0.f, 1.f));
	//glm::mat4 rotateY = glm::rotate(rotateX, (float)1.f, glm::vec3(0.f, 1.f, 0.f));
	//glm::mat4 rotateZ = glm::rotate(rotateY, (float)1.f, glm::vec3(0.f, 0.f, 1.f));
	
	model_ = rotateZ;

	//glm::vec3 up = glm::mat3(model_) * up_;
	glm::vec3 up = up_;
	glm::vec3 zoom = camera_distance_ * glm::normalize(center_ - eye_);
	glm::vec3 eye = eye_ - zoom;
	glm::vec3 center = center_ - zoom;

	eye += glm::vec3(look_at_x_, look_at_y_, 0.f);
	center += glm::vec3(look_at_x_, look_at_y_, 0.f);

	//direction = glm::mat3(model_) * direction;
	//direction = glm::mat3(model_) * direction;

	//glUniformMatrix4fv(model_loc, 1, GL_FALSE, glm::value_ptr(model));

	projection_ = glm::perspective(fovy_, 
		static_cast<float>(size().width())/static_cast<float>(size().height()), 0.1f, 100000.0f);
	//glm::mat4 projection = glm::perspective(fovy_, 1.f, 0.1f, 1000.0f);


	camera_ = glm::lookAt(eye, 
		center, 
		up);
	
}

void RobotViewer::custom_draw_code() {
	//glm::mat4 camera = glm::lookAt(glm::vec3(look_at_x_, look_at_y_, camera_distance_), 
	//	glm::vec3(m_xRot, m_yRot, m_zRot), glm::vec3(0.f, -1.f, 0.f));

	glBindTexture(GL_TEXTURE_2D, texture_id_);
	glActiveTexture(GL_TEXTURE0 + active_texture_);

	GLint camera_location = m_shader.uniformLocation("camera");
	glUniformMatrix4fv(camera_location, 1, GL_FALSE, glm::value_ptr(camera_));

	float ratio = static_cast<float>(size().width())/static_cast<float>(size().height());
	GLint projection_location = m_shader.uniformLocation("projection");
	glUniformMatrix4fv(projection_location, 1, GL_FALSE, glm::value_ptr(projection_));

	GLint texture_used_loc = m_shader.uniformLocation("is_texture_used");
	glUniform1i(texture_used_loc, is_texture_on_);
	//if (is_texture_on_) {
	//} else {
	//	glUniform1i(texture_used_loc, is_texture_on_);
	//}



	if (frames_.size() > 0) {
		if (draw_frame_by_frame_) {
			if ((current_frame_to_draw_ >= 0) 
				&& (current_frame_to_draw_ <= (frames_.size() - 1))) {
				Scene scene = frames_[current_frame_to_draw_];
				draw_scene(scene);
			}
		} else {
			for (auto& scene : frames_) {
				draw_scene(scene);
			}
		}
	} else {
		// always draw the default scene, if nothing is there
		draw_scene(default_scene_);
	}
	
}

void RobotViewer::paintGL()
{
	// Clear the buffer with the current clearing color
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);


	if (!m_shader.bind())
	{
		qWarning() << "Could not bind shader program to context";
		return;
	}

	update_camera();

	custom_draw_code();

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
	RenderEntity plane = get_plane_entity(normal, d, plane_color);
	entities_.push_back(plane);

}


void RobotViewer::create_points(std::vector<cv::Vec3f> points_3d, cv::Vec4f point_color) {
	RenderEntity points = get_points_entity(points_3d, point_color);
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

void RobotViewer::toggle_frame_by_frame(int check_state) {
	draw_frame_by_frame_ = (check_state == 0) ? false : true;
	update();
}

void RobotViewer::create_line(cv::Vec3f a, cv::Vec3f b, cv::Vec4f line_color, cv::Mat& model, RenderEntity::Type type) {
	makeCurrent();

	std::vector<cv::Vec4f> line_colors;
	for (int i = 0; i < 2; ++i) {
		line_colors.push_back(line_color);
	}

	std::vector<cv::Vec3f> line_points;
	line_points.push_back(a);
	line_points.push_back(b);

	RenderEntity points(GL_LINES, &m_shader);
	points.set_type(type);
	points.set_model(convert(model));
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


	//create_line(line_a, line_b, line_color, );

	create_points(points_3d, point_color);

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


	fovy_ = 45.f;

	camera_distance_ = -100.f;

	look_at_z_ = -500.f;
	look_at_x_ = -245.f;
	look_at_y_ = 245.f;
	
	up_ = glm::vec3(0.f,1.f,0.f);
	eye_ = glm::vec3(look_at_x_, look_at_y_, look_at_z_);
	center_ = glm::vec3(0, 0, 0.f);

	m_xRot = m_yRot = m_zRot = 0.f;

	update();
}

void RobotViewer::center_view() {
	//center = point_centroid_;
	auto direction = glm::normalize(center_ - eye_);
	camera_distance_ = 200.f;
	center_ = point_centroid_;
	eye_ = center_ - (camera_distance_ * direction);
	update();
}

void RobotViewer::top_view() {
}

void RobotViewer::set_scale(int value) {
	scale_ = 0.01f * value;
	update();
}

void RobotViewer::start_reconstruction_sequence() {
	frames_.clear();
	points_.clear();
}

void RobotViewer::end_reconstruction_sequence() {
	point_centroid_ = glm::vec3(0.f, 0.f, 0.f);

	if (points_.size() > 0) {
		for (auto& point : points_) {
			point_centroid_ += glm::vec3(point[0], point[1], point[2]);
		}
		point_centroid_ /= points_.size();
	}
}

void RobotViewer::create_calibration_frame(std::vector<cv::Vec3f> points_3d,
	cv::Vec3f line_a, cv::Vec3f line_b, cv::Vec3f normal, double d) {

	std::random_device rd;
	std::mt19937 e2(rd());
	std::uniform_real_distribution<> dist(0.3, 1);

	cv::Vec4f common_color(dist(e2), dist(e2), dist(e2), 0.2f);

	RenderEntity checkerboard_plane = get_plane_entity(normal, d, common_color);

	float elongation_length = 200.f;
	cv::Vec3f elongated_line_b = line_a + elongation_length * (line_b - line_a);
	cv::Vec3f elongated_line_a = line_a - elongation_length * (line_b - line_a);
	RenderEntity fitted_line = get_line_entity(elongated_line_a, elongated_line_b, common_color);

	RenderEntity line_3d_points = get_points_entity(points_3d, common_color);

	RenderMesh calibration;
	calibration.push_back(checkerboard_plane);
	calibration.push_back(line_3d_points);
	calibration.push_back(fitted_line);
	// make copies not references
	RenderMesh camera = assets_[CAMERA];

	RenderMesh robot = assets_[ROBOT];

	RenderMesh floor_grid = assets_[FLOOR_GRID];
	//update_model(floor_grid, RT);

	std::vector<RenderMesh> scene;
	scene.push_back(calibration);
	scene.push_back(camera);
	scene.push_back(robot);
	scene.push_back(floor_grid);

	frames_.push_back(scene);

	// insert points for centroid
	points_.insert(points_.end(), points_3d.begin(), points_3d.end());
}

void RobotViewer::create_final_calibration_frame(cv::Vec3f normal, double d) {

	std::random_device rd;
	std::mt19937 e2(rd());
	std::uniform_real_distribution<> dist(0.3, 1);

	cv::Vec4f common_color(dist(e2), dist(e2), dist(e2), 0.2f);

	RenderEntity scanline_plane = get_plane_entity(normal, d, common_color);

	RenderMesh calibration;
	calibration.push_back(scanline_plane);

	// make copies not references
	RenderMesh camera = assets_[CAMERA];

	RenderMesh robot = assets_[ROBOT];

	RenderMesh floor_grid = assets_[FLOOR_GRID];
	//update_model(floor_grid, RT);

	std::vector<RenderMesh> scene;
	scene.push_back(calibration);
	scene.push_back(camera);
	scene.push_back(robot);
	scene.push_back(floor_grid);

	frames_.push_back(scene);
}

void RobotViewer::create_reconstruction_frame(std::vector<cv::Vec3f> points_3d, 
	cv::Vec3f line_a, cv::Vec3f line_b, cv::Vec3f normal, double d, cv::Mat RT) {

	std::random_device rd;
	std::mt19937 e2(rd());
	std::uniform_real_distribution<> dist(0.3, 1);

	cv::Vec4f common_color(dist(e2), dist(e2), dist(e2), 0.2f);

	RenderEntity laser_line_plane = get_plane_entity(normal, d, common_color);

	float elongation_length = 200.f;
	cv::Vec3f elongated_line_b = line_a + elongation_length * (line_b - line_a);
	cv::Vec3f elongated_line_a = line_a - elongation_length * (line_b - line_a);
	RenderEntity camera_ray = get_line_entity(elongated_line_a, elongated_line_b, common_color);

	RenderEntity line_3d_points = get_points_entity(points_3d, common_color);

	RenderMesh reconstruction;
	reconstruction.push_back(laser_line_plane);
	reconstruction.push_back(line_3d_points);
	reconstruction.push_back(camera_ray);

	// make copies not references
	RenderMesh camera = assets_[CAMERA];
	update_model(camera, RT);

	RenderMesh robot = assets_[ROBOT];
	update_model(robot, RT);

	RenderMesh floor_grid = assets_[FLOOR_GRID];
	//update_model(floor_grid, RT);

	std::vector<RenderMesh> scene;
	scene.push_back(reconstruction);
	scene.push_back(camera);
	scene.push_back(robot);
	scene.push_back(floor_grid);

	frames_.push_back(scene);

	// add points for center view calculation
	points_.insert(points_.end(), points_3d.begin(), points_3d.end());


}

void RobotViewer::set_current_frame_to_draw(int frame_no) {
	current_frame_to_draw_ = frame_no;
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

void RobotViewer::load_obj(std::string filename,  std::vector<cv::Vec3f>& vertices,
	std::vector<cv::Vec3f>& normals, std::vector<cv::Vec2f>& uvs, std::vector<unsigned int>& indices,
	std::vector<int>& count, std::vector<int>& offset, std::vector<int>& base_index) {

	std::vector<unsigned int> glm_indices;
	std::vector<glm::vec3> glm_vertices;
	std::vector<glm::vec2> glm_uvs;
	std::vector<glm::vec3> glm_normals;
	//loadAssImp(filename.c_str(), glm_indices, glm_vertices, glm_uvs, glm_normals);

	std::vector<tinyobj::shape_t> shapes;
	std::vector < tinyobj::material_t> materials;
	std::string error;
	tinyobj::LoadObj(shapes, materials, error, filename.c_str());

	//std::vector<int> count;
	//std::vector<int> offset;

	
	const int no_of_vertices_per_polygon = 3;
	int total_offset = 0;
	int total_indices = 0;
	for (auto& shape : shapes) {
		auto& mesh = shape.mesh;

		assert(mesh.indices.size() % no_of_vertices_per_polygon == 0);
		assert(mesh.positions.size() % no_of_vertices_per_polygon == 0);

		for (size_t i = 0; i < (mesh.positions.size() / no_of_vertices_per_polygon); ++i) {
			glm::vec3 position(mesh.positions[(no_of_vertices_per_polygon * i) + 0], 
				mesh.positions[(no_of_vertices_per_polygon * i) + 1], 
				mesh.positions[(no_of_vertices_per_polygon * i) + 2]);
			glm::vec3 normal(mesh.normals[(no_of_vertices_per_polygon * i) + 0], 
				mesh.normals[(no_of_vertices_per_polygon * i) + 1], 
				mesh.normals[(no_of_vertices_per_polygon * i) + 2]);
			if (mesh.texcoords.size() > 0 && (mesh.texcoords.size() > 2 * i)) {
				glm::vec2 texcoord(mesh.texcoords[(2 * i) + 0],
					mesh.texcoords[(2 * i) + 1]);
				glm_uvs.push_back(texcoord);
			}
			glm_vertices.push_back(position);
			glm_normals.push_back(normal);
		}

		int no_of_vertices = mesh.positions.size() / 3;

		int indices_count = mesh.indices.size();
		for (size_t i = 0; i < indices_count; ++i) {
			glm_indices.push_back(mesh.indices[i]);
		}
		count.push_back(indices_count);
		offset.push_back(total_offset);
		base_index.push_back(total_indices);

		total_offset += (no_of_vertices);
		total_indices += indices_count;
	}
	
	for (auto& out_vertice : glm_vertices) {
		vertices.push_back(cv::Vec3f(static_cast<float>(out_vertice[0]), static_cast<float>(out_vertice[1])
			, static_cast<float>(out_vertice[2])));
	}

	for (auto& out_vertice : glm_normals) {
		normals.push_back(cv::Vec3f(static_cast<float>(out_vertice[0]), static_cast<float>(out_vertice[1])
			, static_cast<float>(out_vertice[2])));
	}

	for (auto& out_vertice : glm_uvs) {
		uvs.push_back(cv::Vec2f(static_cast<float>(out_vertice[0]), static_cast<float>(out_vertice[1])));
	}

	indices = glm_indices;
}

void RobotViewer::load_obj(std::string filename, VertexBufferData& vertex_buffer_data) {
	load_obj(filename, vertex_buffer_data.positions, vertex_buffer_data.normals, vertex_buffer_data.uvs,
		vertex_buffer_data.indices, vertex_buffer_data.count, vertex_buffer_data.offset, vertex_buffer_data.base_index);
}

glm::mat4 RobotViewer::convert(cv::Mat& input_mat) {
	cv::Mat float_mat;
	input_mat.convertTo(float_mat, CV_32F);
	std::vector<float> array;
	if (float_mat.isContinuous()) {
		array.assign((float*)float_mat.datastart, (float*)float_mat.dataend);
	}
	else {
		for (int i = 0; i < float_mat.rows; ++i) {
			array.insert(array.end(), (float*)float_mat.ptr<uchar>(i), (float*)float_mat.ptr<uchar>(i)+float_mat.cols);
		}
	}
	glm::mat4 model = glm::transpose(glm::make_mat4(&array[0]));
	return model;
}

cv::Mat RobotViewer::convert_mat(glm::mat3& input_mat) {

	cv::Mat mat(3, 3, CV_32F, glm::value_ptr(input_mat));
	cv::Mat transposed_mat = mat.t();
	return transposed_mat;
}

RenderEntity RobotViewer::get_line_entity(cv::Vec3f a, cv::Vec3f b, cv::Vec4f line_color, cv::Mat& model, RenderEntity::Type type) {
	makeCurrent();

	std::vector<cv::Vec4f> line_colors;
	for (int i = 0; i < 2; ++i) {
		line_colors.push_back(line_color);
	}

	std::vector<cv::Vec3f> line_points;
	line_points.push_back(a);
	line_points.push_back(b);

	RenderEntity points(GL_LINES, &m_shader);
	points.set_type(type);
	points.set_model(convert(model));
	std::vector<cv::Vec3f> normals;
	points.upload_data_to_gpu(line_points, line_colors, normals);

	return points;
}

RenderEntity RobotViewer::get_line_entity(cv::Vec3f a, cv::Vec3f b, cv::Vec4f line_color, RenderEntity::Type type) {
	cv::Mat identity = cv::Mat::eye(4, 4, CV_64F);
	RenderEntity line_entity = get_line_entity(a, b, line_color, identity, type);
	return line_entity;
}

RenderEntity RobotViewer::get_plane_entity(cv::Vec3f normal, double d, cv::Vec4f plane_color) {
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

	RenderEntity plane(GL_TRIANGLE_STRIP, &m_shader);
	plane.set_type(RenderEntity::Plane);
	std::vector<cv::Vec3f> normals;
	plane.upload_data_to_gpu(plane_points, plane_colors, normals);

	return plane;

}

RenderEntity RobotViewer::get_points_entity(std::vector<cv::Vec3f> points_3d, cv::Vec4f point_color) {
	makeCurrent();

	std::vector<cv::Vec4f> point_colors;
	for (int i = 0; i < points_3d.size(); ++i) {
		point_colors.push_back(point_color);
	}

	RenderEntity points(GL_POINTS, &m_shader);
	points.set_type(RenderEntity::Points);
	std::vector<cv::Vec3f> normals;
	points.upload_data_to_gpu(points_3d, point_colors, normals);

	return points;
}

void RobotViewer::update_model(RenderMesh& mesh, cv::Mat RT) {
	for (auto& entity : mesh) {
		entity.set_model(convert(RT) * entity.get_model());
	}
}

void RobotViewer::update_model(RenderMesh& mesh, glm::mat4 RT) {
	for (auto& entity : mesh) {
		entity.set_model((RT) * entity.get_model());
	}
}

void RobotViewer::draw_camera(cv::Mat& model) {
	makeCurrent();
	RenderEntity camera(GL_TRIANGLES, &m_shader);
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
	glm::mat4 converted_mat = convert(model);
	glm::mat4 final_model = glm::scale(glm::mat4(converted_mat), glm::vec3(scale, scale, scale));
	camera.set_model(final_model);
	camera.upload_data_to_gpu(vertices, colors, normals);
	//entities_.push_back(camera);

	// draw x, y, z lines
	cv::Vec3f origin(0.f, 0.f, 0.f);

	const float multiplier = 100.f;
	cv::Vec3f x_point(multiplier * 1.f, 0.f, 0.f);
	cv::Vec4f x_color(1.f, 0.f, 0.f, 1.f );

	cv::Vec3f y_point(0.f, multiplier * 1.f, 0.f);
	cv::Vec4f y_color(0.f, 1.f, 0.f, 1.f);

	cv::Vec3f z_point(0.f, 0.f, multiplier * 1.f);
	cv::Vec4f z_color(0.f, 0.f, 1.f, 1.f);

	RenderEntity xline = get_line_entity(origin, x_point, x_color, model, RenderEntity::Default);
	RenderEntity yline = get_line_entity(origin, y_point, y_color, model, RenderEntity::Default);
	RenderEntity zline = get_line_entity(origin, z_point, z_color, model, RenderEntity::Default);

	RenderMesh mesh;
	mesh.push_back(camera);
	mesh.push_back(xline);
	mesh.push_back(yline);
	mesh.push_back(zline);

	assets_[CAMERA] = mesh;

}

void RobotViewer::draw_table_top(cv::Mat& model) {
	makeCurrent();

	cv::Vec4f green(0.f, 1.f, 0.f, 1.f);

	float y = -160.f;
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

	RenderEntity grid_floor(GL_LINES, &m_shader);
	grid_floor.set_type(RenderEntity::Default);
	grid_floor.upload_data_to_gpu(points, colors, normals);

	RenderMesh mesh;
	mesh.push_back(grid_floor);

	assets_[FLOOR_GRID] = mesh;
}

void RobotViewer::draw_robot(cv::Mat& world) {
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

	RenderEntity robot(GL_TRIANGLES, &m_shader);
	robot.set_type(RenderEntity::Default);
	float scale = 50.f;
	glm::mat4 model_transform;
	model_transform = glm::translate(model_transform, glm::vec3(20.f, -60.f, 0.f));
	model_transform = glm::scale(model_transform, glm::vec3(scale * 2.5f, scale, scale));
	robot.set_model(convert(world) * model_transform);

	robot.upload_data_to_gpu(vertices, colors, normals);
	//entities_.push_back(robot);
	
	RenderMesh mesh;
	mesh.push_back(robot);

	assets_[ROBOT] = mesh;
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

	//std::cout << m_xRot << ", " << m_yRot << std::endl;
	if (mouse_down_) {
		int dx = event->x() - m_lastPos.x();
		int dy = event->y() - m_lastPos.y();
		

		float mouse_drag_sensitivity = 1;
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

void RobotViewer::load_inital_models() {
	cv::Mat unit_matrix = cv::Mat::eye(4, 4, CV_64F);

	draw_camera(unit_matrix);
	draw_table_top(unit_matrix);
	draw_robot(unit_matrix);

	// create default scene
	default_scene_.push_back(assets_[ROBOT]);
	default_scene_.push_back(assets_[CAMERA]);
	default_scene_.push_back(assets_[FLOOR_GRID]);
}
