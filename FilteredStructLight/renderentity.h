#pragma once
#include "fsl_common.h"
#include "gl_core_3_3.h"

#include <QGLShaderProgram>
#include <QGLWidget>
#include <QGLBuffer>
#include <QGLShaderProgram>
#include "octree/octree.h"


struct VertexBufferData {
	std::vector<cv::Vec3f> positions;
	std::vector<cv::Vec3f> normals;
	std::vector<cv::Vec4f> colors;
	std::vector<cv::Vec2f> uvs;
	std::vector<unsigned int> indices;
	std::vector<int> count;
	std::vector<int> offset;
	std::vector<int> base_index;
};

class RenderEntity {

public:
	GLenum primitive_;
	QGLShaderProgram* shader_;
	GLuint vao_;
	GLuint vbo_[5];
	GLint count_;
	GLint model_loc_;
	bool using_indices_;
	std::vector<int> element_count_;
	std::vector<int> element_offset_;
	std::vector<int> element_base_index_;

	enum Type {
		Plane = 0,
		Points = 1,
		Lines = 2,
		Default = 100
	};

	enum {
		POSITION = 0,
		COLOR = 1,
		NORMAL = 2,
		UV = 3,
		INDEX = 4
	};

	glm::mat4 initial_model_;
	glm::mat4 model_;
	RenderEntity(GLenum primitive, QGLShaderProgram* shader);

	void set_type(Type type);
	Type get_type();
	void set_model(glm::mat4 model);
	glm::mat4 get_model();
	void set_initial_model(glm::mat4 initial_model);
	glm::mat4 get_initial_model();

	void upload_data_to_gpu(std::vector<cv::Vec3f>& vertices, std::vector<cv::Vec4f>& colors,
		std::vector<cv::Vec3f>& normals);

	void upload_data_to_gpu(std::vector<cv::Vec3f>& vertices, std::vector<cv::Vec4f>& colors,
		std::vector<cv::Vec3f>& normals,
		std::vector<cv::Vec2f>& uvs, std::vector<unsigned int>& indices,
		std::vector<int>& count, std::vector<int>& offset, std::vector<int>& base_index, bool reuse = false);

	void upload_data_to_gpu(VertexBufferData& vertex_buffer_data, bool reuse = false);

	void draw();

	~RenderEntity();

private:
	RenderEntity::Type type_;
};

typedef std::vector<RenderEntity> RenderMesh;
typedef std::unordered_map<std::string, RenderMesh> Assets;
typedef std::vector<RenderMesh> Scene;

struct UniformLocations {
	GLint model_loc_;
	GLint inverse_transpose_loc_;
	GLint mvp_loc_;
};

class VisObject {
protected:
	UniformLocations& locations_;
public:
	void clear_gpu_structs();
	virtual ~VisObject();
	RenderMesh mesh_;
	VisObject(UniformLocations& locations);
	virtual void update(glm::mat4 global_model);
	virtual void draw(glm::mat4 global_model, glm::mat4 camera, glm::mat4 projection);
};

