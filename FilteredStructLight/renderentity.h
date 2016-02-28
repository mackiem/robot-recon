#pragma once
#include "fsl_common.h"
#include "gl_core_3_3.h"

#include <QGLShaderProgram>
#include <QGLWidget>
#include <QGLBuffer>
#include <QGLShaderProgram>


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

private:
	GLenum primitive_;
	QGLShaderProgram* shader_;
	GLuint vao_;
	GLuint vbo_[2];
	GLint count_;
	GLint model_loc_;
	bool using_indices_;
	std::vector<int> element_count_;
	std::vector<int> element_offset_;
	std::vector<int> element_base_index_;

public:
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

	glm::mat4 model_;
	RenderEntity(GLenum primitive, QGLShaderProgram* shader);

	void set_type(Type type);
	Type get_type();
	void set_model(glm::mat4 model);
	glm::mat4 get_model();

	void upload_data_to_gpu(std::vector<cv::Vec3f>& vertices, std::vector<cv::Vec4f>& colors,
		std::vector<cv::Vec3f>& normals);

	void upload_data_to_gpu(std::vector<cv::Vec3f>& vertices, std::vector<cv::Vec4f>& colors,
		std::vector<cv::Vec3f>& normals,
		std::vector<cv::Vec2f>& uvs, std::vector<unsigned int>& indices,
		std::vector<int>& count, std::vector<int>& offset, std::vector<int>& base_index);

	void upload_data_to_gpu(VertexBufferData& vertex_buffer_data);

	void draw();

private:
	RenderEntity::Type type_;
};
