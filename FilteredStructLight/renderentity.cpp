#include "renderentity.h"

#define TINYOBJLOADER_IMPLEMENTATION
#include "tiny_obj_loader.h"

RenderEntity::RenderEntity(GLenum primitive, QGLShaderProgram* shader) : primitive_(primitive), shader_(shader), vao_(0), using_indices_(false) {
	model_ = glm::mat4(1.f);
	model_loc_ = shader_->uniformLocation("model");
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

glm::mat4 RenderEntity::get_model() {
	return model_;
}

void RenderEntity::set_initial_model(glm::mat4 initial_model) {
	initial_model_ = initial_model;
}

glm::mat4 RenderEntity::get_initial_model() {
	return initial_model_;
}

void RenderEntity::upload_data_to_gpu(std::vector<cv::Vec3f>& vertices, std::vector<cv::Vec4f>& colors,
	std::vector<cv::Vec3f>& normals) {

	shader_->bind();

	glGenVertexArrays(1, &vao_);
	glBindVertexArray(vao_);


	glGenBuffers(2, vbo_);

	glBindBuffer(GL_ARRAY_BUFFER, vbo_[0]);
	glBufferData(GL_ARRAY_BUFFER, vertices.size() * sizeof(cv::Vec3f), &vertices[0], GL_STATIC_DRAW);


	GLuint vert_pos_attr = shader_->attributeLocation("vertex");

	glEnableVertexAttribArray(vert_pos_attr);
	glVertexAttribPointer(vert_pos_attr, 3, GL_FLOAT, GL_FALSE, sizeof(cv::Vec3f), NULL);

	glBindBuffer(GL_ARRAY_BUFFER, vbo_[1]);
	glBufferData(GL_ARRAY_BUFFER, colors.size() * sizeof(cv::Vec4f), &colors[0], GL_STATIC_DRAW);

	GLuint vert_color_attr = shader_->attributeLocation("vertColor");

	glEnableVertexAttribArray(vert_color_attr);
	glVertexAttribPointer(vert_color_attr, 4, GL_FLOAT, GL_FALSE, sizeof(cv::Vec4f), NULL);

	count_ = vertices.size();

	//is_draw_triangles_ = false;
	//GLint texture_used_loc = m_shader.uniformLocation("is_texture_used");
	//glUniform1i(texture_used_loc, is_draw_triangles_);

	glBindVertexArray(NULL);
	
}

void RenderEntity::upload_data_to_gpu(std::vector<cv::Vec3f>& vertices, std::vector<cv::Vec4f>& colors,
	std::vector<cv::Vec3f>& normals,
	std::vector<cv::Vec2f>& uvs, std::vector<unsigned int>& indices,
	std::vector<int>& count, std::vector<int>& offset, std::vector<int>& base_index, bool reuse) {

	shader_->bind();

	if (!reuse) {
		glGenVertexArrays(1, &vao_);
		glGenBuffers(5, vbo_);
	}

	glBindVertexArray(vao_);

	glBindBuffer(GL_ARRAY_BUFFER, vbo_[POSITION]);
	glBufferData(GL_ARRAY_BUFFER, vertices.size() * sizeof(cv::Vec3f), &vertices[0], GL_STATIC_DRAW);

	glEnableVertexAttribArray(POSITION);
	glVertexAttribPointer(POSITION, 3, GL_FLOAT, GL_FALSE, sizeof(cv::Vec3f), NULL);

	glBindBuffer(GL_ARRAY_BUFFER, vbo_[COLOR]);
	glBufferData(GL_ARRAY_BUFFER, colors.size() * sizeof(cv::Vec4f), &colors[0], GL_STATIC_DRAW);

	glEnableVertexAttribArray(COLOR);
	glVertexAttribPointer(COLOR, 4, GL_FLOAT, GL_FALSE, sizeof(cv::Vec4f), NULL);

	if (normals.size() > 0) {
		glBindBuffer(GL_ARRAY_BUFFER, vbo_[NORMAL]);
		glBufferData(GL_ARRAY_BUFFER, normals.size() * sizeof(cv::Vec3f), &normals[0], GL_STATIC_DRAW);
		glEnableVertexAttribArray(NORMAL);
		glVertexAttribPointer(NORMAL, 3, GL_FLOAT, GL_FALSE, sizeof(cv::Vec3f), NULL);
	}

	if (uvs.size() > 0) {
		glBindBuffer(GL_ARRAY_BUFFER, vbo_[UV]);
		glBufferData(GL_ARRAY_BUFFER, uvs.size() * sizeof(cv::Vec2f), &uvs[0], GL_STATIC_DRAW);
		glEnableVertexAttribArray(UV);
		glVertexAttribPointer(UV, 2, GL_FLOAT, GL_FALSE, sizeof(cv::Vec2f), NULL);
	}

	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, vbo_[INDEX]);
	glBufferData(GL_ELEMENT_ARRAY_BUFFER, indices.size() * sizeof(unsigned int), &indices[0], GL_STATIC_DRAW);

	using_indices_ = true;

	//count_ = indices.size();
	element_count_ = count;
	element_offset_ = offset;
	element_base_index_ = base_index;

	assert(element_count_.size() == element_offset_.size());

	//is_draw_triangles_ = false;
	//GLint texture_used_loc = m_shader.uniformLocation("is_texture_used");
	//glUniform1i(texture_used_loc, is_draw_triangles_);

	glBindVertexArray(NULL);
}

void RenderEntity::upload_data_to_gpu(VertexBufferData& vertex_buffer_data, bool reuse) {
	upload_data_to_gpu(vertex_buffer_data.positions, vertex_buffer_data.colors, vertex_buffer_data.normals, vertex_buffer_data.uvs,
		vertex_buffer_data.indices, vertex_buffer_data.count, vertex_buffer_data.offset, vertex_buffer_data.base_index, reuse);

}

void RenderEntity::draw() {
	shader_->bind();
	glBindVertexArray(vao_);
	if (using_indices_) {
		for (size_t i = 0; i < element_count_.size(); ++i) {
			//glDrawElements(GL_TRIANGLES, element_count_[i], GL_UNSIGNED_INT, element_offset_[i]);
			glDrawElementsBaseVertex(primitive_, element_count_[i], GL_UNSIGNED_INT, (GLvoid*) (sizeof(unsigned int) * (element_base_index_[i])), 
				static_cast<GLint>(element_offset_[i]));
		}
	} else {
		glDrawArrays(primitive_, 0, count_);
	}
}

VisObject::VisObject(UniformLocations& locations) : locations_(locations) {
}

void VisObject::update(glm::mat4 global_model) {
}

void VisObject::draw(glm::mat4 global_model, glm::mat4 camera, glm::mat4 projection) {
	for (auto& entity : mesh_) {
		// model
		glm::mat4 transformed_model = global_model * entity.model_;
		glUniformMatrix4fv(locations_.model_loc_, 1, GL_FALSE, glm::value_ptr(transformed_model));

		// normal matrix
		glm::mat3 inverse_transpose_model = glm::transpose(glm::inverse(glm::mat3(transformed_model)));
		glUniformMatrix3fv(locations_.inverse_transpose_loc_, 1, GL_FALSE, glm::value_ptr(inverse_transpose_model));


		glm::mat4 mvp_matrix = projection * camera * transformed_model;
		glUniformMatrix4fv(locations_.mvp_loc_, 1, GL_FALSE, glm::value_ptr(mvp_matrix));


		entity.draw();
	}

}
