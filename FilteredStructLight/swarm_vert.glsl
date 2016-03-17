#version 400

layout(location = 0) in vec3 position;
layout(location = 1) in vec4 color;
layout(location = 2) in vec3 normal;
layout(location = 3) in vec2 uv;

out vec3 frag_normal;
out vec4 frag_color;
out vec2 frag_uv;
out vec3 frag_position;

uniform mat4 mvp;
uniform mat3 inverse_transpose_model;
uniform mat4 model;

void main() {
	frag_normal = normalize(inverse_transpose_model * normal);
	frag_color = color;
	frag_uv = uv;
	//vec4 homogenous_position = model * vec4(position, 1.0);
	//frag_position = vec3(homogenous_position) / homogenous_position.w;
	frag_position = vec3(model * vec4(position, 1.0));
	gl_Position = mvp * vec4(position, 1.0);
}