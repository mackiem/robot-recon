#version 330

in vec3 vertex;
in vec3 vertColor;
out vec3 fragColor;

uniform mat4 model;
uniform mat4 projection;
uniform mat4 camera;

void main(void)
{
	fragColor = vertColor;
	gl_Position = projection * camera * model * vec4(vertex, 1);
}