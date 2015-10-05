#version 330

in vec3 vertex;
in vec3 vertColor;
in vec2 vertCoords;

out vec3 fragColor;
out vec2 fragCoords;

uniform mat4 model;
uniform mat4 projection;
uniform mat4 camera;

void main(void)
{
	fragColor = vertColor;
	fragCoords = vertCoords;
	gl_Position = projection * camera * model * vec4(vertex, 1);
}