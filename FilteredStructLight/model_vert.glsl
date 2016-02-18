#version 330

in vec3 vertex;
in vec4 vertColor;
in vec2 vertCoords;
in vec3 vertNormal;

out vec4 fragColor;
out vec2 fragCoords;
out vec4 fragNormal;

uniform mat4 model;
uniform mat4 projection;
uniform mat4 camera;

void main(void)
{
	fragColor = vertColor;
	fragCoords = vertCoords;
	gl_Position = projection * camera * model * vec4(vertex, 1);
	fragNormal = normalize(model * vec4(vertNormal, 0));
}