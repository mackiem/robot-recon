#version 330

in vec4 vertex;
in vec2 vertCoord;
out vec2 fragCoord;

uniform mat4 model;

void main(void)
{
	fragCoord = vertCoord;
	gl_Position = model * vertex;
}