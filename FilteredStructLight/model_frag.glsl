#version 330


in vec3 fragColor;
out vec4 final_color;

void main(void)
{
	final_color = vec4(fragColor, 1.0);
}