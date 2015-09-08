#version 330


in vec2 fragCoord;
uniform sampler2D tex;		// Texture
out vec4 fragColor;

void main(void)
{
	vec4 tex_data = texture(tex, fragCoord);
	fragColor = vec4(tex_data.rrr, 1.0);
}