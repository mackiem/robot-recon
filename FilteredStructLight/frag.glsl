#version 330


in vec2 fragCoord;
uniform sampler2D tex;		// Texture
uniform float threshold;
uniform int toggle_threshold;
out vec4 fragColor;

void main(void)
{
	vec4 tex_data = texture(tex, fragCoord);
	if (toggle_threshold == 1) {
		if (tex_data.r > threshold) {
			fragColor = vec4(1.0);
		}
		else {
			fragColor = vec4(0.0);
		}
	}
	else {
		fragColor = vec4(tex_data.rrr, 1.0);
	}
}