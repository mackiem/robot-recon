#version 330


in vec4 fragColor;
in vec2 fragCoords;

out vec4 final_color;

uniform int is_texture_used;
uniform sampler2D tex;

const float ambient = 0.3f;

void main(void)
{
	if (is_texture_used == 1) {
		final_color = texture(tex, fragCoords);
	} else {
		final_color = vec4(fragColor);
    }
}