#version 400

#define NO_OF_LIGHTS 1

in vec3 frag_normal;
in vec4 frag_color;
in vec2 frag_uv;
in vec3 frag_position;

out vec4 final_color;

struct Light {
	vec3 position;
	vec3 color;
};

uniform sampler2D tex;
uniform Light lights[NO_OF_LIGHTS];
uniform vec3 view_position;

const vec3 ambient_light = vec3(0.3f, 0.3f, 0.3f);
//const vec4 ambient_light = vec4(0.0f, 0.0f, 0.0f, 1.f);
const float specular_exponent = 5.f;
const vec3 spec_color = vec3(1.f, 1.f, 1.f);

void main() {
	vec3 normal = normalize(frag_normal);

	vec4 diffuse_component;
	vec4 specular_component;

	vec3 final_light_color;

    for (int i = 0; i < NO_OF_LIGHTS; ++i) {
		Light light = lights[i];
		vec3 light_direction = normalize(light.position - frag_position);

		// diffuse
		float lambertian = max(dot(light_direction, normal), 0.f);
		//float lambertian = abs(dot(light_direction, normal));
		float specular = 0.f;

		if (lambertian > 0.0) {
			vec3 view_direction = normalize(view_position - frag_position);
            // specular
			vec3 reflected_light_direction = reflect(-light_direction, normal);
		    float specular_base = max(dot(reflected_light_direction, view_direction), 0.f);
			specular = pow(specular_base, specular_exponent);
		}

		vec3 diffuse_color = vec3(frag_color.xyz *  light.color * lambertian);
		//vec4 diffuse_color = frag_color;
		vec3 specular_color = spec_color * specular;
		//final_light_color += diffuse_color + specular_color;
		final_light_color += diffuse_color;
    }

	final_color  = vec4(final_light_color + ambient_light, frag_color.a);
}
