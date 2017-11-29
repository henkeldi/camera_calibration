#version 450 core

#extension GL_NV_bindless_texture : require
#extension GL_ARB_shader_draw_parameters : require

layout (location = 0) in vec3 position;
layout (location = 1) in vec3 normal;

struct DirLight {
	vec3 light_pos;
	vec3 ambient;
	vec3 diffuse;
	vec3 specular;
};

struct Material {
	vec3 ambient;
	vec3 diffuse;
	vec3 specular;
	float shininess;
};

struct SceneData {
	mat4 T_view_world;
	mat4 T_proj_view;
	mat4 T_proj_world;
	DirLight dirLight;
	Material material;
};

layout (binding = 0, std430) readonly buffer SceneSSBO {
	SceneData scene;
};

struct ObjectData {
	mat4 T_world_object;
};

layout (binding = 1, std430) readonly buffer ObjectSSBO {
	ObjectData objects[];
};

out VS_OUT {
	smooth vec3 Normal;
	smooth vec3 LightDir;
	smooth vec3 ViewDir;
	ObjectData object;
} vs_out;

void main(void) {
	ObjectData object = objects[0];
	mat4 mv_matrix = scene.T_view_world * object.T_world_object;

	vec4 P = mv_matrix * vec4(position, 1.0);
	
	vs_out.Normal = mat3(mv_matrix) * normalize(normal);
	vs_out.LightDir = scene.dirLight.light_pos - P.xyz;
	vs_out.ViewDir = - P.xyz;
	vs_out.object = object;

	gl_Position = scene.T_proj_view * P;
}