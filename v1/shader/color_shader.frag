#version 450 core

#extension GL_NV_bindless_texture : require

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

in VS_OUT {
	smooth vec3 Normal;
	smooth vec3 LightDir;
	smooth vec3 ViewDir;
	flat ObjectData object;
} fs_in;

layout (location = 0) out vec4 color;

void main(void) {
	ObjectData object = fs_in.object;
	vec3 Normal = normalize(fs_in.Normal);
	vec3 LightDir = normalize(fs_in.LightDir);
	vec3 ViewDir = normalize(fs_in.ViewDir);

	vec3 ambient = scene.material.ambient;
	vec3 diffuse = max(dot(Normal, LightDir), 0.0) * scene.material.diffuse;
	vec3 R = reflect(-LightDir, Normal);
	vec3 specular = pow(max(dot(R, ViewDir), 0.0), scene.material.shininess) * scene.material.specular;

	color = vec4( 	scene.dirLight.ambient  * ambient + 
					scene.dirLight.diffuse  * diffuse + 
					scene.dirLight.specular *specular, 1.0 );
	color = vec4(1.0,0,0,1);
}