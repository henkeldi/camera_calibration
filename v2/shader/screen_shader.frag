#version 450

#extension GL_NV_bindless_texture : require

layout (binding=2, std430) buffer Textures {
	sampler2D samplers[];
};

layout (location = 0) out vec4 color;

in vec2 texCoord;

void main(void) {
	color = texture( samplers[0], texCoord );
}