#version 450

#extension GL_NV_bindless_texture : require

layout (location = 1) uniform vec4 inColor;

layout (location = 0) out vec4 color;

in vec2 texCoord;

void main(void) {
	color = inColor; //vec4(0.3,0.4,0.7,0.4);//texture( samplers[0], texCoord );
}