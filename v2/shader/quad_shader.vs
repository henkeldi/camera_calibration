#version 450

out vec2 texCoord;

layout (location=0) uniform mat4 T;

void main(void){
	const vec4 vertices[] = vec4[](	vec4(-1, -1, 0, 1.0),
										vec4(-1, 9, 0, 1.0),
										vec4(7, 9, 0, 1.0),
										vec4(7, -1, 0, 1.0));
	const vec2 TexCoords[] = vec2[](	vec2(1.0, 1.0),
										vec2(0.0, 1.0),
										vec2(1.0, 0.0),
										vec2(0.0, 0.0));
	gl_Position = T * vertices[gl_VertexID];
	texCoord = TexCoords[gl_VertexID];
}