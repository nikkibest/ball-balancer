#version 330 core

layout(location = 0) in vec3 aPos;

uniform mat4 uModel;
uniform mat4 uView;
uniform mat4 uProjection;

out vec3 WorldPos;

void main() {
    WorldPos = vec3(uModel * vec4(aPos, 1.0));
    gl_Position = uProjection * uView * vec4(WorldPos, 1.0);
}
