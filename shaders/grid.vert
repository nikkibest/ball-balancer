#version 450 core

/**
 * Grid Vertex Shader
 *
 * Simple pass-through for grid floor rendering.
 */

layout(location = 0) in vec3 aPosition;

uniform mat4 uMVP;  // Model-View-Projection matrix

void main() {
    gl_Position = uMVP * vec4(aPosition, 1.0);
}
