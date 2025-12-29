#version 450 core

/**
 * Grid Vertex Shader
 *
 * Simple pass-through for grid floor rendering.
 */

layout(location = 0) in vec3 aPosition;

uniform mat4 uModel;        // Model matrix
uniform mat4 uView;         // View matrix
uniform mat4 uProjection;   // Projection matrix

void main() {
    gl_Position = uProjection * uView * uModel * vec4(aPosition, 1.0);
}
