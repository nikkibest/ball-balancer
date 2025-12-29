#version 300 es

/**
 * Grid Vertex Shader - WebGL 2.0 Compatible
 *
 * Simple pass-through shader for grid lines.
 */

precision highp float;

// Vertex attributes
layout(location = 0) in vec3 aPosition;
layout(location = 1) in vec3 aNormal;
layout(location = 2) in vec3 aColor;

// Uniforms
uniform mat4 uModel;
uniform mat4 uView;
uniform mat4 uProjection;

// Outputs
out vec3 vColor;

void main() {
    gl_Position = uProjection * uView * uModel * vec4(aPosition, 1.0);
    vColor = aColor;
}
