#version 300 es

/**
 * Grid Fragment Shader - WebGL 2.0 Compatible
 *
 * Simple color output for grid lines.
 */

precision mediump float;

// Input from vertex shader
in vec3 vColor;

// Output
out vec4 FragColor;

// Uniform color override
uniform vec3 uColor;

void main() {
    FragColor = vec4(uColor, 1.0);
}
