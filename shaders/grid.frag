#version 450 core

/**
 * Grid Fragment Shader
 *
 * Renders grid lines in a fixed color.
 */

out vec4 FragColor;

uniform vec3 uColor;  // Grid line color

void main() {
    FragColor = vec4(uColor, 1.0);
}
