#version 450 core

/**
 * Grid Fragment Shader
 *
 * Renders grid lines in a fixed color.
 */

out vec4 FragColor;

uniform vec4 uColor = vec4(0.5, 0.5, 0.5, 1.0);  // Grid line color (gray)

void main() {
    FragColor = uColor;
}
