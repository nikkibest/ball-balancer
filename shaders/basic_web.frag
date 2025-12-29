#version 300 es

/**
 * Basic Fragment Shader - WebGL 2.0 Compatible
 *
 * Simple directional lighting with ambient term.
 * Suitable for visualization purposes.
 *
 * GLSL ES 3.0 differences from desktop GLSL 4.5:
 * - Must specify precision qualifiers
 * - Use 'out' instead of implicit FragColor
 */

// Medium precision is sufficient for fragment calculations
precision mediump float;

// Inputs from vertex shader
in vec3 vPosition;  // World-space position
in vec3 vNormal;    // World-space normal
in vec3 vColor;     // Vertex color

// Output
out vec4 FragColor;

// Lighting uniforms
uniform vec3 uColor;        // Object color
uniform vec3 uLightPos;     // Light position
uniform vec3 uLightColor;   // Light color (default: white)
uniform float uAmbient;     // Ambient light strength (default: 0.3)

void main() {
    // Normalize inputs
    vec3 normal = normalize(vNormal);
    vec3 lightDir = normalize(uLightPos - vPosition);

    // Diffuse lighting (Lambertian)
    float diff = max(dot(normal, lightDir), 0.0);

    // Combine ambient and diffuse
    vec3 lighting = (uAmbient + (1.0 - uAmbient) * diff) * uLightColor;

    // Final color
    FragColor = vec4(uColor * lighting, 1.0);
}
