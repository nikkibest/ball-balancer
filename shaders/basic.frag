#version 450 core

/**
 * Basic Fragment Shader
 *
 * Simple directional lighting with ambient term.
 * Suitable for visualization purposes.
 */

// Inputs from vertex shader
in vec3 vPosition;  // World-space position
in vec3 vNormal;    // World-space normal
in vec3 vColor;     // Vertex color

// Output
out vec4 FragColor;

// Lighting uniforms
uniform vec3 uColor;                            // Object color
uniform vec3 uLightPos;                         // Light position
uniform vec3 uLightColor = vec3(1.0, 1.0, 1.0); // Light color
uniform float uAmbient = 0.3;                   // Ambient light strength

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
