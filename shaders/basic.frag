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
uniform vec3 uLightDir = vec3(0.5, 1.0, 0.5);  // Directional light direction
uniform vec3 uLightColor = vec3(1.0, 1.0, 1.0); // Light color
uniform float uAmbient = 0.3;                   // Ambient light strength

void main() {
    // Normalize inputs
    vec3 normal = normalize(vNormal);
    vec3 lightDir = normalize(uLightDir);

    // Diffuse lighting (Lambertian)
    float diff = max(dot(normal, lightDir), 0.0);

    // Combine ambient and diffuse
    vec3 lighting = (uAmbient + (1.0 - uAmbient) * diff) * uLightColor;

    // Final color
    FragColor = vec4(vColor * lighting, 1.0);
}
