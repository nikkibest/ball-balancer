#version 300 es

/**
 * Basic Vertex Shader - WebGL 2.0 Compatible
 *
 * Transforms vertices using MVP matrix and passes data to fragment shader.
 * Supports simple directional lighting calculation.
 *
 * GLSL ES 3.0 differences from desktop GLSL 4.5:
 * - Must specify precision qualifiers
 * - Limited built-in functions
 * - Stricter type conversions
 */

// High precision for vertex positions
precision highp float;

// Vertex attributes
layout(location = 0) in vec3 aPosition;  // Vertex position
layout(location = 1) in vec3 aNormal;    // Vertex normal
layout(location = 2) in vec3 aColor;     // Vertex color

// Uniforms
uniform mat4 uModel;        // Model matrix
uniform mat4 uView;         // View matrix
uniform mat4 uProjection;   // Projection matrix
uniform mat3 uNormalMatrix; // Normal matrix (transpose(inverse(model)))

// Outputs to fragment shader
out vec3 vPosition;  // World-space position
out vec3 vNormal;    // World-space normal
out vec3 vColor;     // Vertex color

void main() {
    // Transform position to clip space
    gl_Position = uProjection * uView * uModel * vec4(aPosition, 1.0);

    // Pass world-space position
    vPosition = vec3(uModel * vec4(aPosition, 1.0));

    // Transform normal to world space
    vNormal = normalize(uNormalMatrix * aNormal);

    // Pass color through
    vColor = aColor;
}
