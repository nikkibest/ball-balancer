#version 330 core

in vec3 WorldPos;

uniform vec3 uColor;

out vec4 FragColor;

void main() {
    // Simple grid pattern
    float gridSize = 0.1;
    vec2 coord = WorldPos.xz / gridSize;
    vec2 grid = abs(fract(coord - 0.5) - 0.5) / fwidth(coord);
    float line = min(grid.x, grid.y);
    float alpha = 1.0 - min(line, 1.0);

    // Fade grid in the distance
    float dist = length(WorldPos);
    alpha *= smoothstep(10.0, 2.0, dist);

    FragColor = vec4(uColor, alpha * 0.3);
}
