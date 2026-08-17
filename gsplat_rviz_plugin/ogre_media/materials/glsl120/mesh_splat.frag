#version 330 core

// Opaque mesh splat: colour is the per-vertex SH result, Gouraud-interpolated.
in  vec3 vColor;
out vec4 frag_color;

void main()
{
    frag_color = vec4(vColor, 1.0);
}
