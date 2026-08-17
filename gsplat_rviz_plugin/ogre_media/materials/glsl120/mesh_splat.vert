#version 330 core

// Mesh splat: an opaque triangle mesh whose vertices carry SH colour.
// Per-vertex SH is evaluated here (Gouraud) using the object-space view
// direction, matching the Mesh-Splatting reference rasteriser.

layout(location = 0) in vec3 position;   // object-space vertex position

// Per-vertex SH coefficients, coefficient-major RGB, RGBA16F texels:
//   vertex v, coeff c → texel (v * u_sh_stride + c), coeff 0 = DC.
uniform samplerBuffer u_mesh_sh;

uniform mat4  worldViewProj;
uniform vec3  camPosObj;     // camera position in object space
uniform int   sh_degree;     // active SH degree (0..3)
uniform int   u_sh_stride;   // coeffs per vertex = (max_degree+1)²

out vec3 vColor;

// ── SH constants ────────────────────────────────────────────────────────────
const float SH_C0    =  0.28209479177387814;
const float SH_C1    =  0.4886025119029199;
const float SH_C2_0  =  1.0925484305920792;
const float SH_C2_1  = -1.0925484305920792;
const float SH_C2_2  =  0.31539156525252005;
const float SH_C2_3  = -1.0925484305920792;
const float SH_C2_4  =  0.5462742152960396;
const float SH_C3_0  = -0.5900435899266435;
const float SH_C3_1  =  2.890611442640554;
const float SH_C3_2  = -0.4570457994644658;
const float SH_C3_3  =  0.3731763325901154;
const float SH_C3_4  = -0.4570457994644658;
const float SH_C3_5  =  1.445305721320277;
const float SH_C3_6  = -0.5900435899266435;

vec3 fetchSH(int base, int k) { return texelFetch(u_mesh_sh, base + k).rgb; }

void main()
{
    int base = gl_VertexID * u_sh_stride;

    // DC term: SH_C0 * f_dc + 0.5
    vec3 rgb = SH_C0 * fetchSH(base, 0) + vec3(0.5);

    if (sh_degree > 0) {
        vec3 d = normalize(position - camPosObj);

        rgb += -SH_C1 * d.y * fetchSH(base, 1)
             +  SH_C1 * d.z * fetchSH(base, 2)
             + -SH_C1 * d.x * fetchSH(base, 3);

        if (sh_degree >= 2) {
            float xx = d.x * d.x, yy = d.y * d.y, zz = d.z * d.z;
            float xy = d.x * d.y, yz = d.y * d.z, xz = d.x * d.z;
            rgb += SH_C2_0 * xy               * fetchSH(base, 4)
                 + SH_C2_1 * yz               * fetchSH(base, 5)
                 + SH_C2_2 * (2.0*zz-xx-yy)   * fetchSH(base, 6)
                 + SH_C2_3 * xz               * fetchSH(base, 7)
                 + SH_C2_4 * (xx - yy)        * fetchSH(base, 8);

            if (sh_degree >= 3) {
                rgb += SH_C3_0 * d.y * (3.0*xx - yy)             * fetchSH(base, 9)
                     + SH_C3_1 * d.z * xy                         * fetchSH(base, 10)
                     + SH_C3_2 * d.y * (4.0*zz - xx - yy)         * fetchSH(base, 11)
                     + SH_C3_3 * d.z * (2.0*zz - 3.0*xx - 3.0*yy) * fetchSH(base, 12)
                     + SH_C3_4 * d.x * (4.0*zz - xx - yy)         * fetchSH(base, 13)
                     + SH_C3_5 * d.z * (xx - yy)                  * fetchSH(base, 14)
                     + SH_C3_6 * d.x * (xx - 3.0*yy)              * fetchSH(base, 15);
            }
        }
    }

    vColor = clamp(rgb, 0.0, 1.0);
    gl_Position = worldViewProj * vec4(position, 1.0);
}
