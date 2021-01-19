#version 330


uniform vec3 u_cam_pos;

uniform samplerCube u_texture_cubemap;

in vec4 v_position;
in vec4 v_normal;
in vec4 v_tangent;

out vec4 out_color;

void main() {
  // YOUR CODE HERE
  vec3 wo = u_cam_pos - v_position.xyz;
  //reflect across the normal
  vec3 wi = 2 * dot(wo, v_normal.xyz) * v_normal.xyz - wo;
  //sample from cubemap
  out_color.rgb = (texture(u_texture_cubemap, wi)).rgb;
  //out_color = (vec4(1, 1, 1, 0) + v_normal) / 2;
  out_color.a = 1;
}
