#version 330

uniform vec3 u_cam_pos;
uniform vec3 u_light_pos;
uniform vec3 u_light_intensity;

uniform vec4 u_color;

uniform sampler2D u_texture_2;
uniform vec2 u_texture_2_size;

uniform float u_normal_scaling;
uniform float u_height_scaling;

in vec4 v_position;
in vec4 v_normal;
in vec4 v_tangent;
in vec2 v_uv;

out vec4 out_color;

float h(vec2 uv) {
  // You may want to use this helper function...
  return (texture(u_texture_2, uv)).r;
}

void main() {
  // YOUR CODE HERE
  vec3 b = cross(v_normal.xyz, v_tangent.xyz);
  mat3 tbn = mat3(v_tangent.xyz, b, v_normal.xyz);
  float height = h(v_uv);
  vec2 u = vec2(v_uv.x + (1 / u_texture_2_size.x), v_uv.y);
  vec2 v = vec2(v_uv.x, v_uv.y + (1 / u_texture_2_size.y));
  float du = ((h(u) - height) * u_height_scaling * u_normal_scaling);
  float dv = ((h(v) - height) * u_height_scaling * u_normal_scaling);
  vec3 n0 = vec3(-1 * du, -1 * dv, 1);
  vec3 nd = tbn * n0;

  vec3 r = u_light_pos - v_position.xyz;
  float r2 = length(r) * length(r);
  vec3 v_diff = u_cam_pos - v_position.xyz;
  vec3 h = (v_diff + r) / length(v_diff + r);

  vec3 p = (u_light_intensity / r2) * pow(max(dot(nd, normalize(h)), 0), 80);
  vec3 d = (u_light_intensity / r2) * max(dot(nd, normalize(r)), 0);
  out_color.rgb = p + d;
  // (Placeholder code. You will want to replace it.)
  //out_color = (vec4(1, 1, 1, 0) + v_normal) / 2;
  out_color.a = 1;
}

