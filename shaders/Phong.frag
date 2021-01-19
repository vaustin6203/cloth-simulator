#version 330

uniform vec4 u_color;
uniform vec3 u_cam_pos;
uniform vec3 u_light_pos;
uniform vec3 u_light_intensity;

in vec4 v_position;
in vec4 v_normal;
in vec2 v_uv;

out vec4 out_color;

void main() {
  // YOUR CODE HERE
  float magnitude = length(v_position.xyz + u_cam_pos);
  vec3 r = u_light_pos - v_position.xyz;
  float r2 = length(r) * length(r);
  vec3 v = u_cam_pos - v_position.xyz;
  vec3 h = (v + r) / length(v + r);

  //specular component
  vec3 p = (u_light_intensity / r2) * pow(max(dot(v_normal.xyz, normalize(h)), 0), 90);
  //diffuse component
  vec3 d = (u_light_intensity / r2) * max(dot(v_normal.xyz, normalize(r)), 0);
  //ambient component
  vec3 a = vec3(1, 1, 1) * 0.01;
  out_color.rgb = a;
  // (Placeholder code. You will want to replace it.)
  //out_color = (vec4(1, 1, 1, 0) + v_normal) / 2;
  out_color.a = 1;
}

