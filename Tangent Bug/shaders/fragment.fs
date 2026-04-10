uniform vec3 u_ObjectColor;
out vec4 FragColor;

void main() {
  FragColor = vec4(u_ObjectColor, 1.0);
}