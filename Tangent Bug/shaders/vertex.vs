#version 330 core
layout (location = 0) in vec2 aPos; 
uniform mat4 u_MVP;
uniform mat4 u_Model;
void main() {
  gl_Position = u_MVP * vec4(aPos, 0.0, 1.0);
}