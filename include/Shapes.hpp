#ifndef SHAPES_H
#define SHAPES_H
#include <cmath>
#include <vector>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <vector>
const float pi =  acos(-1);
struct ShapeData{
	std::vector<float> vertices;
	std::vector<unsigned int> indices;
};

ShapeData createEquilateralTriangle(float size){
	float h = size * sqrt(3.0f) / 2.0f;
  ShapeData data;
  data.vertices = {
    -size/2.0f, -h/3.0f,
     size/2.0f, -h/3.0f,
     0.0f,       2.0f*h/3.0f
  };
  data.indices = { 0, 1, 2 };
  return data; 
}

ShapeData createRectangle(float width, float height, float centerX, float centerY){
		float hh = height/2.0f;
		float hw = width/2.0f;
		ShapeData data;
		data.vertices = {
			-hw, -hh,
			hw, -hh,
			-hw, hh,
			hw, hh
		};
		data.indices = {
			0, 1, 2,
			2, 3, 0
		};
	}
ShapeData createCircle(float radius, float centerX, float centerY, int segments){
	ShapeData data;
	data.vertices.push_back(centerY);
	data.vertices.push_back(centerX);
	for(int i = 0; i <= segments; i++){
		float theta = 2.0f * pi * float(i)/float(segments);
		float x = radius * cos(theta);
		float y = radius * sin(theta);
		data.vertices.push_back(centerX + x);
		data.vertices.push_back(centerY + y);
		
		if(i > 0){
			data.indices.push_back(0);
			data.indices.push_back(i);
			data.indices.push_back(i+1);
		}	
	}
	return data;
}
#endif