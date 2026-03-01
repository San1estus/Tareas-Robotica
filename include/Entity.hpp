#ifndef ENTITY_H
#define ENTITY_H
#include "Renderer.hpp"
#include <glad/glad.h>

enum class EntityType{
	ROBOT,
	OBSTACLE,
	GOAL,
	SENSOR
};

struct Entity{
	EntityType type;
	Renderer* renderer;
	glm::vec2 position;
	float rotation;
	float scale;
	glm::vec3 color;
	GLenum drawMode = GL_TRIANGLES;

	void draw(unsigned int shaderID, const glm::mat4& viewProj){
		glm::mat4 model = glm::mat4(1.0f);
		model = glm::translate(model, glm::vec3(position, 0.0f));
    model = glm::rotate(model, rotation, glm::vec3(0.0f, 0.0f, 1.0f));
    model = glm::scale(model, glm::vec3(scale, scale, 1.0f));
		renderer->draw(shaderID, viewProj * model, model, color, drawMode);
	}
};
#endif