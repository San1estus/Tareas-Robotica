#ifndef ENTITY_H
#define ENTITY_H

#include <vector>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>

#include "Renderer.hpp"
#include "Shapes.hpp"

enum class EntityType{
    ROBOT,
    OBSTACLE,
    GOAL,
    SENSOR
};

enum class ShapeType{
    CIRCLE,
    POLYGON
};

struct Entity{

    EntityType type;

    Renderer* renderer;

    glm::vec2 position = glm::vec2(0);
    float rotation = 0.0f;   // radians
    float scale = 1.0f;

    glm::vec3 color = glm::vec3(1);

    ShapeType shape;

    ShapeData data;

    GLenum drawMode = GL_TRIANGLES;

    std::vector<glm::vec2> worldVertices;

    bool dirty = true;


    void updateWorldVertices(){

        if(shape != ShapeType::POLYGON || !dirty)
            return;

        worldVertices.clear();

        float cosA = cos(rotation);
        float sinA = sin(rotation);

        int n = data.vertices.size();

        for(int i=0;i<n;i+=2){

            float x = data.vertices[i] * scale;
            float y = data.vertices[i+1] * scale;

            float rx = x*cosA - y*sinA;
            float ry = x*sinA + y*cosA;

            worldVertices.push_back(glm::vec2(rx,ry) + position);
        }

        dirty = false;
    }


    void draw(unsigned int shaderID, const glm::mat4& viewProj){

        glm::mat4 model(1.0f);

        model = glm::translate(model, glm::vec3(position,0));
        model = glm::rotate(model, rotation, glm::vec3(0,0,1));
        model = glm::scale(model, glm::vec3(scale,scale,1));

        renderer->draw(shaderID, viewProj*model, model, color, drawMode);
    }
};

#endif