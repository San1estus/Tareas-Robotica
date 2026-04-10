#pragma once
#include <vector>
#include <glm/glm.hpp>
#include "Entity.hpp"
#include "Renderer.hpp"
#include "Shapes.hpp"

class PolygonEditor
{

public:

    std::vector<std::vector<glm::vec2>> polygons;
    std::vector<glm::vec2> current;

    void addVertex(glm::vec2 p);

    bool finishPolygon(std::vector<Entity>& obstacles);

};


void PolygonEditor::addVertex(glm::vec2 p)
{
    current.push_back(p);
}

bool PolygonEditor::finishPolygon(std::vector<Entity>& obstacles)
{

    if(current.size() < 3)
        return false;

    ShapeData data;

    for(auto& v : current)
    {
        data.vertices.push_back(v.x);
        data.vertices.push_back(v.y);
    }

    int n = current.size();

    for(int i=1;i<n-1;i++)
    {
        data.indices.push_back(0);
        data.indices.push_back(i);
        data.indices.push_back(i+1);
    }

    Renderer* renderer = new Renderer(data.vertices,data.indices);

    Entity obstacle =
    {
        EntityType::OBSTACLE,
        renderer,
        glm::vec2(0,0),
        0,
        1,
        glm::vec3(0.8f,0.2f,0.2f),
        ShapeType::POLYGON,
        data
    };

    obstacle.updateWorldVertices();

    obstacles.push_back(obstacle);

    polygons.push_back(current);

    current.clear();

    return true;
}