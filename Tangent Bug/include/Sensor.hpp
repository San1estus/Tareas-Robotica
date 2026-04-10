#pragma once
#include <vector>
#include <glm/glm.hpp>
#include "Entity.hpp"
#include "Geometry.hpp"

struct RayHit
{
    bool hit;
    glm::vec2 point;
    float distance;
};

class Sensor
{

public:

    float maxRange = 10.0f;
    int rayCount = 180;

    std::vector<RayHit> scan(
        const glm::vec2& origin,
        const std::vector<Entity>& obstacles
    );

};

std::vector<RayHit> Sensor::scan(
    const glm::vec2& origin,
    const std::vector<Entity>& obstacles)
{

    std::vector<RayHit> hits;

    for(int i=0;i<rayCount;i++)
    {

        float angle = (float)i / rayCount * glm::two_pi<float>();

        glm::vec2 dir(
            cos(angle),
            sin(angle)
        );

        RayHit bestHit;
        bestHit.hit = false;
        bestHit.distance = maxRange;

        for(const auto& obs : obstacles)
        {

            if(obs.shape != ShapeType::POLYGON)
                continue;

            const auto& vertices = obs.worldVertices;
            int n = vertices.size();

            for(int j=0;j<n;j++)
            {

                glm::vec2 a = vertices[j];
                glm::vec2 b = vertices[(j+1)%n];

                float t;

                if(raySegmentIntersection(origin,dir,a,b,t))
                {

                    if(t < bestHit.distance && t <= maxRange)
                    {
                        bestHit.hit = true;
                        bestHit.distance = t;
                        bestHit.point = origin + dir * t;
                    }

                }

            }

        }

        if(!bestHit.hit)
            bestHit.point = origin + dir * maxRange;

        hits.push_back(bestHit);

    }

    return hits;
}