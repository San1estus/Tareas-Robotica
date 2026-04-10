#pragma once
#include <glm/glm.hpp>

inline float cross2D(const glm::vec2& a, const glm::vec2& b)
{
    return a.x * b.y - a.y * b.x;
}

bool raySegmentIntersection(
    const glm::vec2& rayOrigin,
    const glm::vec2& rayDir,
    const glm::vec2& a,
    const glm::vec2& b,
    float& t)
{

    glm::vec2 v1 = rayOrigin - a;
    glm::vec2 v2 = b - a;
    glm::vec2 v3(-rayDir.y, rayDir.x);

    float dot = glm::dot(v2, v3);

    if (fabs(dot) < 1e-6f)
        return false;

    float t1 = cross2D(v2, v1) / dot;
    float t2 = glm::dot(v1, v3) / dot;

    if (t1 >= 0.0f && t2 >= 0.0f && t2 <= 1.0f)
    {
        t = t1;
        return true;
    }

    return false;
}