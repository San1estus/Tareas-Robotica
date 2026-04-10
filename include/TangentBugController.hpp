#pragma once
#include <vector>
#include <cmath>
#include "Entity.hpp"
#include "Sensor.hpp"

enum class TBState
{
    GO_TO_GOAL,
    FOLLOW_BOUNDARY
};

class TangentBugController
{

public:
	Sensor sensor;

    TBState state = TBState::GO_TO_GOAL;

    float robotSpeed = 2.0f;
    float safeDistance = 0.4f;

    float d_min = std::numeric_limits<float>::max();

    int currentEdge = -1;
    int followDir = 1;

    const Entity* boundaryObstacle = nullptr;

    void update(
        Entity& robot,
        const Entity& goal,
        const std::vector<Entity>& obstacles,
        float dt
    );

};


void TangentBugController::update(
    Entity& robot,
    const Entity& goal,
    const std::vector<Entity>& obstacles,
    float dt)
{

    float step = robotSpeed * dt;

    auto scan = sensor.scan(robot.position,obstacles);

    if(state == TBState::GO_TO_GOAL)
    {

        glm::vec2 toGoal = goal.position - robot.position;

        if(glm::length(toGoal) < 0.01f)
            return;

        glm::vec2 dir = glm::normalize(toGoal);

        bool blocked = false;

        for(auto& r : scan)
        {
            if(r.hit && r.distance < safeDistance)
            {
                blocked = true;
                break;
            }
        }

        if(!blocked)
        {

            robot.position += dir * step;
            robot.rotation = atan2(dir.y,dir.x);

        }
        else
        {

            d_min = glm::distance(robot.position,goal.position);

            state = TBState::FOLLOW_BOUNDARY;

        }

    }

    else if(state == TBState::FOLLOW_BOUNDARY)
	{
		RayHit closestHit;
		closestHit.hit = false;
		float minDist = std::numeric_limits<float>::max();

		// 1. Encontrar el punto de colisión más cercano para determinar la "pared"
		for(auto& r : scan)
		{
			if(r.hit && r.distance < minDist)
			{
				minDist = r.distance;
				closestHit = r;
			}
		}

		if(closestHit.hit)
		{
			// 2. Calcular el vector normal (desde el punto de impacto hacia el robot)
			glm::vec2 normal = glm::normalize(robot.position - closestHit.point);

			// 3. Calcular la dirección tangencial (perpendicular a la normal)
			// followDir puede ser 1 (horario) o -1 (anti-horario)
			glm::vec2 tangent = glm::vec2(-normal.y, normal.x) * (float)followDir;

			// 4. Corregir la distancia al muro (P-Controller simple)
			// Si estamos muy cerca, nos alejamos un poco; si estamos lejos, nos acercamos.
			float error = minDist - safeDistance;
			glm::vec2 correction = normal * error;

			// Dirección final balanceada entre avanzar por la tangente y mantener la distancia
			glm::vec2 finalDir = glm::normalize(tangent + correction * 0.5f);

			robot.position += finalDir * step;
			robot.rotation = atan2(finalDir.y, finalDir.x);
		}
		else 
		{
			// Si perdemos el muro (ej. esquina), giramos en la dirección de búsqueda
			robot.rotation += (float)followDir * step;
			robot.position += glm::vec2(cos(robot.rotation), sin(robot.rotation)) * step;
		}

		// 5. Condición de salida: Si el camino a la meta está libre y estamos más cerca que d_min
		float distGoal = glm::distance(robot.position, goal.position);
		
		// Verificamos si hay obstáculos en línea recta a la meta
		bool pathBlocked = false;
		glm::vec2 toGoal = glm::normalize(goal.position - robot.position);
		for(auto& r : scan) {
			float dot = glm::dot(glm::normalize(r.point - robot.position), toGoal);
			if(r.hit && r.distance < safeDistance * 2.0f && dot > 0.9f) {
				pathBlocked = true;
				break;
			}
		}

		if(!pathBlocked && distGoal < d_min)
		{
			state = TBState::GO_TO_GOAL;
		}
	}

}