#include <iostream>
#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include "Entity.hpp"
#include "Renderer.hpp"
#include "Shapes.hpp"
#include "Shader.hpp"

using namespace std; 

enum class State{
	GO_TO_GOAL,
	FOLLOW_BOUNDARY
};

State currentState = State::GO_TO_GOAL;
float robotSpeed = 2.5f;

glm::vec2 calculateT(const glm::vec2& robotPos, const glm::vec2& goalPos, const float sensorRadius){
	glm::vec2 direction = goalPos - robotPos;
		float distToGoal = glm::length(direction);
		if(distToGoal <= sensorRadius){
			return goalPos;
		}
		glm::vec2 directionNorm = glm::normalize(direction);
		
		return robotPos + directionNorm * sensorRadius;

}

std::vector<glm::vec2> getPolygonVertices(const Entity& entity) {
  std::vector<glm::vec2> vertices;
  int n = entity.data.vertices.size();
  
  // Pre-calcular seno y coseno si hay rotación
  float rad = glm::radians(entity.rotation);
  float cosA = std::cos(rad);
  float sinA = std::sin(rad);

  for(int i = 0; i < n; i += 2) {
    float x = entity.data.vertices[i];
    float y = entity.data.vertices[i+1];
    
    // 1. Escala
    x *= entity.scale;
    y *= entity.scale;

    // 2. Rotación
    float rx = x * cosA - y * sinA;
    float ry = x * sinA + y * cosA;
    
    // 3. Traslación al mundo real
    vertices.push_back(glm::vec2(rx, ry) + entity.position);
  }
	
  return vertices;
}

std::vector<glm::vec2> getSegmentCircleIntersections(glm::vec2 p1, glm::vec2 p2, glm::vec2 center, float radius) {
  std::vector<glm::vec2> intersections;
  
  glm::vec2 D = p2 - p1;
  glm::vec2 F = p1 - center;
  
  float a = glm::dot(D, D);
  float b = 2.0f * glm::dot(F, D);
  float c = glm::dot(F, F) - (radius * radius);
  
  float discriminant = b * b - 4 * a * c;
  
  // Si el discriminante es negativo, la línea no toca el círculo
  if (discriminant >= 0.0f) {
    discriminant = std::sqrt(discriminant);
    
    // Hay dos posibles puntos de intersección en la línea infinita
    float t1 = (-b - discriminant) / (2.0f * a);
    float t2 = (-b + discriminant) / (2.0f * a);
    
    // Solo nos importan los que caen dentro del segmento (t entre 0 y 1)
    if (t1 >= 0.0f && t1 <= 1.0f) {
      intersections.push_back(p1 + t1 * D);
    }
    if (t2 >= 0.0f && t2 <= 1.0f && t1 != t2) {
      intersections.push_back(p1 + t2 * D);
    }
  }
  
  return intersections;
}
float distancePointSegment(const glm::vec2& p,
                           const glm::vec2& a,
                           const glm::vec2& b)
{
    glm::vec2 ab = b - a;
    float ab2 = glm::dot(ab, ab);

    if(ab2 < 1e-8f) // segmento degenerado
        return glm::distance(p, a);

    float t = glm::dot(p - a, ab) / ab2;
    t = glm::clamp(t, 0.0f, 1.0f);

    glm::vec2 projection = a + t * ab;
    return glm::distance(p, projection);
}

// Esta función detecta los puntos tangentes que se encuentran en el rango del sensor.
std::vector<glm::vec2> getVisibleTangents(const glm::vec2& robotPos, float sensorRadius, const Entity& obstacle){
	std::vector<glm::vec2> tangents;
	if(obstacle.shape == ShapeType::CIRCLE){
		glm::vec2 toCenter = obstacle.position - robotPos;
		float D = glm::length(toCenter);
		float obstacleRadius = obstacle.scale * 0.5f;
		if(D > sensorRadius + obstacleRadius || D <= obstacleRadius){
			return tangents;
		}
		
		float L = std::sqrt(D*D - obstacleRadius*obstacleRadius);

		if (L <= sensorRadius){
			float alpha = std::atan2(toCenter.x, toCenter.y);
			float theta = std::asin(obstacleRadius/D);

			tangents.push_back(robotPos + L * glm::vec2(std::cos(alpha+theta), std::sin(alpha+theta)));
			tangents.push_back(robotPos + L * glm::vec2(std::cos(alpha-theta), std::sin(alpha-theta)));
		}
	}
	else if (obstacle.shape == ShapeType::POLYGON){
		std::vector<glm::vec2>  vertices = getPolygonVertices(obstacle);
		int n = vertices.size();

		for (const auto& v : vertices) {
      if (glm::distance(robotPos, v) <= sensorRadius) {
        tangents.push_back(v);
      }
    }


		for (int i = 0; i < n; ++i) {
      glm::vec2 p1 = vertices[i];
      glm::vec2 p2 = vertices[(i + 1) % n]; 
      
      auto intersections = getSegmentCircleIntersections(p1, p2, robotPos, sensorRadius);
      tangents.insert(tangents.end(), intersections.begin(), intersections.end());
    }
	}
	return tangents;
}

// Variables globales para gestionar los estados
float previousDistToGoal = std::numeric_limits<float>::max();
float d_min = std::numeric_limits<float>::max();
glm::vec2 currentBoundaryTarget;
const Entity* boundaryObstacle = nullptr;
int currentEdgeIndex = -1;
int followDirection = 1;

void updateTangentBug(Entity& robot,
                      const Entity& goal,
                      const std::vector<Entity>& obstacles,
                      float deltaTime)
{
  float step = robotSpeed * deltaTime;
  float distToGoal = glm::distance(robot.position, goal.position);
  
  // Definimos la distancia de seguridad (ej. el radio visual de tu robot)
  float safeDistance = 1.0f; 

  if(currentState == State::GO_TO_GOAL)
  {
    glm::vec2 dir = glm::normalize(goal.position - robot.position);
    glm::vec2 newPos = robot.position + dir * step;

    bool collision = false;
    const Entity* hitObstacle = nullptr;

    for(const auto& obs : obstacles)
    {
      if(obs.shape == ShapeType::CIRCLE)
      {
        float r = obs.scale * 1.0f;
        // 1. Modificación: Añadimos safeDistance al radio del obstáculo
        if(glm::distance(newPos, obs.position) <= (r + safeDistance))
        {
          collision = true;
          hitObstacle = &obs;
          break;
        }
      }
      else if(obs.shape == ShapeType::POLYGON)
      {
        auto vertices = getPolygonVertices(obs);
        int n = vertices.size();

        for(int i=0;i<n;i++)
        {
          // 1. Modificación: Verificamos contra safeDistance en lugar de 0.05f
          if(distancePointSegment(newPos,
                                  vertices[i],
                                  vertices[(i+1)%n]) <= safeDistance)
          {
            collision = true;
            hitObstacle = &obs;
            break;
          }
        }
      }
    }

    if(!collision)
    {
      robot.position = newPos;
      robot.rotation = glm::degrees(std::atan2(dir.y, dir.x));
    }
    else
    {
      boundaryObstacle = hitObstacle;
      d_min = distToGoal;
      currentState = State::FOLLOW_BOUNDARY;

      if(boundaryObstacle->shape == ShapeType::POLYGON)
      {
        auto vertices = getPolygonVertices(*boundaryObstacle);
        int n = vertices.size();

        float minDist = std::numeric_limits<float>::max();
        for(int i=0;i<n;i++)
        {
          float d = distancePointSegment(robot.position,
                                         vertices[i],
                                         vertices[(i+1)%n]);
          if(d < minDist)
          {
            minDist = d;
            currentEdgeIndex = i;
          }
        }
      }
    }
  }
  else if(currentState == State::FOLLOW_BOUNDARY)
  {
    if(!boundaryObstacle) return;

    if(boundaryObstacle->shape == ShapeType::CIRCLE)
    {
      glm::vec2 center = boundaryObstacle->position;
      float r = boundaryObstacle->scale * 1.0f;

      glm::vec2 radial = robot.position - center;
      float dist = glm::length(radial);

      if(dist > 0.0001f)
        radial = glm::normalize(radial);
      else
        radial = glm::vec2(1,0);

      // 2. Modificación: Expandimos el anclaje radial sumando safeDistance
      robot.position = center + radial * (r + safeDistance);

      glm::vec2 tangent(-radial.y, radial.x);
      tangent = glm::normalize(tangent) ;

      robot.position += tangent * step;
      robot.rotation = glm::degrees(std::atan2(tangent.y, tangent.x));
    }
    else if(boundaryObstacle->shape == ShapeType::POLYGON)
    {
      auto vertices = getPolygonVertices(*boundaryObstacle);
      int n = vertices.size();

      if(currentEdgeIndex < 0)
        currentEdgeIndex = 0;

      glm::vec2 a = vertices[currentEdgeIndex];
      glm::vec2 b = vertices[(currentEdgeIndex+1)%n];

      glm::vec2 edgeDir = glm::normalize(b - a);
      
      // En un polígono creado en sentido antihorario, la normal hacia afuera es (y, -x)
      glm::vec2 normal(edgeDir.y, -edgeDir.x);

      glm::vec2 theoreticalPos = robot.position + edgeDir * step;

      // 2. Modificación: Proyectamos para encontrar el punto exacto en el segmento
      glm::vec2 ab = b - a;
      float t = glm::dot(theoreticalPos - a, ab) / glm::dot(ab, ab);	
      t = glm::clamp(t, 0.0f, 1.0f);
      glm::vec2 closestPointOnEdge = a + t * ab;

      // Y anclamos el robot sumando la normal multiplicada por la distancia de seguridad
      robot.position = closestPointOnEdge + normal * safeDistance;
      robot.rotation = glm::degrees(std::atan2(edgeDir.y, edgeDir.x));

      // Usamos 't' para saber con precisión matemática si llegamos al final del vértice
      if(t >= 0.99f)
      {
        currentEdgeIndex = (currentEdgeIndex + followDirection + n) % n;
      }
    }

    float newDist = glm::distance(robot.position, goal.position);

    if(newDist < d_min)
    {
      currentState = State::GO_TO_GOAL;
      boundaryObstacle = nullptr;
      currentEdgeIndex = -1;
    }
  }
}

void framebuffer_size_callback(GLFWwindow* window, int width, int height)
{
    glViewport(0, 0, width, height);
}


int main()
{	
	if (!glfwInit())
    {
        cout << "Failed to initialize GLFW" << endl;
        return -1;
    }
			
    glfwWindowHint(GLFW_SAMPLES, 4);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    glfwWindowHint(GLFW_MAXIMIZED, GLFW_TRUE);

    GLFWwindow* window;
		const GLFWvidmode * mode = glfwGetVideoMode(glfwGetPrimaryMonitor());

    int window_width = mode->width;
    int window_height = mode->height;
    window = glfwCreateWindow(window_width, window_height, "Tangent Bug", NULL, NULL);
    if (window == NULL)
    {
        cout << "Failed to open GLFW window" << endl;
        return -1;
    }
    glfwMakeContextCurrent(window);

    if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress))
    {
        cout << "Failed to initialize GLAD" << endl;
        return -1;
    }	

		Shader shader;
		shader.init("shaders/vertex.vs", "shaders/fragment.fs");
		shader.use();

		float aspect = (float)window_width / (float)window_height;

		glm::mat4 projection = glm::ortho(aspect*-10.0f, aspect*10.f, -10.0f, 10.0f, -1.0f, 1.0f);

		glm::mat4 view = glm::mat4(1.0f); 
 
		glm::vec3 robotColor = glm::vec3(0.0f, 1.0f, 0.0f);
				
    glViewport(0, 0, window_width, window_height);
    glfwSetFramebufferSizeCallback(window, framebuffer_size_callback);

		// Creacion de las entidades del mundo. La meta, el sensor y el robot van aparte.
		std::vector<Entity> obstacles;
		
		ShapeData rectData = createRectangle(2.0f, 1.0f);
		ShapeData squareData = createRectangle(1.0f, 1.0f);
		ShapeData circleData = createCircle(1.0f, 0.0f, 0.0f, 32);

		Renderer* rectRenderer = new Renderer(rectData.vertices, rectData.indices);
		Renderer* squareRenderer = new Renderer(squareData.vertices, squareData.indices);
		Renderer* circleRenderer = new Renderer(circleData.vertices, circleData.indices);
		obstacles.push_back({EntityType::OBSTACLE, rectRenderer, glm::vec2(3.0f, 1.5f), 0.0f, 2.0f, glm::vec3(0.5f), ShapeType::POLYGON, rectData});
		obstacles.push_back({EntityType::OBSTACLE, squareRenderer, glm::vec2(-2.0f, 2.0f), 0.0f, 1.5f, glm::vec3(1.0f, 0.0f, 0.0f), ShapeType::POLYGON, squareData});
		obstacles.push_back({EntityType::OBSTACLE, circleRenderer, glm::vec2(10.0f, 7.0f), 0.0f, 1.0f, glm::vec3(1.0f, 0.0f, 0.0f), ShapeType::CIRCLE, circleData});
		Entity goal = {EntityType::GOAL, circleRenderer, glm::vec2(12.0f, 8.0f), 0.0f, 0.5f, glm::vec3(0.0f, 0.5f, 0.0f), ShapeType::CIRCLE, circleData};
		Entity sensor = {EntityType::SENSOR, circleRenderer, glm::vec2(-12.0f, -9.0f), 0.0f, 1.0f, glm::vec3(0.3f, 0.3f, 1.0f), ShapeType::CIRCLE, circleData, GL_LINE_LOOP};
		Entity robot = {EntityType::ROBOT, circleRenderer, glm::vec2(-12.0f, -9.0f), 0.0f, 0.5f, glm::vec3(1.0f, 1.0f, 0.0f), ShapeType::CIRCLE, circleData};
		
		float lastFrame = 0.0f;
    while(!glfwWindowShouldClose(window))
		{
			glClearColor(0.1f, 0.1f, 0.1f, 1.0f);
			glClear(GL_COLOR_BUFFER_BIT);

			float posX = 0.0f; 
			float posY = 0.0f;
			float angle = 0.0f; 
      
			float currentFrame = glfwGetTime();
      float deltaTime = currentFrame - lastFrame;
			lastFrame = currentFrame;
			updateTangentBug(robot, goal, obstacles, deltaTime);
			sensor.position = robot.position;
			glm::mat4 modelMatrix = glm::mat4(1.0f);
			modelMatrix = glm::translate(modelMatrix, glm::vec3(posX, posY, 0.0f));
			modelMatrix = glm::rotate(modelMatrix, angle, glm::vec3(0.0f, 0.0f, 1.0f));

			glm::mat4 mvp = projection * view * modelMatrix;
			glm::mat4 viewProj = projection * view;
			for(Entity obstacle : obstacles){
				obstacle.draw(shader.ID, viewProj);
			}
			goal.draw(shader.ID, viewProj);
			sensor.draw(shader.ID, viewProj);
			robot.draw(shader.ID, viewProj);
			
			glfwSwapBuffers(window);
			glfwPollEvents();    
		}

    glfwTerminate();
    return 0;
}