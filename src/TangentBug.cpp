#include <iostream>
#include <vector>

#include <glad/glad.h>
#include <GLFW/glfw3.h>

#include "Entity.hpp"
#include "Renderer.hpp"
#include "Shapes.hpp"
#include "Shader.hpp"

#include "TangentBugController.hpp"
#include "PolygonEditor.hpp"

using namespace std;

PolygonEditor editor;
std::vector<Entity> obstacles;

TangentBugController controller;

int window_width;
int window_height;

glm::vec2 screenToWorld(double x,double y)
{
    float aspect = (float)window_width / (float)window_height;

    float worldX =
        ((float)x / window_width) * (20.0f * aspect)
        - 10.0f * aspect;

    float worldY =
        10.0f
        - ((float)y / window_height) * 20.0f;

    return glm::vec2(worldX,worldY);
}

void mouseCallback(GLFWwindow* window,int button,int action,int mods)
{

    if(button == GLFW_MOUSE_BUTTON_LEFT && action == GLFW_PRESS)
    {

        double posX,posY;
        glfwGetCursorPos(window,&posX,&posY);

        glm::vec2 world = screenToWorld(posX,posY);

        editor.addVertex(world);

        std::cout<<"Vertex added\n";
    }

}

void keyCallback(GLFWwindow* window,int key,int scancode,int action,int mods)
{

    if(action == GLFW_PRESS)
    {

        if(key == GLFW_KEY_ENTER)
        {

            if(editor.finishPolygon(obstacles))
                std::cout<<"Polygon created\n";

        }

    }

}

void framebuffer_size_callback(GLFWwindow* window,int width,int height)
{
    glViewport(0,0,width,height);

    window_width = width;
    window_height = height;
}

int main()
{

    if(!glfwInit())
        return -1;

    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR,3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR,3);
    glfwWindowHint(GLFW_OPENGL_PROFILE,GLFW_OPENGL_CORE_PROFILE);

    const GLFWvidmode* mode = glfwGetVideoMode(glfwGetPrimaryMonitor());

    window_width = mode->width;
    window_height = mode->height;

    GLFWwindow* window =
        glfwCreateWindow(window_width,window_height,"Tangent Bug",NULL,NULL);

    if(!window)
        return -1;

    glfwMakeContextCurrent(window);

    glfwSetMouseButtonCallback(window,mouseCallback);
    glfwSetKeyCallback(window,keyCallback);
    glfwSetFramebufferSizeCallback(window,framebuffer_size_callback);

    if(!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress))
        return -1;

    glViewport(0,0,window_width,window_height);

    Shader shader;
    shader.init("../shaders/vertex.vs","../shaders/fragment.fs");
    shader.use();

    float aspect = (float)window_width / (float)window_height;

    glm::mat4 projection = glm::ortho(
        aspect*-10.0f,
        aspect*10.0f,
        -10.0f,
        10.0f,
        -1.0f,
        1.0f
    );

    glm::mat4 view = glm::mat4(1.0f);

    ShapeData circleData = createCircle(1.0f,32);

    Renderer* circleRenderer =
        new Renderer(circleData.vertices,circleData.indices);

    Entity goal =
    {
        EntityType::GOAL,
        circleRenderer,
        glm::vec2(12,8),
        0,
        0.5,
        glm::vec3(0,0.7f,0),
        ShapeType::CIRCLE,
        circleData
    };

    Entity robot =
    {
        EntityType::ROBOT,
        circleRenderer,
        glm::vec2(-12,-9),
        0,
        0.5,
        glm::vec3(1,1,0),
        ShapeType::CIRCLE,
        circleData
    };

    float lastFrame = 0.0f;

    while(!glfwWindowShouldClose(window))
    {

        glClearColor(0.1f,0.1f,0.1f,1.0f);
        glClear(GL_COLOR_BUFFER_BIT);
        float currentFrame = glfwGetTime();
        float deltaTime = currentFrame - lastFrame;
        lastFrame = currentFrame;
		
		auto scan = controller.sensor.scan(robot.position,obstacles);
		for(auto& r : scan)
		{

			glm::vec2 end = r.point;

			float lineVertices[] =
			{
				robot.position.x, robot.position.y,
				end.x, end.y
			};

		}
        controller.update(robot,goal,obstacles,deltaTime);

        glm::mat4 viewProj = projection * view;

        for(auto& obs : obstacles)
        {
            obs.updateWorldVertices();
            obs.draw(shader.ID,viewProj);
        }

        goal.draw(shader.ID,viewProj);
        robot.draw(shader.ID,viewProj);

        glfwSwapBuffers(window);
        glfwPollEvents();
    }

    glfwTerminate();

    return 0;
}