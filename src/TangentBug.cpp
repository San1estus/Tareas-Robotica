#include <iostream>
#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include "Entity.hpp"
#include "Renderer.hpp"
#include "Shapes.hpp"
#include "Shader.hpp"
using namespace std; 

enum State{
	GO_TO_GOAL,
	FOLLOW_BOUNDARY
};

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

		glm::mat4 projection = glm::ortho(aspect*-7.5f, aspect*7.5f, 	-7.5f, 7.5f, -1.0f, 1.0f);

		glm::mat4 view = glm::mat4(1.0f); 
 
		glm::vec3 robotColor = glm::vec3(0.0f, 1.0f, 0.0f);
				
    glViewport(0, 0, window_width, window_height);
    glfwSetFramebufferSizeCallback(window, framebuffer_size_callback);
		ShapeData bug = createCircle(0.5f, 0.0f, 0.0f, 32);
		Renderer bugRenderer(bug.vertices, bug.indices);
    while(!glfwWindowShouldClose(window))
		{
			glClearColor(0.1f, 0.1f, 0.1f, 1.0f);
			glClear(GL_COLOR_BUFFER_BIT);

			float posX = 0.0f; 
			float posY = 0.0f;
			float angle = 0.0f; 

			glm::mat4 modelMatrix = glm::mat4(1.0f);
			modelMatrix = glm::translate(modelMatrix, glm::vec3(posX, posY, 0.0f));
			modelMatrix = glm::rotate(modelMatrix, angle, glm::vec3(0.0f, 0.0f, 1.0f));

			glm::mat4 mvp = projection * view * modelMatrix;

			bugRenderer.draw(shader.ID, mvp, modelMatrix, robotColor);

			glfwSwapBuffers(window);
			glfwPollEvents();    
		}

    glfwTerminate();
    return 0;
}