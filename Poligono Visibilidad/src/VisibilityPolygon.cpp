#include <iostream>
#include <vector>
#include <cmath>
#include <algorithm>
#include <GLFW/glfw3.h>

using namespace std;

const float PI = acos(-1);

struct Point {
    float x, y;
    Point(float _x = 0.0f, float _y = 0.0f) : x(_x), y(_y) {}
};

struct Segment {
    Point a, b;
    Segment(Point _a, Point _b) : a(_a), b(_b) {}
};

struct AngularPoint {
    Point p;
    float angle;
    float dist;
    bool operator<(const AngularPoint& other) const {
        return angle < other.angle;
    }
};

// --- Variables Globales Modificadas para Múltiples Polígonos ---
vector<vector<Point>> polygons;  // Almacena los polígonos ya terminados
vector<Point> currentPolygon;    
bool visibilityMode = false;     
Point currentMousePos(0, 0);

int window_width;
int window_height;
float orthoAspectX = 10.0f;
float orthoY = 10.0f;

// --- Matemáticas y Visibilidad ---
float crossProduct(Point a, Point b) {
    return (a.x * b.y) - (a.y * b.x);
}

float getRaySegmentIntersection(Point p, Point r, Point a, Point b) {
    Point s = {b.x - a.x, b.y - a.y};
    Point q_minus_p = {a.x - p.x, a.y - p.y};

    float r_cross_s = crossProduct(r, s);

    if (abs(r_cross_s) < 1e-5f) return -1.0f;

    float t = crossProduct(q_minus_p, s) / r_cross_s;
    float u = crossProduct(q_minus_p, r) / r_cross_s;

    if (t > 1e-4f && u >= 0.0f && u <= 1.0f) {
        return t; 
    }
    return -1.0f;
}

vector<Point> calculateVisibilityPolygon(Point p, const vector<Segment>& segments) {
    vector<AngularPoint> detectedPoints;
    vector<Point> endpoints;
    
    for (const auto& seg : segments) {
        endpoints.push_back(seg.a);
        endpoints.push_back(seg.b);
    }

    for (const auto& target : endpoints) {
        float base_angle = atan2(target.y - p.y, target.x - p.x);
        float offset = 0.0001f;
        float angles[3] = { base_angle - offset, base_angle, base_angle + offset };

        for (int i = 0; i < 3; ++i) {
            float ang = angles[i];
            Point r = {cos(ang), sin(ang)}; 
            
            float minT = INFINITY; 
            
            for (const auto& seg : segments) {
                float t = getRaySegmentIntersection(p, r, seg.a, seg.b);
                if (t != -1.0f && t < minT) {
                    minT = t;
                }
            }

            if (minT != INFINITY) {
                Point result = {p.x + r.x * minT, p.y + r.y * minT};
                detectedPoints.push_back({result, ang, minT});
            }
        }
    }

    sort(detectedPoints.begin(), detectedPoints.end());

    vector<Point> resultPoly;
    for (const auto& ap : detectedPoints) {
        resultPoly.push_back(ap.p);
    }

    if (!resultPoly.empty()) {
        resultPoly.push_back(resultPoly[0]);
    }

    return resultPoly;
}

vector<Segment> getSceneSegments() {
    vector<Segment> segments;

    // Agregar todos los polígonos cerrados a la lista de colisiones
    for (const auto& poly : polygons) {
        int n = poly.size();
        for (int i = 0; i < n; ++i) {
            segments.push_back(Segment(poly[i], poly[(i + 1) % n]));
        }
    }

    float b = 50.0f; 
    Point tl(-b,  b), tr( b,  b);
    Point bl(-b, -b), br( b, -b);

    segments.push_back(Segment(tl, tr)); 
    segments.push_back(Segment(tr, br)); 
    segments.push_back(Segment(br, bl)); 
    segments.push_back(Segment(bl, tl)); 

    return segments;
}


// --- Callbacks de GLFW ---

void framebuffer_size_callback(GLFWwindow* window, int width, int height) {
    glViewport(0, 0, width, height);
    window_width = width;
    window_height = height;

    float aspect = (float)width / (float)height;
    orthoAspectX = aspect * 10.0f; 

    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    glOrtho(-orthoAspectX, orthoAspectX, -orthoY, orthoY, -1.0f, 1.0f);
    
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
}

void cursor_position_callback(GLFWwindow* window, double xpos, double ypos) {
    float normX = (xpos / window_width) * 2.0f - 1.0f;
    float normY = 1.0f - (ypos / window_height) * 2.0f;
    currentMousePos.x = normX * orthoAspectX;
    currentMousePos.y = normY * orthoY;
}

void mouse_button_callback(GLFWwindow* window, int button, int action, int mods) {
    if (button == GLFW_MOUSE_BUTTON_LEFT && action == GLFW_PRESS) {
        if (!visibilityMode) {
            currentPolygon.push_back(currentMousePos);
            cout << "Punto agregado al poligono actual.\n";
        }
    }
}

void key_callback(GLFWwindow* window, int key, int scancode, int action, int mods) {
    if (action == GLFW_PRESS) {
        // Cerrar el polígono actual y guardarlo
        if (key == GLFW_KEY_ENTER || key == GLFW_KEY_C) {
            if (currentPolygon.size() >= 3) {
                polygons.push_back(currentPolygon);
                currentPolygon.clear(); 
                cout << "Poligono cerrado. Dibuja otro o presiona ESPACIO para ver la visibilidad.\n";
            } else {
                cout << "Necesitas al menos 3 puntos.\n";
            }
        }
        // Alternar el modo de visibilidad
        if (key == GLFW_KEY_SPACE) {
            visibilityMode = !visibilityMode;
            if (visibilityMode) cout << "-- MODO VISIBILIDAD --\n";
            else cout << "-- MODO DIBUJO --\n";
        }
        // Limpiar lienzo completo
        if (key == GLFW_KEY_R) {
            polygons.clear();
            currentPolygon.clear();
            visibilityMode = false;
            cout << "Lienzo limpiado.\n";
        }
    }
}

// --- Main ---

int main() {
    if (!glfwInit()) return -1;
    
    const GLFWvidmode* mode = glfwGetVideoMode(glfwGetPrimaryMonitor());
    window_width = mode->width;
    window_height = mode->height;

    GLFWwindow* window = glfwCreateWindow(window_width, window_height, "Poligono de Visibilidad", NULL, NULL);
    if (!window) {
        glfwTerminate();
        return -1;
    }

    glfwMakeContextCurrent(window);
    
    glfwSetFramebufferSizeCallback(window, framebuffer_size_callback);
    glfwSetCursorPosCallback(window, cursor_position_callback);
    glfwSetMouseButtonCallback(window, mouse_button_callback);
    glfwSetKeyCallback(window, key_callback);

    framebuffer_size_callback(window, window_width, window_height);

    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    cout << "INSTRUCCIONES:\n";
    cout << "- Clic izquierdo: Agregar punto al poligono.\n";
    cout << "- Presiona ENTER: Cerrar poligono actual (puedes dibujar varios).\n";
    cout << "- Presiona ESPACIO: Activar/Desactivar Linterna (Visibilidad).\n";
    cout << "- Presiona R: Limpiar todo.\n";

    while (!glfwWindowShouldClose(window)) {
        glClearColor(0.1f, 0.1f, 0.1f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT);

        // Dibujar todos los polígonos terminados 
        glColor3f(1.0f, 1.0f, 1.0f);
        for (const auto& poly : polygons) {
            glBegin(GL_LINE_LOOP);
            for (const auto& p : poly) glVertex2f(p.x, p.y);
            glEnd();
        }

        // Dibujar el polígono que estamos construyendo actualmente
        if (!visibilityMode) {
            glColor3f(0.5f, 0.5f, 1.0f);
            glBegin(GL_LINE_STRIP);
            for (const auto& p : currentPolygon) glVertex2f(p.x, p.y);
            if (!currentPolygon.empty()) {
                glVertex2f(currentMousePos.x, currentMousePos.y); // Línea guía
            }
            glEnd();
            
            glPointSize(6.0f);
            glColor3f(1.0f, 0.0f, 0.0f);
            glBegin(GL_POINTS);
            for (const auto& p : currentPolygon) glVertex2f(p.x, p.y);
            glEnd();
        }

        // Modo Visibilidad
        if (visibilityMode) {
            vector<Segment> scene = getSceneSegments();
            vector<Point> visPoly = calculateVisibilityPolygon(currentMousePos, scene);

            if (!visPoly.empty()) {
                glColor4f(1.0f, 1.0f, 0.0f, 0.5f); 
                glBegin(GL_TRIANGLE_FAN);
                glVertex2f(currentMousePos.x, currentMousePos.y); 
                for (const auto& p : visPoly) {
                    glVertex2f(p.x, p.y);
                }
                glEnd();

                // Dibujar punto de visibilidad
                glPointSize(10.0f);
                glColor3f(0.0f, 1.0f, 0.0f); 
                glBegin(GL_POINTS);
                glVertex2f(currentMousePos.x, currentMousePos.y);
                glEnd();
            }
        }

        glfwSwapBuffers(window);
        glfwPollEvents();
    }

    glfwTerminate();
    return 0;
}