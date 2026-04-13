#include <GLFW/glfw3.h>
#include <vector>
#include <cmath>
#include <algorithm>

// Variables de la simulación
const int   W = 800;
const int   H = 800;
const float WORLD = 10.0f;
const float PI = acos(-1.0f);
const float RADIUS = 0.20f;
const float STEP_SPEED = 0.005f;

// Estructuras de datos adicionales

struct Vec2 {
    float x = 0, y = 0;
    Vec2 operator+(Vec2 o)  const { return {x + o.x, y + o.y}; }
    Vec2 operator-(Vec2 o)  const { return {x - o.x, y - o.y}; }
    Vec2 operator*(float s) const { return {x * s, y * s}; }
    float dot(Vec2 o)       const { return x * o.x + y * o.y; }
    float len()             const { return std::sqrt(x * x + y * y); }
    Vec2  norm()            const { float l = len(); return l > 1e-6f ? Vec2{x / l, y / l} : Vec2{0, 0}; }
};

struct Polygon { std::vector<Vec2> pts; };

std::vector<Polygon> obstacles;
std::vector<Vec2>    building;
bool drawing = true;
Vec2 goal    = {6, 6};

bool segmentsIntersect(Vec2 p1, Vec2 p2, Vec2 p3, Vec2 p4) {
    Vec2 d1 = p2 - p1, d2 = p4 - p3;
    float den = d1.x * d2.y - d1.y * d2.x;
    if (std::fabs(den) < 1e-6f) return false;
    Vec2 d3 = p3 - p1;
    float t = (d3.x * d2.y - d3.y * d2.x) / den;
    float u = (d3.x * d1.y - d3.y * d1.x) / den;
    return t > 0.01f && t < 0.99f && u > 0.01f && u < 0.99f;
}

// Robot

struct Robot {
    Vec2 pos = {-6, -6};
    std::vector<Vec2> trail;
    int  followDir  = 0;
    bool dirLocked  = false;
    Vec2 lastMoveDir = {1, 0};

	Vec2  orbitStart    = {};
    bool  orbitLeft     = false;
    bool  unreachable   = false;

    void update();
    void applyPhysics();
	private:
    bool lineOfSightClear() const;
};

bool Robot::lineOfSightClear() const {
    for (const auto& poly : obstacles) {
        int n = poly.pts.size();
        for (int i = 0; i < n; i++) {
            if (segmentsIntersect(pos, goal, poly.pts[i], poly.pts[(i+1)%n]))
                return false;
        }
    }
    return true;
}

void Robot::update() {
    if (trail.empty() || (trail.back() - pos).len() > 0.1f)
        trail.push_back(pos);

    // Encontrar el punto más cercano en cualquier obstáculo y su normal
    float minDist = 1e9f;
    Vec2  closestPt = {};
    Vec2  wallNormal = {};

    for (const auto& poly : obstacles) {
        int n = poly.pts.size();
        for (int i = 0; i < n; i++) {
            Vec2 a = poly.pts[i], b = poly.pts[(i+1)%n];
            Vec2 ab = b-a, ap = pos-a;
            float t = std::max(0.f, std::min(1.f, ap.dot(ab)/ab.dot(ab)));
            Vec2 cl = a + ab*t;
            float d = (pos-cl).len();
            if (d < minDist) {
                minDist = d;
                closestPt = cl;
                wallNormal = (pos-cl).norm();
            }
        }
    }

    const float ORBIT_DIST = RADIUS * 1.5f; 

    if (!dirLocked) {
        // Si no hay obstáculo cercano, avanzar directo al objetivo
        if (minDist > ORBIT_DIST) {
            lastMoveDir = (goal - pos).norm();
            pos = pos + lastMoveDir * STEP_SPEED;
        } else {
            // Al entrar en contacto con un obstáculo, decidir dirección de seguimiento
            Vec2 t1 = {-wallNormal.y,  wallNormal.x}; // tangente CCW
            Vec2 t2 = { wallNormal.y, -wallNormal.x}; // tangente CW
            followDir = (t1.dot(lastMoveDir) >= 0) ? +1 : -1;
            dirLocked = true;
        }
    } else {
		// Antes de continuar orbitando verificamos si el camino directo al objetivo está despejado
		if (lineOfSightClear()) {
            dirLocked = false;
            lastMoveDir = (goal - pos).norm();
            pos = pos + lastMoveDir * STEP_SPEED;
            applyPhysics();
            return;
        }

        // Seguir la pared en la dirección elegida
        Vec2 t1 = {-wallNormal.y,  wallNormal.x};
        Vec2 t2 = { wallNormal.y, -wallNormal.x};
        Vec2 tangent = (followDir > 0) ? t1 : t2;

        // Corrección suave para mantener distancia de la pared
        float error = ORBIT_DIST - minDist; 
        Vec2 correction = wallNormal * error * 0.3f;

        pos = pos + tangent * STEP_SPEED + correction;
        lastMoveDir = tangent;
    }

    applyPhysics();
}

// Física de colisiones simple para evitar que el robot atraviese paredes
void Robot::applyPhysics() {
    for (const auto& poly : obstacles) {
        int n = poly.pts.size();
        for (int i = 0; i < n; i++) {
            Vec2 a = poly.pts[i], b = poly.pts[(i + 1) % n];
            Vec2 ap = pos - a, ab = b - a;
            float t = std::max(0.f, std::min(1.f, ap.dot(ab) / ab.dot(ab)));
            Vec2 closest = a + ab * t;
            float d = (pos - closest).len();
            if (d < RADIUS)
                pos = pos + (pos - closest).norm() * (RADIUS - d + 0.001f);
        }
    }
}
Robot robot;

// Renderizado

void toScreen(Vec2 p, float& sx, float& sy) {
    sx = (p.x + WORLD) / (2 * WORLD) * W;
    sy = H - (p.y + WORLD) / (2 * WORLD) * H;
}

void drawCircle(Vec2 c, float r) {
    float cx, cy; toScreen(c, cx, cy);
    float rx = r / (2 * WORLD) * W;
    glBegin(GL_TRIANGLE_FAN);
    for (int i = 0; i <= 32; i++) {
        float a = i / 32.f * 2 * PI;
        glVertex2f(cx + cosf(a) * rx, cy + sinf(a) * rx);
    }
    glEnd();
}

void drawScene() {
    glColor3f(0.8f, 0.8f, 0.8f);
    glLineWidth(2.0f);
    for (auto& p : obstacles) {
        glBegin(GL_LINE_LOOP);
        for (auto& v : p.pts) { float x, y; toScreen(v, x, y); glVertex2f(x, y); }
        glEnd();
    }

    if (!building.empty()) {
        glColor3f(1, 1, 0);
        glBegin(GL_LINE_STRIP);
        for (auto& p : building) { float x, y; toScreen(p, x, y); glVertex2f(x, y); }
        glEnd();
    }

    glColor4f(0.0f, 0.4f, 1.0f, 0.5f);
    glLineWidth(2.0f);
    if (robot.trail.size() > 1) {
        glBegin(GL_LINE_STRIP);
        for (auto& v : robot.trail) { float x, y; toScreen(v, x, y); glVertex2f(x, y); }
        glEnd();
    }

    glColor3f(0.0f, 0.8f, 0.2f); drawCircle(goal, 0.3f);
    glColor3f(0.0f, 0.4f, 1.0f); drawCircle(robot.pos, RADIUS);
    glColor3f(1.0f, 1.0f, 1.0f); drawCircle(robot.pos, 0.05f);
}

// Utilidades de entrada

Vec2 s2w(double sx, double sy) {
    return { (float)(sx / W) * 2 * WORLD - WORLD,
             (float)((H - sy) / H) * 2 * WORLD - WORLD };
}

void mouseBtn(GLFWwindow* w, int btn, int act, int) {
    if (!drawing) return;
    if (btn == GLFW_MOUSE_BUTTON_LEFT && act == GLFW_PRESS) {
        double x, y; glfwGetCursorPos(w, &x, &y);
        building.push_back(s2w(x, y));
    }
}

void keyBtn(GLFWwindow*, int key, int, int act, int) {
    if (act != GLFW_PRESS) return;
    if (key == GLFW_KEY_C && building.size() >= 3) {
        obstacles.push_back({building}); building.clear();
    }
    if (key == GLFW_KEY_ENTER) drawing = false;
    if (key == GLFW_KEY_R) {
        obstacles.clear(); building.clear(); drawing = true;
        robot = Robot{};
    }
}

int main() {
    glfwInit();
    GLFWwindow* win = glfwCreateWindow(W, H, "Tangent Bug", NULL, NULL);
    glfwMakeContextCurrent(win);
    glfwSetMouseButtonCallback(win, mouseBtn);
    glfwSetKeyCallback(win, keyBtn);
    glMatrixMode(GL_PROJECTION); glLoadIdentity(); glOrtho(0, W, H, 0, -1, 1);
    glEnable(GL_BLEND); glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    while (!glfwWindowShouldClose(win)) {
        glfwPollEvents();
        if (!drawing) robot.update();
        glClearColor(0.1f, 0.1f, 0.12f, 1);
        glClear(GL_COLOR_BUFFER_BIT);
        drawScene();
        glfwSwapBuffers(win);
    }
    glfwTerminate();
}