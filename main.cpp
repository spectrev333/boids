#include <iostream>
#include <vector>

#include "raylib.h"

typedef struct {
    Vector2 position;
    Vector2 velocity;
    Vector2 acceleration;
} Boid;

void DrawBoid(Boid boid) {
    DrawCircleV(boid.position, 10, RED);
}

int main() {
    InitWindow(800, 600, "Boids");

    std::vector<Boid> boids = {
        {
            .position = {400, 300},
            .velocity = {0, 0},
            .acceleration = {0, 0}
        }
    };

    while (!WindowShouldClose()) {
        BeginDrawing();
        ClearBackground(RAYWHITE);

        for (Boid boid : boids) {
            DrawBoid(boid);
        }

        EndDrawing();
    }

    return 0;
}