#include <iostream>
#include <vector>

#include "raylib.h"
#include "raymath.h"

typedef struct {
    Vector2 position;
    Vector2 velocity;
    Vector2 acceleration;
} Boid;

void DrawBoid(Boid& boid) {
    DrawCircleV(boid.position, 10, RED);
}

void UpdateBoid(Boid& boid) {
    // TODO: maybe, scale with deltaTime
    boid.position = Vector2Add(boid.position, boid.velocity);
    boid.velocity = Vector2Add(boid.velocity, boid.acceleration);
}

float RandomFloat(float min, float max) {
    // Source - https://stackoverflow.com/questions/686353/random-float-number-generation
    // Posted by John Dibling
    // Retrieved 11/5/2025, License - CC BY-SA 3.0
    return min + static_cast <float> (std::rand()) /( static_cast <float> (RAND_MAX/(min-max)));
}

Vector2 RandomVector2(float max) {
    return {RandomFloat(0, max), RandomFloat(0, max)};
}

int main() {
    InitWindow(800, 600, "Boids");
    SetTargetFPS(60);

    std::vector<Boid> boids = {
        {
            .position = {400, 300},
            .velocity = RandomVector2(3.0),
            .acceleration = {0, 0}
        }
    };

    while (!WindowShouldClose()) {
        BeginDrawing();
        ClearBackground(RAYWHITE);

        for (Boid& boid : boids) {
            UpdateBoid(boid);
            DrawBoid(boid);
        }

        EndDrawing();
    }

    return 0;
}