#include <stdlib.h>
#include "raylib.h"
#include "raymath.h"

#define WINDOW_WIDTH 800
#define WINDOW_HEIGHT 800
#define MAX_LOCAL_FLOCK_SIZE 128

typedef struct {
    Vector2 position;
    Vector2 velocity;
    Vector2 acceleration;
} Boid;

typedef struct {
    // TODO: use array of pointers??
    Boid boids[MAX_LOCAL_FLOCK_SIZE];
    int size;
} LocalFlock;

void DrawBoid(Boid* boid) {
    DrawCircleV(boid->position, 10, RED);
}

void UpdateBoid(Boid* boid) {
    // TODO: maybe, scale with deltaTime
    boid->position = Vector2Add(boid->position, boid->velocity);
    boid->position.x = Wrap(boid->position.x, 0, WINDOW_WIDTH);
    boid->position.y = Wrap(boid->position.y, 0, WINDOW_HEIGHT);
    boid->velocity = Vector2Add(boid->velocity, boid->acceleration);
}

void GetLocalFlock(Boid* currentBoid, Boid* boids, const int size, LocalFlock* flock, float radius) {
    flock->size = 0;
    for (int i = 0; i < size; i++) {
        float dist = Vector2Distance(boids[i].position, currentBoid->position);
        if (dist < radius && currentBoid != &boids[i]) {
            flock->boids[flock->size] = boids[i];
            flock->size++;
        }
    }
}

void AlignBoid(Boid* boid, LocalFlock* localFlock, float weight) {
    Vector2 averageVelocity = {0, 0};
    for (int i = 0; i < localFlock->size; i++) {
        averageVelocity = Vector2Add(averageVelocity, localFlock->boids[i].velocity);
    }
    if (localFlock->size > 0) {
        averageVelocity = Vector2Scale(averageVelocity, 1.0 / localFlock->size);
        boid->acceleration = Vector2Subtract(averageVelocity, boid->velocity);
        boid->acceleration = Vector2Scale(boid->acceleration, weight);
    }
}

float RandomFloat(float min, float max) {
    // Calculate the range size
    float range = max - min;

    // Scale the result of rand() (0 to RAND_MAX) to the desired range (0 to range)
    // and then offset by min.
    // We cast rand() to float to ensure floating-point division.
    return min + ((float)rand() / RAND_MAX) * range;
}

Vector2 RandomVector2(float min, float max) {
    return (Vector2){
        .x = RandomFloat(min, max),
        .y = RandomFloat(min, max)
    };
}

int main() {
    InitWindow(800, 800, "Boids");
    SetTargetFPS(60);

    const int boidCount = 100;
    Boid boids[boidCount];

    for (int i = 0; i < boidCount; i++) {
        boids[i].position = RandomVector2(0, 800);
        boids[i].velocity = RandomVector2(-10, 10);
        boids[i].acceleration = (Vector2){ .x = 0, .y = 0 };
    }

    LocalFlock localFlock;

    while (!WindowShouldClose()) {
        BeginDrawing();
        ClearBackground(RAYWHITE);

        for (int i = 0; i < boidCount; i++) {
            GetLocalFlock(&boids[i], boids, boidCount, &localFlock, 50);
            AlignBoid(&boids[i], &localFlock, 0.1);
            UpdateBoid(&boids[i]);
            DrawBoid(&boids[i]);
        }

        EndDrawing();
    }

    return 0;
}