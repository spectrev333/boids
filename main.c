#include <stdlib.h>
#include <math.h>
#include "raylib.h"
#include "raymath.h"

#define WINDOW_WIDTH 1200
#define WINDOW_HEIGHT 1000
#define MAX_LOCAL_FLOCK_SIZE 128*2

#define MAX_VELOCITY 5
#define MAX_ACCELERATION 1

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
    DrawCircleV(boid->position, 5, RED);
}

void UpdateBoid(Boid* boid) {
    // TODO: maybe, scale with deltaTime
    boid->position = Vector2Add(boid->position, boid->velocity);
    boid->position.x = Wrap(boid->position.x, 0, WINDOW_WIDTH);
    boid->position.y = Wrap(boid->position.y, 0, WINDOW_HEIGHT);
    boid->velocity = Vector2Add(boid->velocity, boid->acceleration);
    boid->velocity = Vector2ClampValue(boid->velocity, -MAX_VELOCITY, MAX_VELOCITY);
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

Vector2 GetBoidAlignmentForce(Boid* boid, LocalFlock* localFlock, float weight) {
    Vector2 averageVelocity = {0, 0};
    for (int i = 0; i < localFlock->size; i++) {
        averageVelocity = Vector2Add(averageVelocity, localFlock->boids[i].velocity);
    }
    if (localFlock->size > 0) {
        averageVelocity = Vector2Scale(averageVelocity, 1.0 / localFlock->size);
        averageVelocity = Vector2Subtract(averageVelocity, boid->velocity);
        averageVelocity = Vector2Scale(averageVelocity, weight);
        averageVelocity = Vector2ClampValue(averageVelocity, -MAX_ACCELERATION, MAX_ACCELERATION);
    }
    return averageVelocity;
}

Vector2 GetBoidCohesionForce(Boid* boid, LocalFlock* localFlock, float weight) {
    Vector2 averagePosition = {0, 0};
    for (int i = 0; i < localFlock->size; i++) {
        averagePosition = Vector2Add(averagePosition, localFlock->boids[i].position);
    }
    if (localFlock->size > 0) {
        averagePosition = Vector2Scale(averagePosition, 1.0 / localFlock->size);
        // get the vector from boid -> averagePosition
        averagePosition = Vector2Subtract(averagePosition, boid->position);
        // scale it
        averagePosition = Vector2Scale(averagePosition, weight);
        averagePosition = Vector2ClampValue(averagePosition, -MAX_ACCELERATION, MAX_ACCELERATION);
    }
    return averagePosition;
}

Vector2 GetBoidSeparationForce(Boid* boid, LocalFlock* localFlock, float weight) {
    Vector2 averageOppositeDirection = {0, 0};
    for (int i = 0; i < localFlock->size; i++) {
        // gets a vector that goes from other boid to me
        Vector2 oppositeDirection = Vector2Subtract(boid->position, localFlock->boids[i].position);
        // set the magnitude to be inversely proportional to the distance (or length)
        oppositeDirection = Vector2Scale(oppositeDirection, 1.0 / pow(Vector2Length(oppositeDirection), 2));
        //oppositeDirection = Vector2Normalize(oppositeDirection);
        averageOppositeDirection = Vector2Add(averageOppositeDirection, oppositeDirection);
    }
    if (localFlock->size > 0) {
        averageOppositeDirection = Vector2Scale(averageOppositeDirection, 1.0 / localFlock->size);
        averageOppositeDirection = Vector2Scale(averageOppositeDirection, weight);
        averageOppositeDirection = Vector2ClampValue(averageOppositeDirection, -MAX_ACCELERATION, MAX_ACCELERATION);
    }
    return averageOppositeDirection;
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
    InitWindow(WINDOW_WIDTH, WINDOW_HEIGHT, "Boids");
    SetTargetFPS(60);

    const int boidCount = 300;
    Boid boids[boidCount];

    for (int i = 0; i < boidCount; i++) {
        boids[i].position = RandomVector2(0, 800);
        boids[i].velocity = RandomVector2(-4, 4);
        boids[i].acceleration = (Vector2){ .x = 0, .y = 0 };
    }

    LocalFlock localFlock;

    while (!WindowShouldClose()) {
        BeginDrawing();
        ClearBackground(RAYWHITE);

        for (int i = 0; i < boidCount; i++) {
            GetLocalFlock(&boids[i], boids, boidCount, &localFlock, 50);
            Vector2 allignmentForce = GetBoidAlignmentForce(&boids[i], &localFlock, 0.1);
            Vector2 cohesionForce = GetBoidCohesionForce(&boids[i], &localFlock, 0.02);
            Vector2 separationForce = GetBoidSeparationForce(&boids[i], &localFlock, 25);
            //boids[i].acceleration = separationForce;
            boids[i].acceleration = Vector2Add(allignmentForce, cohesionForce);
            boids[i].acceleration = Vector2Add(boids[i].acceleration, separationForce);
            UpdateBoid(&boids[i]);
            DrawBoid(&boids[i]);
        }

        EndDrawing();
    }

    return 0;
}