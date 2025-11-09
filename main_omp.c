#include <assert.h>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include "raylib.h"
#include "raymath.h"
#include <omp.h>
#include <string.h>

#include "timeit.h"

#define WINDOW_WIDTH 2000
#define WINDOW_HEIGHT 2000
#define MAX_LOCAL_FLOCK_SIZE 4096

#define MAX_VELOCITY 30
#define MAX_ACCELERATION 1

#define PERCEPTION_RADIUS 50
#define GRID_RESOLUTION 50

typedef struct {
    Vector2 position;
    Vector2 velocity;
    Vector2 acceleration;
} Boid;

typedef struct {
    Boid boids[MAX_LOCAL_FLOCK_SIZE];
    int size;
} LocalFlock;

typedef struct {
    Boid** boids;
    int size;
} GridCell;

typedef struct {
    GridCell** grid;
    int gridResolution;
    int gridHeight;
    int gridWidth;
} BoidGrid;

BoidGrid BoidGridAlloc(int resolution, int width, int height, int cellCapacity) {
    GridCell** cells = malloc(height * sizeof(GridCell*));

    if (cells == NULL) {
        perror("Failed to allocate grid rows");
        return (BoidGrid){ .grid = NULL };
    }

    GridCell* data_block = (GridCell*)malloc(height * width * sizeof(GridCell));
    if (data_block == NULL) {
        perror("Failed to allocate grid data block");
        free(cells);
        return (BoidGrid){ .grid = NULL };
    }

    cells[0] = (GridCell*)cells + height;
    for (int r = 1; r < height; r++) {
        cells[r] = cells[r-1] + width;
    }

    for (int r = 0; r < height; r++) {
        cells[r] = data_block + (r * width);
    }

    for (int i = 0; i < height * width; i++) {
        data_block[i].boids = (Boid**)malloc(cellCapacity * sizeof(Boid*));
        data_block[i].size = 0;

        if (data_block[i].boids == NULL) {
            perror("Failed to allocate internal cell array");
            for (int j = 0; j < i; j++) {
                free(data_block[j].boids);
            }
            free(data_block);
            free(cells);
            return (BoidGrid){ .grid = NULL };
        }
    }

    return (BoidGrid){
        .grid = cells,
        .gridResolution = resolution,
        .gridHeight = height,
        .gridWidth = width
    };
}

void BoidGridFree(BoidGrid* grid) {
    if (grid == NULL || grid->grid == NULL) return;

    GridCell* data_block = grid->grid[0];
    int totalCells = grid->gridHeight * grid->gridWidth;

    for (int i = 0; i < totalCells; i++) {
        free(data_block[i].boids);
    }

    free(data_block);

    free(grid->grid);

    grid->grid = NULL;
}

void DrawBoid(Boid* boid) {
    DrawCircleV(boid->position, 5, RED);
}

void UpdateBoid(Boid* boid) {
    boid->position = Vector2Add(boid->position, boid->velocity);
    boid->position.x = Wrap(boid->position.x, 0, WINDOW_WIDTH);
    boid->position.y = Wrap(boid->position.y, 0, WINDOW_HEIGHT);
    boid->velocity = Vector2Add(boid->velocity, boid->acceleration);
    boid->velocity = Vector2ClampValue(boid->velocity, -MAX_VELOCITY, MAX_VELOCITY);
}

void GetLocalFlockSlow(Boid* currentBoid, Boid* boids, const int size, LocalFlock* flock, float radius) {
    flock->size = 0;
    for (int i = 0; i < size; i++) {
        float dist = Vector2Distance(boids[i].position, currentBoid->position);
        if (dist < radius && currentBoid != &boids[i]) {
            //DrawLineV(currentBoid->position, boids[i].position, BLACK);
            flock->boids[flock->size] = boids[i];
            flock->size++;
        }
    }
}

void GetLocalFlock(Boid* current, BoidGrid* grid, int range, LocalFlock* flock, float radius) {
    const int row = current->position.x / grid->gridResolution;
    const int col = current->position.y / grid->gridResolution;
    flock->size=0;

    for (int dcol = -range; dcol <= range; dcol++) {

        for (int drow = -range; drow <= range; drow++) {
            int gridRow = row+drow;
            // if gridRol > gridHeight we get a segfault, so we wrap the grid
            gridRow = (gridRow+grid->gridHeight)%grid->gridHeight;

            int gridCol = col+dcol;
            gridCol = (gridCol+grid->gridWidth)%grid->gridWidth;

            GridCell* cell = &grid->grid[gridRow][gridCol];

            for (int i = 0; i < cell->size; i++) {
                float dist = Vector2Distance(cell->boids[i]->position, current->position);
                if (dist < radius && current != cell->boids[i]) {
                    flock->boids[flock->size] = *cell->boids[i];
                    flock->size++;
                    //DrawLineV(current->position, cell->boids[i]->position, BLACK);
                }
            }
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
        float distSqr = Vector2LengthSqr(oppositeDirection);
        if (distSqr > 0.0001f) {
            oppositeDirection = Vector2Scale(oppositeDirection, 1.0 / distSqr);
        }
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

    const int boidCount = 10000;
    Boid boids[boidCount];
    int cellCapacity = 256;
    static_assert(WINDOW_WIDTH%GRID_RESOLUTION==0 && WINDOW_HEIGHT%GRID_RESOLUTION==0);
    BoidGrid boidGrid = BoidGridAlloc(GRID_RESOLUTION, WINDOW_HEIGHT/GRID_RESOLUTION, WINDOW_WIDTH/GRID_RESOLUTION, cellCapacity);

    for (int i = 0; i < boidCount; i++) {
        boids[i] = (Boid) {
            .position = RandomVector2(0, 2000),
            .velocity = RandomVector2(-30, 30),
            .acceleration = (Vector2){ .x = 0, .y = 0 }
        };
    }

    double frameTimes = 0;
    double measurements = 0;

    while (!WindowShouldClose()) {
        double frame_time_start = omp_get_wtime();

#pragma omp for collapse(2)
        for (int row = 0; row < boidGrid.gridHeight; row++) {
            for (int col = 0; col < boidGrid.gridWidth; col++) {
                boidGrid.grid[row][col].size = 0;
            }
        }

        for (int i = 0; i < boidCount; i++) {
            int col = (int)(boids[i].position.y / boidGrid.gridResolution);
            int row = (int)(boids[i].position.x / boidGrid.gridResolution);
            GridCell* cell = &boidGrid.grid[row][col];
            if (cell->size < cellCapacity) {
                cell->boids[cell->size] = &boids[i];
                cell->size++;
            }
        }

#pragma omp parallel for schedule(dynamic)
        for (int i = 0; i < boidCount; i++) {
            LocalFlock threadLocalFlock;

            GetLocalFlock(&boids[i], &boidGrid, 1, &threadLocalFlock, PERCEPTION_RADIUS);

            Vector2 allignmentForce = GetBoidAlignmentForce(&boids[i], &threadLocalFlock, 0.1);
            Vector2 cohesionForce = GetBoidCohesionForce(&boids[i], &threadLocalFlock, 0.02);
            Vector2 separationForce = GetBoidSeparationForce(&boids[i], &threadLocalFlock, 25);
            boids[i].acceleration = Vector2Add(allignmentForce, cohesionForce);
            boids[i].acceleration = Vector2Add(boids[i].acceleration, separationForce);
        } // implicit barrier

#pragma omp parallel for schedule(dynamic)
        for (int i = 0; i < boidCount; i++) {
            UpdateBoid(&boids[i]);
        } // implicit barrier

        // Drawing must be done in a singlethreaded manner

        BeginDrawing();
        ClearBackground(RAYWHITE);
        for (int i = 0; i < boidCount; i++) {
            DrawBoid(&boids[i]);
        }
        double frame_time = omp_get_wtime() - frame_time_start;
        frameTimes += frame_time;
        measurements++;
        DrawRectangle(0, 0, WINDOW_WIDTH/2, 70, RAYWHITE);
        DrawText(TextFormat("FRAME TIME: %f", frame_time), 10, 10, 50, GREEN);
        EndDrawing();
    }

    BoidGridFree(&boidGrid);

    return 0;
}