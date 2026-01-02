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

#define WORLD_SIZE 5000

#define MIN_VELOCITY 10
#define MAX_VELOCITY 30
#define MAX_ACCELERATION 1

#define PERCEPTION_RADIUS 50
#define GRID_RESOLUTION 50

#define BENCHMARK_MODE
#define BENCHMARK_FRAMES 100

typedef struct {
    Vector2 position;
    Vector2 velocity;
    Vector2 acceleration;
} Boid;

typedef struct {
    Vector2 velocitiesSum;
    Vector2 positionsSum;
    Vector2 oppositeDirectionsSum;
    int size;
} LocalFlock;

typedef struct {
    Boid **boids;
    int size;
} GridCell;

typedef struct {
    GridCell **grid;
    int gridResolution;
    int gridHeight;
    int gridWidth;
} BoidGrid;

BoidGrid BoidGridAlloc(int resolution, int width, int height, int cellCapacity) {
    GridCell **cells = malloc(height * sizeof(GridCell *));

    if (cells == NULL) {
        perror("Failed to allocate grid rows");
        return (BoidGrid){.grid = NULL};
    }

    GridCell *data_block = (GridCell *) malloc(height * width * sizeof(GridCell));
    if (data_block == NULL) {
        perror("Failed to allocate grid data block");
        free(cells);
        return (BoidGrid){.grid = NULL};
    }

    cells[0] = (GridCell *) cells + height;
    for (int r = 1; r < height; r++) {
        cells[r] = cells[r - 1] + width;
    }

    for (int r = 0; r < height; r++) {
        cells[r] = data_block + (r * width);
    }

    for (int i = 0; i < height * width; i++) {
        data_block[i].boids = (Boid **) malloc(cellCapacity * sizeof(Boid *));
        data_block[i].size = 0;

        if (data_block[i].boids == NULL) {
            perror("Failed to allocate internal cell array");
            for (int j = 0; j < i; j++) {
                free(data_block[j].boids);
            }
            free(data_block);
            free(cells);
            return (BoidGrid){.grid = NULL};
        }
    }

    return (BoidGrid){
        .grid = cells,
        .gridResolution = resolution,
        .gridHeight = height,
        .gridWidth = width
    };
}

void BoidGridFree(BoidGrid *grid) {
    if (grid == NULL || grid->grid == NULL) return;

    GridCell *data_block = grid->grid[0];
    int totalCells = grid->gridHeight * grid->gridWidth;

    for (int i = 0; i < totalCells; i++) {
        free(data_block[i].boids);
    }

    free(data_block);

    free(grid->grid);

    grid->grid = NULL;
}

void DrawBoid(Boid *boid) {
    //DrawCircleV(boid->position, 5, RED);
    DrawRectangle(boid->position.x - 5, boid->position.y - 5, 10, 10, RED);
}

void UpdateBoid(Boid *boid) {
    boid->position = Vector2Add(boid->position, boid->velocity);
    boid->position.x = Wrap(boid->position.x, 0, WORLD_SIZE);
    boid->position.y = Wrap(boid->position.y, 0, WORLD_SIZE);
    boid->velocity = Vector2Add(boid->velocity, boid->acceleration);
    //boid->velocity = Vector2ClampValue(boid->velocity, -MAX_VELOCITY, MAX_VELOCITY);
    float speedSqr = Vector2LengthSqr(boid->velocity);
    if (speedSqr > MAX_VELOCITY * MAX_VELOCITY) {
        boid->velocity = Vector2Scale(Vector2Normalize(boid->velocity), MAX_VELOCITY);
    } else if (speedSqr < MIN_VELOCITY * MIN_VELOCITY) {
        boid->velocity = Vector2Scale(Vector2Normalize(boid->velocity), MIN_VELOCITY);
    }
}


void GetLocalFlock(Boid *current, BoidGrid *grid, int range, LocalFlock *flock, float radius) {
    const int row = current->position.x / grid->gridResolution;
    const int col = current->position.y / grid->gridResolution;
    flock->positionsSum = (Vector2){0, 0};
    flock->velocitiesSum = (Vector2){0, 0};
    flock->oppositeDirectionsSum = (Vector2){0, 0};
    flock->size = 0;

    for (int dcol = -range; dcol <= range; dcol++) {
        for (int drow = -range; drow <= range; drow++) {
            // if gridRol > gridHeight we get a segfault, so we wrap the grid
            int gridRow = (row + drow) % grid->gridHeight;
            if (gridRow < 0) gridRow += grid->gridHeight;

            int gridCol = (col + dcol) % grid->gridWidth;
            if (gridCol < 0) gridCol += grid->gridWidth;

            GridCell *cell = &grid->grid[gridRow][gridCol];

            for (int i = 0; i < cell->size; i++) {
                float dist = Vector2DistanceSqr(cell->boids[i]->position, current->position);
                if (dist < radius * radius && current != cell->boids[i]) {
                    flock->velocitiesSum = Vector2Add(flock->velocitiesSum, cell->boids[i]->velocity);
                    flock->positionsSum = Vector2Add(flock->positionsSum, cell->boids[i]->position);
                    Vector2 oppositeDirection = Vector2Subtract(current->position, cell->boids[i]->position);
                    if (dist > 0.0001f) {
                        oppositeDirection = Vector2Scale(oppositeDirection, 1.0f / dist);
                        flock->oppositeDirectionsSum = Vector2Add(flock->oppositeDirectionsSum, oppositeDirection);
                    }
                    flock->size++;
                }
            }
        }
    }
}

Vector2 GetBoidAlignmentForce(Boid *boid, LocalFlock *localFlock, float weight) {
    Vector2 averageVelocity = localFlock->velocitiesSum;
    if (localFlock->size > 0) {
        averageVelocity = Vector2Scale(averageVelocity, 1.0 / localFlock->size);
        averageVelocity = Vector2Subtract(averageVelocity, boid->velocity);
        averageVelocity = Vector2Scale(averageVelocity, weight);
        averageVelocity = Vector2ClampValue(averageVelocity, -MAX_ACCELERATION, MAX_ACCELERATION);
    }
    return averageVelocity;
}

Vector2 GetBoidCohesionForce(Boid *boid, LocalFlock *localFlock, float weight) {
    Vector2 averagePosition = localFlock->positionsSum;
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

Vector2 GetBoidSeparationForce(Boid *boid, LocalFlock *localFlock, float weight) {
    Vector2 averageOppositeDirection = localFlock->oppositeDirectionsSum;
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
    return min + ((float) rand() / RAND_MAX) * range;
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

    srand(10);

    const int boidCount = 50000;
    Boid boids[boidCount];
    int cellCapacity = 256;
    BoidGrid boidGrid = BoidGridAlloc(GRID_RESOLUTION, WORLD_SIZE / GRID_RESOLUTION, WORLD_SIZE / GRID_RESOLUTION,
                                      cellCapacity);

    for (int i = 0; i < boidCount; i++) {
        boids[i] = (Boid){
            .position = RandomVector2(0, WORLD_SIZE),
            .velocity = RandomVector2(-30, 30),
            .acceleration = (Vector2){.x = 0, .y = 0}
        };
    }

    LocalFlock localFlock;

    double frameTimes = 0;
    double measurements = 0;

    Camera2D cam;
    cam.offset = (Vector2){0, 0};
    cam.target = (Vector2){0, 0};
    cam.rotation = 0;
    cam.zoom = 1;

#ifdef BENCHMARK_MODE
    for (int i = 0; i < BENCHMARK_FRAMES; i++) {
#else
        while (!WindowShouldClose()) {
#endif
        if (IsMouseButtonDown(MOUSE_BUTTON_RIGHT)) {
            Vector2 delta = Vector2Scale(GetMouseDelta(), 1 / cam.zoom);
            cam.target = Vector2Subtract(cam.target, delta);
        }

        if (GetMouseWheelMove() != 0) {
            cam.target = GetScreenToWorld2D(GetMousePosition(), cam);
            cam.offset = GetMousePosition();
            if (GetMouseWheelMove() < 0) {
                cam.zoom *= 0.75f;
            } else {
                cam.zoom /= 0.75f;
            }
        }

        double frame_time_start = omp_get_wtime();
        BeginDrawing();
        BeginMode2D(cam);
        ClearBackground(RAYWHITE);

        for (int row = 0; row < boidGrid.gridHeight; row++) {
            for (int col = 0; col < boidGrid.gridWidth; col++) {
                boidGrid.grid[row][col].size = 0;
            }
        }

        for (int i = 0; i < boidCount; i++) {
            int col = (int) (boids[i].position.y / boidGrid.gridResolution) % boidGrid.gridWidth;
            int row = (int) (boids[i].position.x / boidGrid.gridResolution) % boidGrid.gridHeight;
            GridCell *cell = &boidGrid.grid[row][col];
            if (cell->size < cellCapacity) {
                cell->boids[cell->size] = &boids[i];
                cell->size++;
            }
        }

        for (int i = 0; i < boidCount; i++) {
            GetLocalFlock(&boids[i], &boidGrid, 1, &localFlock, PERCEPTION_RADIUS);
            Vector2 allignmentForce = GetBoidAlignmentForce(&boids[i], &localFlock, 0.1f);
            Vector2 cohesionForce = GetBoidCohesionForce(&boids[i], &localFlock, 0.03f);
            Vector2 separationForce = GetBoidSeparationForce(&boids[i], &localFlock, 50);
            boids[i].acceleration = Vector2Add(allignmentForce, cohesionForce);
            boids[i].acceleration = Vector2Add(boids[i].acceleration, separationForce);
            UpdateBoid(&boids[i]);
            DrawBoid(&boids[i]);
        }

        EndMode2D();
        double frame_time = omp_get_wtime() - frame_time_start;
        frameTimes += frame_time;
        measurements++;
        DrawRectangle(0, 0, WINDOW_WIDTH / 2, 70, RAYWHITE);
        DrawText(TextFormat("FRAME TIME: %f", frame_time), 10, 10, 50, GREEN);
        EndDrawing();
    }

    CloseWindow();
    BoidGridFree(&boidGrid);

    printf("Average frame time: %f", frameTimes / measurements);
    return 0;
}
