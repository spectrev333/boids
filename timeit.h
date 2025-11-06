//
// Created by leonardo on 06/11/25.
//

#ifndef BOIDS_EXECISE_TIMEIT_H
#define BOIDS_EXECISE_TIMEIT_H

/**
 * @brief Measures the execution time of a code block using omp_get_wtime().
 *
 * This macro uses a 'for' loop trick to execute code before the block (get start time)
 * and after the block (get end time, calculate duration, and print result).
 *
 * Usage:
 * TIMEIT("My Task Label") {
 * // Code to be timed
 * }
 *
 * @param label The string literal to print along with the duration.
 */
#define TIMEIT(label) \
for (double __start = omp_get_wtime(), __end = 0.0, __duration = 0.0; \
/* Condition: runs once while __end is 0.0 */ \
__end == 0.0; \
/* Post-Execution: Runs after the block completes */ \
__end = omp_get_wtime(), \
__duration = __end - __start, \
printf("\n-> %s took %.6f seconds\n", label, __duration)) \
/* The user's block becomes the body of this 'if (1)' */ \
if (1)

#endif //BOIDS_EXECISE_TIMEIT_H