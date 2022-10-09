#include <iostream>
#include "td3.hpp"
#include "support.hpp"
#include <stdlib.h>
#include <math.h>       // sin, cos
#include <assert.h>

using namespace std;

using namespace support;

double* extend_array(double* array, int length, int new_size) {
  // Increases the size of the array of double array from length to new_size elements
    double*newarr = new double[new_size];
    for (int i = 0; i < new_size; i++){
        if (i < length){newarr[i] = array[i];}
        else{newarr[i] = 0;}
    }
    delete[] array;
    return newarr;
}

double* shrink_array(double* array, int length, int new_size) {
  // Shrinks the array of double array from size length to size new_size
    double*newarr = new double[new_size];
    for (int i = 0; i < new_size; i++){newarr[i] = array[i];}
    delete[] array;
    return newarr ;
}

double* append_to_array(double element,
                        double* array,
                        int &current_size,
                        int &max_size) {
  // Appends the number contained in element to the array, and returns the array pointer as output.
    if (current_size == max_size){array = extend_array(array, current_size, current_size+5);
                max_size +=5;}
    current_size++;
    *(array + current_size-1) = element;
    return array;
}

double* remove_from_array(double* array,
                          int &current_size,
                          int &max_size) {
  /* Removes the last element from array. The function further shrinks the array when the difference between the total
    number of used elements total_elements and the array maximum size array_size is at least 5 */
    array = shrink_array(array, current_size, current_size - 1);
    current_size--;
    if (max_size-current_size >= 5) {max_size -= 5;}
    return array;
}

bool simulate_projectile(const double magnitude, const double angle,
                         const double simulation_interval,
                         double *targets, int &tot_targets,
                         int *obstacles, int tot_obstacles,
                         double* &telemetry,
                         int &telemetry_current_size,
                         int &telemetry_max_size) {
  /* Simulates the motion of a projectile, checking its collision with targets and obstacles and stores in the telemetry
    pointer the address to an array of double containing, in sequence, the time, the x coordinate, and the y coordinate
    simulated for the projectile.*/
  bool hit_target, hit_obstacle;
  double v0_x, v0_y, x, y, t;
  double PI = 3.14159265;
  double g = 9.8;

  v0_x = magnitude * cos(angle * PI / 180);
  v0_y = magnitude * sin(angle * PI / 180);

  t = 0;
  x = 0;
  y = 0;
  hit_target = false;
  hit_obstacle = false;
  while (y >= 0 && (! hit_target) && (! hit_obstacle)) {
    double * target_coordinates = find_collision(x, y, targets, tot_targets);
    telemetry = append_to_array(t, telemetry, telemetry_current_size, telemetry_max_size);
    telemetry = append_to_array(x, telemetry, telemetry_current_size, telemetry_max_size);
    telemetry = append_to_array(y, telemetry, telemetry_current_size, telemetry_max_size);
    if (target_coordinates != NULL) {
      remove_target(targets, tot_targets, target_coordinates);
      hit_target = true;
    } else if (find_collision(x, y, obstacles, tot_obstacles) != NULL) {
      hit_obstacle = true;
    } else {
      t = t + simulation_interval;
      y = v0_y * t  - 0.5 * g * t * t;
      x = v0_x * t;
    }
  }

  return hit_target;
}


void merge_telemetry(double **telemetries,
                     int tot_telemetries,
                     int *telemetries_sizes,
                     double* &global_telemetry,
                     int &global_telemetry_current_size,
                     int &global_telemetry_max_size) {
  // Merges the telemetries of multiple projectiles into a single telemetry.
    int size = 0;
    for (int i = 0; i < tot_telemetries; i++){
        size += telemetries_sizes[i];
    }
    global_telemetry = extend_array(global_telemetry, global_telemetry_max_size, size);
    for (int j = 0 ; j < tot_telemetries ; j++){
        int i = 0;
        while (i < telemetries_sizes[j]){
            global_telemetry = append_to_array(telemetries[j][i], global_telemetry, global_telemetry_current_size, global_telemetry_max_size);
            i++;}}


    int k = 0;
    for (;k < global_telemetry_current_size;){
       if (global_telemetry[k+3] < global_telemetry[k]){
           double tmp = global_telemetry[k] ;
           double tmp1 = global_telemetry[k+1];
           double tmp2 = global_telemetry[k+2];
           global_telemetry[k] = global_telemetry[k+3];
           global_telemetry[k+1] = global_telemetry[k+4];
           global_telemetry[k+2] = global_telemetry[k+5];
           global_telemetry[k+3] = tmp;
           global_telemetry[k+4] = tmp1;
           global_telemetry[k+5] = tmp2;
           k = 0 ;}
       else{k += 3 ;}
}
}
