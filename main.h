//
// Created by jquist on 28-10-18.
//

#ifndef HIGHERLEVELCONTROLLER_MAIN_H
#define HIGHERLEVELCONTROLLER_MAIN_H

#endif //HIGHERLEVELCONTROLLER_MAIN_H

struct Goal {
    double position[3];
    double time;
};

struct Drone {
    double position[3];
    double velocity[3];
    struct Goal goal[50];
    int goal_index;
    double last_updated_time;
};


int main();
double get_time_since_start();
void update_drone(struct Drone *drone);
void print_drone(struct Drone *drone);
