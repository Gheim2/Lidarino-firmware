#include <Arduino.h>
#include "robot_core.h"

// Istanziamo il cervello del robot
LidarinoRobot robot;

void setup() {
    robot.begin();
}

void loop() {
    robot.update();
}