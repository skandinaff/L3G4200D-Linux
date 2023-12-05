/*
    L3G4200D Triple Axis Gyroscope: Pitch, Roll and Yaw.
    Read more: http://www.jarzebski.pl/arduino/czujniki-i-sensory/3-osiowy-zyroskop-l3g4200d.html
    GIT: https://github.com/jarzebski/Arduino-L3G4200D
    Web: http://www.jarzebski.pl
    (c) 2014 by Korneliusz Jarzebski
*/

#include <stdio.h>
#include <stdlib.h>
#include <time.h>

#include "L3G4200D.h"

L3G4200D gyroscope;

// Timers

struct timespec start, end;
double elapsed;
float timeStep = 0.25;

// Pitch, Roll and Yaw values
float pitch = 0;
float roll = 0;
float yaw = 0;

int reading_num = 0;

Vector norm = Vector();
Vector raw = Vector();

void setup() 
{
  // Initialize L3G4200D
  printf("Initialize L3G4200D\n");

  // Set scale 2000 dps and 400HZ Output data rate (cut-off 50)
  while(!gyroscope.begin(L3G4200D_SCALE_250DPS, L3G4200D_DATARATE_100HZ_12_5))
  {
    printf("Could not find a valid L3G4200D sensor, check wiring!");
    usleep(500000); 
  }
 
  // Calibrate gyroscope. The calibration must be at rest.
  // If you don't want calibrate, comment this line.
  gyroscope.calibrate(100);
  usleep(500*1000); 
  printf("Setup done!\n");
}

void loop()
{
    clock_gettime(CLOCK_MONOTONIC, &start);

    // Read normalized values
    norm = gyroscope.readNormalize();
    raw = gyroscope.readRaw();
    reading_num ++;
    printf("Reading number: %d\n", reading_num);
    // Calculate Pitch, Roll and Yaw
    pitch = pitch + norm.YAxis * timeStep;
    roll = roll + norm.XAxis * timeStep;
    yaw = yaw + norm.ZAxis * timeStep;

    // Output raw
    printf("Pitch = %.2f Roll = %.2f Yaw = %.2f\n", pitch, roll, yaw);

    printf("X raw = %.2f Y raw = %.2f Y raw = %.2f\n", raw.XAxis, raw.YAxis, raw.ZAxis);
    // Wait to full timeStep period
    clock_gettime(CLOCK_MONOTONIC, &end);

    // Calculate elapsed time in milliseconds
    elapsed = (end.tv_sec - start.tv_sec) * 1000.0;
    elapsed += (end.tv_nsec - start.tv_nsec) / 1000000.0;

    // Wait to complete the full timeStep period
    if (elapsed < (timeStep * 1000))
    {
        usleep((timeStep * 1000 - elapsed) * 1000); // Convert to microseconds
    }
}
int main()
{
    setup();
    while (1) // Continuous loop
    {
        loop();
    }

    return 0;
}