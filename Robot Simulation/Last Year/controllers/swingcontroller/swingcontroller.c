/*
 * File:         void.c
 * Description:  This is an empty robot controller, the robot does nothing. 
 * Author:       www.cyberbotics.com
 * Note:         !!! PLEASE DO NOT MODIFY THIS SOURCE FILE !!!
 *               This is a system file that Webots needs to work correctly.
 */

#include <webots/utils/motion.h>
#include <webots/robot.h>
#include <webots/motor.h> 
#include <webots/camera.h>
#include <webots/distance_sensor.h>
#include <webots/accelerometer.h>
#include <webots/receiver.h>
#include <webots/emitter.h>
#include <webots/gyro.h>
#include <webots/inertial_unit.h>
#include <webots/touch_sensor.h>
#include <webots/led.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <assert.h>

#ifdef _MSC_VER
#define snprintf sprintf_s
#endif

#define PHALANX_MAX 8

static int time_step = -1;

static WbDeviceTag seatgyro;
static WbDeviceTag seatIU;
static WbDeviceTag seatEmitter;

static void find_and_enable_devices() {


  //new foot angle sensors
  seatgyro = wb_robot_get_device("Seat_Gyro");
  wb_gyro_enable(seatgyro, time_step);
  seatIU = wb_robot_get_device("Seat_IU");
  wb_inertial_unit_enable(seatIU, time_step);
  seatEmitter = wb_robot_get_device("SeatEmitter");
  
}


static void send_angles() {
  const double *seatangvel = wb_gyro_get_values(seatgyro);
  const double *seatangle = wb_inertial_unit_get_roll_pitch_yaw(seatIU);
  double angles[5] = {seatangvel[0],seatangvel[1],seatangle[0],seatangle[1],seatangle[2]};
  wb_emitter_send(seatEmitter, angles, 5*sizeof(double));
}

static void terminate() {
  // add you cleanup code here: write results, close files, free memory, etc.
  // ...

  wb_robot_cleanup();
}

static void simulation_step() {
  if (wb_robot_step(time_step) == -1)
    terminate();
}

// main function
int main(int argc, const char *argv[]) {
  
  // call this before any other call to a Webots function
  wb_robot_init();

  // simulation step in milliseconds
  time_step = wb_robot_get_basic_time_step();
  
  // initialize stuff
  find_and_enable_devices();
  while (1) {
    send_angles();
    simulation_step();
  }
  
  return 0;
}