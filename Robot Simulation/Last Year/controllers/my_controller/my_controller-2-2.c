//---------------------------------------------------------------------------------
//  Description:  Example C controller program for Nao robot.
//                This demonstrates how to access sensors and actuators
//---------------------------------------------------------------------------------

#include <webots/utils/motion.h>
#include <webots/robot.h>
#include <webots/motor.h> 
#include <webots/camera.h>
#include <webots/distance_sensor.h>
#include <webots/accelerometer.h>
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

// simulated devices
static WbDeviceTag CameraTop, CameraBottom;            // cameras
static WbDeviceTag us[2];                              // ultra sound sensors
static WbDeviceTag accelerometer, gyro, inertial_unit; // inertial unit
static WbDeviceTag fsr[2];                             // force sensitive resistors
static WbDeviceTag lfoot_lbumper, lfoot_rbumper;       // left foot bumpers
static WbDeviceTag rfoot_lbumper, rfoot_rbumper;       // right foot bumpers
static WbDeviceTag leds[7];                            // controllable led groupsstatic WbDeviceTag lphalanx[PHALANX_MAX];
static WbDeviceTag rphalanx[PHALANX_MAX];              // right hand motors
static WbDeviceTag lphalanx[PHALANX_MAX];              // left hand motors
static WbDeviceTag RShoulderPitch;
static WbDeviceTag LShoulderPitch;
static WbDeviceTag RHipPitch;
static WbDeviceTag LHipPitch;
static WbDeviceTag RHipYawPitch;
static WbDeviceTag LHipYawPitch;
static WbDeviceTag RKneePitch;
static WbDeviceTag LKneePitch;
// motion file handles
static WbMotionRef hand_wave, forwards, backwards, side_step_left, side_step_right, turn_left_60, turn_right_60;
static WbMotionRef currently_playing = NULL;

static double maxPhalanxMotorPosition[PHALANX_MAX];
static double minPhalanxMotorPosition[PHALANX_MAX];

static void find_and_enable_devices() {

  // camera
  CameraTop = wb_robot_get_device("CameraTop");
  CameraBottom = wb_robot_get_device("CameraBottom");
  wb_camera_enable(CameraTop, 4 * time_step);
  wb_camera_enable(CameraBottom, 4 * time_step);

  // accelerometer
  accelerometer = wb_robot_get_device("accelerometer");
  wb_accelerometer_enable(accelerometer, time_step);

  // gyro
  gyro = wb_robot_get_device("gyro");
  wb_gyro_enable(gyro, time_step);
  
  // inertial unit
  inertial_unit = wb_robot_get_device("inertial unit");
  wb_inertial_unit_enable(inertial_unit, time_step);

  // ultrasound sensors
  us[0] = wb_robot_get_device("Sonar/Left");
  us[1] = wb_robot_get_device("Sonar/Right");
  int i;
  for (i = 0; i < 2; i++)
    wb_distance_sensor_enable(us[i], time_step);

  // foot sensors
  fsr[0] = wb_robot_get_device("LFsr");
  fsr[1] = wb_robot_get_device("RFsr");
  wb_touch_sensor_enable(fsr[0], time_step);
  wb_touch_sensor_enable(fsr[1], time_step);

  // foot bumpers
  lfoot_lbumper = wb_robot_get_device("LFoot/Bumper/Left");
  lfoot_rbumper = wb_robot_get_device("LFoot/Bumper/Right");
  rfoot_lbumper = wb_robot_get_device("RFoot/Bumper/Left");
  rfoot_rbumper = wb_robot_get_device("RFoot/Bumper/Right");
  wb_touch_sensor_enable(lfoot_lbumper, time_step);
  wb_touch_sensor_enable(lfoot_rbumper, time_step);
  wb_touch_sensor_enable(rfoot_lbumper, time_step);
  wb_touch_sensor_enable(rfoot_rbumper, time_step);

  // There are 7 controlable LED groups in Webots
  leds[0] = wb_robot_get_device("ChestBoard/Led");
  leds[1] = wb_robot_get_device("RFoot/Led");
  leds[2] = wb_robot_get_device("LFoot/Led");
  leds[3] = wb_robot_get_device("Face/Led/Right");
  leds[4] = wb_robot_get_device("Face/Led/Left");
  leds[5] = wb_robot_get_device("Ears/Led/Right");
  leds[6] = wb_robot_get_device("Ears/Led/Left");
  
  // get phalanx motor tags
  // the real Nao has only 2 motors for RHand/LHand
  // but in Webots we must implement RHand/LHand with 2x8 motors
  for (i = 0; i < PHALANX_MAX; i++) {
    char name[32];
    sprintf(name, "LPhalanx%d", i + 1);
    lphalanx[i] = wb_robot_get_device(name);
    sprintf(name, "RPhalanx%d", i + 1);
    rphalanx[i] = wb_robot_get_device(name);
    
    // assume right and left hands have the same motor position bounds
    maxPhalanxMotorPosition[i] = wb_motor_get_max_position(rphalanx[i]);
    minPhalanxMotorPosition[i] = wb_motor_get_min_position(rphalanx[i]);
  }
  
  // shoulder pitch motors
  RShoulderPitch = wb_robot_get_device("RShoulderPitch");
  LShoulderPitch = wb_robot_get_device("LShoulderPitch");

  // HipYawPitch
  RHipYawPitch = wb_robot_get_device("RHipYawPitch");
  LHipYawPitch = wb_robot_get_device("LHipYawPitch");
  
   // HipPitch
  RHipPitch = wb_robot_get_device("RHipPitch");
  LHipPitch = wb_robot_get_device("LHipPitch");
  
  // HipPitch
  RKneePitch = wb_robot_get_device("RKneePitch");
  LKneePitch = wb_robot_get_device("LKneePitch");

  // keyboard
  wb_robot_keyboard_enable(10 * time_step); 
}

// load motion files
static void load_motion_files() {
  hand_wave = wbu_motion_new("../../motions/HandWave.motion");
  forwards = wbu_motion_new("../../motions/Forwards50.motion");
  backwards = wbu_motion_new("../../motions/Backwards.motion");
  side_step_left = wbu_motion_new("../../motions/SideStepLeft.motion");
  side_step_right = wbu_motion_new("../../motions/SideStepRight.motion");
  turn_left_60 = wbu_motion_new("../../motions/TurnLeft60.motion");
  turn_right_60 = wbu_motion_new("../../motions/TurnRight60.motion");
}

static void start_motion(WbMotionRef motion) {
  
  // interrupt current motion
  if (currently_playing)
    wbu_motion_stop(currently_playing);
  
  // start new motion
  wbu_motion_play(motion);
  currently_playing = motion;
}

static double clamp(double value, double min, double max) {
  if (min > max) {
    assert(0);
    return value;
  }
  return value < min ? min : value > max ? max : value;
}

// the accelerometer axes are oriented as on the real robot
// however the sign of the returned values may be opposite
static void print_acceleration() {
  const double *acc = wb_accelerometer_get_values(accelerometer);
  printf("----------accelerometer----------\n");
  printf("acceleration: [ x y z ] = [%f %f %f]\n", acc[0], acc[1], acc[2]);
}

// the gyro axes are oriented as on the real robot
// however the sign of the returned values may be opposite
static void print_gyro() {
  const double *vel = wb_gyro_get_values(gyro);
  printf("----------gyro----------\n");
  printf("angular velocity: [ x y ] = [%f %f]\n", vel[0], vel[1]);
}

static void print_angvel(double time) {
  const double *vel = wb_gyro_get_values(gyro);
  printf("%f %f\n", time, vel[1]);
}

static bool checkDirection() {
  bool forwards;
  const double *vel = wb_gyro_get_values(gyro);
  if(vel[1]<0){
    forwards = 1;
    //printf("forwards");
  }
  else{
    forwards = 0;
    //printf("backwards");
  }
  return forwards;
}


// the InertialUnit roll/pitch angles are equal to naoqi's AngleX/AngleY
static void print_inertial_unit() {
  const double *rpy = wb_inertial_unit_get_roll_pitch_yaw(inertial_unit);
  printf("----------inertial unit----------\n");
  printf("roll/pitch/yaw: = [%f %f %f]\n", rpy[0], rpy[1], rpy[2]);
}

static void print_foot_sensors() {
  const double *fsv[2] = {wb_touch_sensor_get_values(fsr[0]),
                    wb_touch_sensor_get_values(fsr[1])}; // force sensor values
  
  double l[4], r[4];
  double newtonLeft = 0, newtonRight = 0;

  // The coefficients were calibrated against the real 
  // robot so as to obtain realistic sensor values.                             
  l[0] = fsv[0][2]/3.4 + 1.5*fsv[0][0] + 1.15*fsv[0][1]; // Left Foot Front Left
  l[1] = fsv[0][2]/3.4 + 1.5*fsv[0][0] - 1.15*fsv[0][1]; // Left Foot Front Right
  l[2] = fsv[0][2]/3.4 - 1.5*fsv[0][0] - 1.15*fsv[0][1]; // Left Foot Rear Right
  l[3] = fsv[0][2]/3.4 - 1.5*fsv[0][0] + 1.15*fsv[0][1]; // Left Foot Rear Left
    
  r[0] = fsv[1][2]/3.4 + 1.5*fsv[1][0] + 1.15*fsv[1][1]; // Right Foot Front Left
  r[1] = fsv[1][2]/3.4 + 1.5*fsv[1][0] - 1.15*fsv[1][1]; // Right Foot Front Right
  r[2] = fsv[1][2]/3.4 - 1.5*fsv[1][0] - 1.15*fsv[1][1]; // Right Foot Rear Right
  r[3] = fsv[1][2]/3.4 - 1.5*fsv[1][0] + 1.15*fsv[1][1]; // Right Foot Rear Left
  
  int i;
  for (i = 0; i < 4; ++i) {
    l[i] = clamp(l[i], 0, 25);
    r[i] = clamp(r[i], 0, 25);
    newtonLeft += l[i];
    newtonRight += r[i];
  }
  
  printf("----------foot sensors----------\n");
  printf("   left       right\n");
  printf("+--------+ +--------+\n");
  printf("|%3.1f  %3.1f| |%3.1f  %3.1f|  front\n", l[0], l[1], r[0], r[1]);
  printf("|        | |        |\n");
  printf("|%3.1f  %3.1f| |%3.1f  %3.1f|  back\n", l[3], l[2], r[3], r[2]);
  printf("+--------+ +--------+\n");
  printf("total: %g Newtons, %g kilograms\n", newtonLeft + newtonRight, (newtonLeft + newtonRight) / 9.81);
}

static void print_foot_bumpers() {
  int ll = (int)wb_touch_sensor_get_value(lfoot_lbumper);
  int lr = (int)wb_touch_sensor_get_value(lfoot_rbumper);
  int rl = (int)wb_touch_sensor_get_value(rfoot_lbumper);
  int rr = (int)wb_touch_sensor_get_value(rfoot_rbumper);

  printf("----------foot bumpers----------\n");
  printf("   left       right\n");
  printf("+--------+ +--------+\n");
  printf("|%d      %d| |%d      %d|\n", ll, lr, rl, rr);
  printf("|        | |        |\n");
  printf("|        | |        |\n");
  printf("+--------+ +--------+\n");
}

static void print_ultrasound_sensors() {
  double dist[2];
  int i;
  for (i = 0; i < 2; i++)
    dist[i] = wb_distance_sensor_get_value(us[i]);

  printf("-----ultrasound sensors-----\n");
  printf("left: %f m, right %f m\n", dist[0], dist[1]);
}

static void print_camera_image(WbDeviceTag camera) {

  const int SCALED = 2;

  int width = wb_camera_get_width(camera);
  int height = wb_camera_get_height(camera);

  // read rgb pixel values from the camera
  const unsigned char *image = wb_camera_get_image(camera);
  
  printf("----------camera image (grey levels)---------\n");
  printf("original resolution: %d x %d, scaled to %d x %d\n",
    width, height, width / SCALED, height / SCALED);
  
  int y, x;
  char *line = malloc(width / SCALED + 1);
  line[width / SCALED] = 0;  // add line termination
  for (y = 0; y < height; y += SCALED) {
    int count = 0;
    for (x = 0; x < width; x += SCALED) {
      unsigned char grey = wb_camera_image_get_grey(image, width, x, y);
      line[count++] = '0' + grey * 9 / 255;
    }
    line[count++] = 0;
    printf("%s\n", line);
  }
  free(line);
}

static void set_all_leds_color(int rgb) {

  // these leds take RGB values
  int i;
  for (i = 0; i < 5; i++)
    wb_led_set(leds[i], rgb);

  // ear leds are single color (blue)
  // and take values between 0 - 255
  wb_led_set(leds[5], rgb & 0xff);
  wb_led_set(leds[6], rgb & 0xff);
}

static void set_hands_angle(double angle) {
  // we must activate the 8 phalanx motors
  int j;
  double clampedAngle;
  for (j = 0; j < PHALANX_MAX; j++) {
    clampedAngle = angle;
    if (clampedAngle > maxPhalanxMotorPosition[j])
      clampedAngle = maxPhalanxMotorPosition[j];
    else if (maxPhalanxMotorPosition[j] < minPhalanxMotorPosition[j])
      clampedAngle = minPhalanxMotorPosition[j];
    
    if (rphalanx[j])
      wb_motor_set_position(rphalanx[j], clampedAngle);
    if (lphalanx[j])
      wb_motor_set_position(lphalanx[j], clampedAngle);
  }
}

static void sit_up() {
  
  wb_motor_set_position(RShoulderPitch,0);
  wb_motor_set_position(LShoulderPitch,0);
  wb_motor_set_position(RHipPitch,-1.1);
  wb_motor_set_position(LHipPitch,-1.1);
  wb_motor_set_position(RKneePitch,1);
  wb_motor_set_position(LKneePitch,1);
  printf("sitting\n");
}

static void lie_down() {
  
  wb_motor_set_position(RShoulderPitch,1);
  wb_motor_set_position(LShoulderPitch,1);
  wb_motor_set_position(RHipPitch,0);
  wb_motor_set_position(LHipPitch,0);
  wb_motor_set_position(RKneePitch,0);
  wb_motor_set_position(LKneePitch,0);
  printf("lieing down\n");
}

static void swing_legs() {
  wb_motor_set_position(RShoulderPitch,0);
  wb_motor_set_position(LShoulderPitch,0);
  wb_motor_set_position(RHipPitch,0);
  wb_motor_set_position(LHipPitch,0);
  //wb_motor_set_position(RHipYawPitch,0);
  //wb_motor_set_position(LHipYawPitch,0);
  wb_motor_set_position(RKneePitch,0);
  wb_motor_set_position(LKneePitch,0);
  printf("swinging legs\n");
}

static void swing_body() {
  wb_motor_set_position(RShoulderPitch,1);
  wb_motor_set_position(LShoulderPitch,1);
  wb_motor_set_position(RHipPitch,-1);
  wb_motor_set_position(LHipPitch,-1);
  //wb_motor_set_position(RHipYawPitch,-1.7);
  //wb_motor_set_position(LHipYawPitch,-1.7);
  wb_motor_set_position(RKneePitch,1.5);
  wb_motor_set_position(LKneePitch,1.5);
  printf("swinging body\n");
}



static void print_help() {
  printf("----------nao_demo----------\n");
  printf("Select the robot and use the keyboard to control it:\n");
  printf("(The 3D window need to be focused)\n");
  printf("[Up][Down]: move one step forward/backwards\n");
  printf("[<-][->]: side step left/right\n");
  printf("[Shift] + [<-][->]: turn left/right\n");
  printf("[U]: print ultrasound sensors\n");
  printf("[A]: print accelerometer\n");
  printf("[G]: print gyro\n");
  printf("[I]: print inertial unit (roll/pitch/yaw)\n");
  printf("[F]: print foot sensors\n");
  printf("[B]: print foot bumpers\n");
  printf("[Home][End]: print scaled top/bottom camera image\n");
  printf("[PageUp][PageDown]: open/close hands\n");
  printf("[7][8][9]: change all leds RGB color\n");
  printf("[0]: turn all leds off\n");
  printf("[H]: print this help message\n");
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

static void run_command(int key) {

  switch (key) {
    case WB_ROBOT_KEYBOARD_LEFT:
      start_motion(side_step_left);
      break;
    case WB_ROBOT_KEYBOARD_RIGHT:
      start_motion(side_step_right);
      break;
    case WB_ROBOT_KEYBOARD_UP:
      start_motion(forwards);
      break;
    case WB_ROBOT_KEYBOARD_DOWN:
      start_motion(backwards);
      break;
    case WB_ROBOT_KEYBOARD_LEFT | WB_ROBOT_KEYBOARD_SHIFT:
      start_motion(turn_left_60);
      break;
    case WB_ROBOT_KEYBOARD_RIGHT | WB_ROBOT_KEYBOARD_SHIFT:
      start_motion(turn_right_60);
      break;
    case 'A':
      print_acceleration();
      break;
    case 'G':
      print_gyro();
      break;
    case 'I':
      print_inertial_unit();
      break;
    case 'F':
      print_foot_sensors();
      break;
    case 'B':
      print_foot_bumpers();
      break;
    case 'U':
      print_ultrasound_sensors();
      break;
    case 'P':
      lie_down();
      break;
    case 'O':
      sit_up();
      break;
    case WB_ROBOT_KEYBOARD_HOME:
      print_camera_image(CameraTop);
      break;
    case WB_ROBOT_KEYBOARD_END:
      print_camera_image(CameraBottom);
      break;
    case WB_ROBOT_KEYBOARD_PAGEUP:
      set_hands_angle(0.96);
      break;
    case WB_ROBOT_KEYBOARD_PAGEDOWN:
      set_hands_angle(0.0);
      break;      
    case '7':
      set_all_leds_color(0xff0000); // red
      break;
    case '8':
      set_all_leds_color(0x00ff00); // green
      break;
    case '9':
      set_all_leds_color(0x0000ff); // blue
      break;
    case '0':
      set_all_leds_color(0x000000); // off
      break;
    case 'H':
      print_help();
      break;
  }
}

// main function
int main(int argc, const char *argv[]) {
  
  // call this before any other call to a Webots function
  wb_robot_init();

  // simulation step in milliseconds
  time_step = wb_robot_get_basic_time_step();
  
  // initialize stuff
  find_and_enable_devices();
  load_motion_files();
  
  int key = 0;
  int count = 0;
  double time = 0;
  int ignore = 0;
  bool lastDirection;
  while (1) {
    key = wb_robot_keyboard_get_key();
    if(key){
      run_command(key);
    }
    count = count + 1;
    time = time + time_step/1000.;
    bool direction = checkDirection();
    if(ignore == 0){
      if(direction==1 && lastDirection == 0){
        //printf("going forwards now\n");
        swing_legs();
        ignore = 10;
      }
      else if(direction == 0 && lastDirection == 1){
        //printf("going backwards now\n");
        swing_body();
        ignore = 10;
      }
    }
    else{
      ignore = ignore - 1;
    }
    lastDirection = direction;
    if(count % 10 == 0){
      //print_angvel(time);
      if(count == 100){
        count = 0;
      }
    }
    
    //print_gyro(); 
    simulation_step();
    
  }
  
  return 0;
}
