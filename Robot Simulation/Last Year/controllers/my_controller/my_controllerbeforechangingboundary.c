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
#include <webots/receiver.h>
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
static WbDeviceTag RShoulderRoll;
static WbDeviceTag LShoulderRoll;

static WbDeviceTag rightfootgyro;
static WbDeviceTag rightfootIU;
static WbDeviceTag Receiver;

static WbDeviceTag RElbowRoll;
static WbDeviceTag LElbowRoll;
static WbDeviceTag RElbowYaw;
static WbDeviceTag LElbowYaw;

static WbDeviceTag RHipPitch;
static WbDeviceTag LHipPitch;
static WbDeviceTag RHipYawPitch;
static WbDeviceTag LHipYawPitch;
static WbDeviceTag RKneePitch;
static WbDeviceTag LKneePitch;


static double Ms = 25.24294;
static double Ml = 1.17528;
static double Mb = 2.84115;
static double Mt = 25.24294 + 1.17528 + 2.84115;
static double Ls = 1.5;
static double Ll = 0.14809;
static double Lb = 0.2115;
static double g = 9.81;
static double maxHip = 0.;
static double minHip = -1.19;
static double maxKnee = 1.4;
static double minKnee = -0.09;
int legsToMax = 0;
int legsToMin = 0;
int bodyToMax = 0;
int bodyToMin = 0;

// motion file handles
static WbMotionRef hand_wave, forwards, backwards, side_step_left, side_step_right, turn_left_60, turn_right_60;
static WbMotionRef currently_playing = NULL;

static double maxPhalanxMotorPosition[PHALANX_MAX];
static double minPhalanxMotorPosition[PHALANX_MAX];

static void find_and_enable_devices() {


  //new foot angle sensors
  rightfootgyro = wb_robot_get_device("Right_Foot_Gyro");
  wb_gyro_enable(rightfootgyro, time_step);
  rightfootIU = wb_robot_get_device("Right_Foot_IU");
  wb_inertial_unit_enable(rightfootIU, time_step);
  Receiver = wb_robot_get_device("Robot_Receiver");
  wb_receiver_enable(Receiver, time_step);

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
  
    // shoulder roll motors
  RShoulderRoll = wb_robot_get_device("RShoulderRoll");
  LShoulderRoll = wb_robot_get_device("LShoulderRoll");

  // elbow roll motors
  RElbowRoll = wb_robot_get_device("RElbowRoll");
  LElbowRoll = wb_robot_get_device("LElbowRoll");
  
  // elbow yaw motors
  RElbowYaw = wb_robot_get_device("RElbowYaw");
  LElbowYaw = wb_robot_get_device("LElbowYaw");

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




static double * get_seat_angle() {
  static double returnangle[5];
  
  while (wb_receiver_get_queue_length(Receiver) > 0) {
    const double *angle = wb_receiver_get_data(Receiver);
    for(int i = 0; i<5; i++){
      returnangle[i] = angle[i];
    }
    
    //printf("%f %f %f %f %f \n",angle[0], angle[1], angle[2], angle[3], angle[4]);

    wb_receiver_next_packet(Receiver);
  }
  return returnangle;
}

static void print_angles() {
  const double *footangvel = wb_gyro_get_values(rightfootgyro);
  const double *footangle = wb_inertial_unit_get_roll_pitch_yaw(rightfootIU);
  const double *seatangle = get_seat_angle();
  //printf("%f %f %f  %f %f %f \n", footangle[0],footangle[1],footangle[2],seatangle[2],seatangle[3],seatangle[4]);
  double angdiff = footangle[1] - seatangle[3];
  printf("%f \n", angdiff);
}

static bool swing_horizontal() {
  const double *seatangle = get_seat_angle();
  double pitch = seatangle[3];
  bool horizontal = 0;
  //printf("%f %f \n",seatangle[0], seatangle[1]);
  if(pitch<0.03 && pitch>-0.03){
    horizontal = 1;
  }
  return horizontal;
}

static void print_angvel(double time) {
  const double *vel = wb_gyro_get_values(gyro);
  printf("%f %f\n", time, vel[1]);
}

static bool checkDirection() {
  bool forwards;
  //const double *vel = wb_gyro_get_values(gyro);
  const double *vel = get_seat_angle();
  if(vel[1]>0){
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

static void set_zero() {
  wb_motor_set_position(RShoulderPitch,0);
  wb_motor_set_position(LShoulderPitch,0);
  wb_motor_set_position(RShoulderRoll,0);
  wb_motor_set_position(LShoulderRoll,0);
  wb_motor_set_position(RElbowRoll,0);
  wb_motor_set_position(LElbowRoll,0);
  wb_motor_set_position(RElbowYaw,0);
  wb_motor_set_position(LElbowYaw,-0.7);
  wb_motor_set_position(RHipPitch,0);
  wb_motor_set_position(LHipPitch,0);
  wb_motor_set_position(RHipYawPitch,0);
  wb_motor_set_position(LHipYawPitch,0);
  wb_motor_set_position(RKneePitch,0);
  wb_motor_set_position(LKneePitch,0);
  printf("setting to zero\n");
}

static void sit_up() {
  
  wb_motor_set_position(RShoulderPitch,0);
  wb_motor_set_position(LShoulderPitch,0);
  wb_motor_set_position(RShoulderRoll,-1.);
  wb_motor_set_position(LShoulderRoll,1.);
  wb_motor_set_position(RElbowRoll,1.4);
  wb_motor_set_position(LElbowRoll,-1.4);
  wb_motor_set_position(RHipPitch,-1.1);
  wb_motor_set_position(LHipPitch,-1.1);
  wb_motor_set_position(RKneePitch,1);
  wb_motor_set_position(LKneePitch,1);
  printf("sitting\n");
}

static void pull_elbows(){
  wb_motor_set_position(RShoulderPitch,0);
  wb_motor_set_position(LShoulderPitch,0);
  wb_motor_set_position(RShoulderRoll,-1.);
  wb_motor_set_position(LShoulderRoll,1.);
  wb_motor_set_position(RElbowRoll,1.4);
  wb_motor_set_position(LElbowRoll,-1.4);
    wb_motor_set_position(RHipPitch,-1.1);
  wb_motor_set_position(LHipPitch,-1.1);
  printf("pulling elbosw\n");
}

static void push_elbows(){
  wb_motor_set_position(RElbowRoll,0);
  wb_motor_set_position(LElbowRoll,-0);
  wb_motor_set_position(RShoulderPitch,0.4);
  wb_motor_set_position(LShoulderPitch,0.4);
  wb_motor_set_position(RShoulderRoll,0);
  wb_motor_set_position(LShoulderRoll,0);
  wb_motor_set_position(RHipPitch,-0.8);
  wb_motor_set_position(LHipPitch,-0.8);
  printf("pushing elbosw\n");
}

static void open_elbows(){
  double posR = wb_motor_get_target_position(RElbowRoll);
  double posL = wb_motor_get_target_position(LElbowRoll);
  wb_motor_set_position(RElbowRoll,posR+0.05);
  wb_motor_set_position(LElbowRoll,posL+0.05);
  printf("Right elbow = %f Left elbow = %f\n",posR+0.05,posL+0.05);
}

static void close_elbows(){
  double posR = wb_motor_get_target_position(RElbowRoll);
  double posL = wb_motor_get_target_position(LElbowRoll);
  wb_motor_set_position(RElbowRoll,posR-0.05);
  wb_motor_set_position(LElbowRoll,posL-0.05);
  printf("Right elbow = %f Left elbow = %f\n",posR-0.05,posL-0.05);
}

static void open_shoulders(){
  double posR = wb_motor_get_target_position(RShoulderRoll);
  double posL = wb_motor_get_target_position(LShoulderRoll);
  wb_motor_set_position(RShoulderRoll,posR+0.05);
  wb_motor_set_position(LShoulderRoll,posL-0.05);
  printf("Right shoulder = %f Left shoulder = %f\n",posR+0.05,posL-0.05);
}

static void close_shoulders(){
  double posR = wb_motor_get_target_position(RShoulderRoll);
  double posL = wb_motor_get_target_position(LShoulderRoll);
  wb_motor_set_position(RShoulderRoll,posR-0.05);
  wb_motor_set_position(LShoulderRoll,posL+0.05);
  printf("Right shoulder = %f Left shoulder = %f\n",posR-0.05,posL+0.05);
}

static void elbow_yaw(){
  double posL = wb_motor_get_target_position(LElbowYaw);
  //double posL = wb_motor_get_target_position(LShoulderRoll);
  //wb_motor_set_position(RShoulderRoll,posR-0.05);
  wb_motor_set_position(LElbowYaw,posL+0.05);
  printf("Left elbow= %f\n",posL+0.05);
}

static void lie_down() {
  
  wb_motor_set_position(RShoulderPitch,-1.57);
  wb_motor_set_position(LShoulderPitch,-1.57);
  wb_motor_set_position(RHipPitch,0);
  wb_motor_set_position(LHipPitch,0);
  wb_motor_set_position(RKneePitch,0);
  wb_motor_set_position(LKneePitch,0);
  printf("lieing down\n");
}

static void extend_legs() {
  wb_motor_set_position(RKneePitch,0);
  wb_motor_set_position(LKneePitch,0);
  //printf("swinging legs\n");
}

static void contract_legs() {
  wb_motor_set_position(RKneePitch,1.57);
  wb_motor_set_position(LKneePitch,1.57);
  //printf("swinging legs\n");
}

static void lean_back_body() {
  wb_motor_set_position(RHipPitch,-0.2);
  wb_motor_set_position(LHipPitch,-0.2);
  //printf("swinging body\n");
}

static void lean_upright_body() {
  wb_motor_set_position(RHipPitch,-1.1);
  wb_motor_set_position(LHipPitch,-1.1); 
  //printf("swinging body\n");
}

static void lean_forward_body() {
  wb_motor_set_position(RHipPitch,-1.3);
  wb_motor_set_position(LHipPitch,-1.3);
  //printf("swinging body\n");
  }
  

static void move_body(int b, double angleChange){
  double posR = wb_motor_get_target_position(RHipPitch);
  double targR = posR+angleChange*b;
  if(targR>maxHip){
    targR = maxHip;
    //targR = minHip;
  }
  if(targR<minHip){
    targR = minHip;
    //targR = maxHip;
  }
  wb_motor_set_position(RHipPitch,targR);
  wb_motor_set_position(LHipPitch,targR);
}

static void set_body_limit(double angle){
  if(angle == maxHip){
    bodyToMax = 1;
  }
  else if(angle == minHip){
    bodyToMin = 1;
  }
  wb_motor_set_position(RHipPitch,angle);
  wb_motor_set_position(LHipPitch,angle);
}

static bool check_body_max(){
  double posR = wb_motor_get_target_position(RHipPitch);
  double diffFromMax = abs(posR-maxHip);
  if(diffFromMax < 0.1){
    bodyToMax = 0;
  }
}

static bool check_body_min(){
  double posR = wb_motor_get_target_position(RHipPitch);
  double diffFromMin = abs(posR-minKnee);
  if(diffFromMin < 0.1){
    bodyToMin = 0;
  }
}


static void move_legs(int l, double angleChange){
  double posR = wb_motor_get_target_position(RKneePitch);
  double targR = posR+angleChange*l;
  if(targR>maxKnee){
    targR = maxKnee;
    //targR = minKnee;
  }
  if(targR<minKnee){
    targR = minKnee;
    //targR = maxKnee;
  }
  wb_motor_set_position(RKneePitch,targR);
  wb_motor_set_position(LKneePitch,targR);
}

static void set_legs_limit(double angle){
  if(angle == maxKnee){
    legsToMax = 1;
  }
  else if(angle == minKnee){
    legsToMin = 1;
  }
  wb_motor_set_position(RKneePitch,angle);
  wb_motor_set_position(LKneePitch,angle);
}

static void check_legs_max(){
  double posR = wb_motor_get_target_position(RKneePitch);
  double diffFromMax = abs(posR-maxKnee);
  if(diffFromMax < 0.1){
    legsToMax = 0;
  }
}

static bool check_legs_min(){
  double posR = wb_motor_get_target_position(RKneePitch);
  double diffFromMin = abs(posR-minKnee);
  if(diffFromMin < 0.1){
    legsToMin = 0;
  }
}
  
  
static double predict_w(double currentang, double lastang) {
  double predict_vel = currentang-lastang;
  return predict_vel;
  }
  
static double predict_acc(double currentang, double lastang1, double lastang2) {
  double predict_vel1 = currentang-lastang1;
  double predict_vel2 = lastang1-lastang2;
  double predict_acc = predict_w(predict_vel1,predict_vel2);
  return predict_acc;
  }
  
static double knee_theta() {
  const double *footangvel = wb_gyro_get_values(rightfootgyro);
  const double *footangle = wb_inertial_unit_get_roll_pitch_yaw(rightfootIU);
  const double *seatangle = get_seat_angle();
  double angdiff = footangle[1] - seatangle[3];
  return angdiff;
  }
  
static double knee_w(double lastang){
  double currentang = knee_theta();
  return predict_w(currentang, lastang);
}   

static double knee_wdot(double lastang1, double lastang2){
  double currentang = knee_theta();
  return predict_acc(currentang, lastang1, lastang2);
}  

static double body_theta() {
  const double *bodyangle = wb_inertial_unit_get_roll_pitch_yaw(inertial_unit);
  const double *seatangle = get_seat_angle();
  double angdiff = bodyangle[1] - seatangle[3];
  return angdiff;
  }

static double body_w(double lastang){
  double currentang = body_theta();
  return predict_w(currentang, lastang);
}  

static double body_wdot(double lastang1, double lastang2){
  double currentang = body_theta();
  return predict_acc(currentang, lastang1, lastang2);
}  

static double swing_theta() {
  const double *seatangle = get_seat_angle();
  return seatangle[3];
  }
  
static double swing_w(double lastang){
  double currentang = swing_theta();
  return predict_w(currentang, lastang);
}  

static double swing_wdot(double lastang1, double lastang2){
  double currentang = swing_theta();
  return predict_acc(currentang, lastang1, lastang2);
}  
  
static void test_print(double kneeang1,double kneeang2,double bodyang1,double bodyang2,double swingang1,double swingang2){
//printf("%f %f %f %f %f %f \n",kneeang1, kneeang2, bodyang1, bodyang2, swingang1, swingang2);
  printf(" knee1: %f knee2: %f knee3: %f body1: %f body2: %f body3: %f  swing1: %f swing2: %f swing3: %f\n",
  knee_theta(), knee_w(kneeang1), knee_wdot(kneeang1,kneeang2),
  body_theta(), body_w(bodyang1), body_wdot(bodyang1, bodyang2),
  swing_theta(), swing_w(swingang1), swing_wdot(swingang1, swingang2));  
}

static double calcAcc(double kneeang1,double kneeang2,double bodyang1,double bodyang2,double swingang1,double swingang2){
  double lthet[3] = {knee_theta(),knee_w(kneeang1),knee_wdot(kneeang1,kneeang2)};
  double bthet[3] = {body_theta(),body_w(bodyang1),body_wdot(bodyang1, bodyang2)};
  double sthet[3] = {swing_theta(),swing_w(swingang1),swing_wdot(swingang1, swingang2)};
  double AccT = (Ml*Ll*lthet[1]*sin(sthet[0]-lthet[0])*(sthet[1]-lthet[1]) + 
                Mb*Lb*bthet[1]*sin(sthet[0]-bthet[0])*(sthet[1]-bthet[1]) -
                Ml*Ll*lthet[2]*cos(sthet[0]-lthet[0]) - 
                Mb*Lb*bthet[2]*cos(sthet[0]-bthet[0]) - 
                Ml*sthet[1]*Ll*lthet[1]*sin(sthet[0]-lthet[0]) - 
                Mb*Lb*bthet[1]*sthet[1]*sin(sthet[0] - bthet[0]) - 
                Mt*g*sin(sthet[0])
                )/(Mt*Ls);
  printf("acc = %f , swingacc = %f \n", AccT,sthet[2]);
  return AccT;
  }

static double testAcc(double kneeang1,double kneeang2,double bodyang1,double bodyang2,double swingang1,double swingang2){
  double lthet[3] = {knee_theta(),knee_w(kneeang1),knee_wdot(kneeang1,kneeang2)};
  double bthet[3] = {body_theta(),body_w(bodyang1),body_wdot(bodyang1, bodyang2)};
  double sthet[3] = {swing_theta(),swing_w(swingang1),swing_wdot(swingang1, swingang2)};
  double AccT;
  double greatestAcc = 0;
  int bestlchange;
  int bestbchange;
  double angleChange = 0.05;
  for(int lcount = -1; lcount < 2; lcount = lcount + 1){
  
    double newlthet = lthet[0] + lcount*angleChange;
    
    for(int bcount = -1; bcount < 2; bcount++){
    
      double newbthet = bthet[0] + bcount*angleChange;
      AccT = (Ml*Ll*lthet[1]*sin(sthet[0]-newlthet)*(sthet[1]-lthet[1]) + 
                Mb*Lb*bthet[1]*sin(sthet[0]-newbthet)*(sthet[1]-bthet[1]) -
                Ml*Ll*lthet[2]*cos(sthet[0]-newlthet) - 
                Mb*Lb*bthet[2]*cos(sthet[0]-newbthet) - 
                Ml*sthet[1]*Ll*lthet[1]*sin(sthet[0]-newlthet) - 
                Mb*Lb*bthet[1]*sthet[1]*sin(sthet[0] - newbthet) - 
                Mt*g*sin(sthet[0])
                )/(Mt*Ls);
      if(AccT>greatestAcc){
        greatestAcc = AccT;
        bestlchange = lcount;
        bestbchange = bcount;
        //printf("b = %d l = %d acc = %f\n",bestbchange,bestlchange, greatestAcc);
      }
      
    }
    
  }
  printf("b = %d l = %d acc = %f\n",bestbchange,bestlchange, greatestAcc);
}

static double bestAcc(bool direction, double kneeang1,double kneeang2,double bodyang1,double bodyang2,double swingang1,double swingang2){
  double lthet[3] = {knee_theta(),knee_w(kneeang1),knee_wdot(kneeang1,kneeang2)};
  double bthet[3] = {body_theta(),body_w(bodyang1),body_wdot(bodyang1, bodyang2)};
  double sthet[3] = {swing_theta(),swing_w(swingang1),swing_wdot(swingang1, swingang2)};
  double AccT;
  double greatestAcc;
  if(direction){
    greatestAcc = 0;
    }
  else{
    greatestAcc = 1000000;
    }
  int bestlchange;
  int bestbchange;
  double angleChange = 0.25;
  int minlcount;
  int maxlcount;
  int minbcount;
  int maxbcount;
  if(legsToMax || legsToMin){
    minlcount = 0;
    maxlcount = 1;
  }
  else{
    minlcount = -1;
    maxlcount = 2;
  }
  if(bodyToMax || bodyToMin){
    minbcount = 0;
    maxbcount = 1;
  }
  else{
    minbcount = -1;
    maxbcount = 2;
  }

  for(int lcount = minlcount; lcount < maxlcount; lcount = lcount + 1){
  
    double newlthet = lthet[0] + lcount*angleChange;
    
    for(int bcount = minbcount; bcount < maxbcount; bcount++){
    
      double newbthet = bthet[0] + bcount*angleChange;
      AccT = (Ml*Ll*lthet[1]*sin(sthet[0]-newlthet)*(sthet[1]-lthet[1]) + 
                Mb*Lb*bthet[1]*sin(sthet[0]-newbthet)*(sthet[1]-bthet[1]) -
                Ml*Ll*lthet[2]*cos(sthet[0]-newlthet) - 
                Mb*Lb*bthet[2]*cos(sthet[0]-newbthet) - 
                Ml*sthet[1]*Ll*lthet[1]*sin(sthet[0]-newlthet) - 
                Mb*Lb*bthet[1]*sthet[1]*sin(sthet[0] - newbthet) - 
                Mt*g*sin(sthet[0])
                )/(Mt*Ls);
      if(direction){
        if(AccT>greatestAcc){
          greatestAcc = AccT;
          bestlchange = lcount;
          bestbchange = bcount;
        }
      }
      else{
        if(AccT<greatestAcc){
          greatestAcc = AccT;
          bestlchange = lcount;
          bestbchange = bcount;
        }
      }
    }
    
  }
  if(maxbcount == 2){
    move_body(bestbchange,angleChange);
  }
  if(maxlcount == 2){
    move_legs(bestlchange,angleChange);
  }
  //printf("b = %d l = %d acc = %f\n",bestbchange,bestlchange, greatestAcc);
}


static double minmaxKneeAcc(bool direction, double kneeang1,double kneeang2,double bodyang1,double bodyang2,double swingang1,double swingang2){
  double lthet[3] = {knee_theta(),knee_w(kneeang1),knee_wdot(kneeang1,kneeang2)};
  double bthet[3] = {body_theta(),body_w(bodyang1),body_wdot(bodyang1, bodyang2)};
  double sthet[3] = {swing_theta(),swing_w(swingang1),swing_wdot(swingang1, swingang2)};
  double AccT;
  double newlthetmax =atan((0.5 + sin(maxKnee))/(-0.25+cos(maxKnee)));
  double newlthetmin =atan((0.5 + sin(minKnee))/(-0.25+cos(minKnee)));

  double AccTmax = (Ml*Ll*lthet[1]*sin(sthet[0]-newlthetmax)*(sthet[1]-lthet[1]) + 
                Mb*Lb*bthet[1]*sin(sthet[0]-bthet[0])*(sthet[1]-bthet[1]) -
                Ml*Ll*lthet[2]*cos(sthet[0]-newlthetmax) - 
                Mb*Lb*bthet[2]*cos(sthet[0]-bthet[0]) - 
                Ml*sthet[1]*Ll*lthet[1]*sin(sthet[0]-newlthetmax) - 
                Mb*Lb*bthet[1]*sthet[1]*sin(sthet[0] - bthet[0]) - 
                Mt*g*sin(sthet[0])
                )/(Mt*Ls);
  double AccTmin = (Ml*Ll*lthet[1]*sin(sthet[0]-newlthetmin)*(sthet[1]-lthet[1]) + 
                Mb*Lb*bthet[1]*sin(sthet[0]-bthet[0])*(sthet[1]-bthet[1]) -
                Ml*Ll*lthet[2]*cos(sthet[0]-newlthetmin) - 
                Mb*Lb*bthet[2]*cos(sthet[0]-bthet[0]) - 
                Ml*sthet[1]*Ll*lthet[1]*sin(sthet[0]-newlthetmin) - 
                Mb*Lb*bthet[1]*sthet[1]*sin(sthet[0] - bthet[0]) - 
                Mt*g*sin(sthet[0])
                )/(Mt*Ls);  
   double bestForwardAngle;
   double bestBackwardAngle;
   if(AccTmax>AccTmin){
     bestForwardAngle = maxKnee;
     bestBackwardAngle = minKnee;
   }   
   else{
     bestForwardAngle = minKnee;
     bestBackwardAngle = maxKnee;
   }  
   if(direction){
     set_legs_limit(bestForwardAngle);
   }
   else{
     set_legs_limit(bestBackwardAngle);
   }
}

static double minmaxBodyAcc(bool direction, double kneeang1,double kneeang2,double bodyang1,double bodyang2,double swingang1,double swingang2){
  double lthet[3] = {knee_theta(),knee_w(kneeang1),knee_wdot(kneeang1,kneeang2)};
  double bthet[3] = {body_theta(),body_w(bodyang1),body_wdot(bodyang1, bodyang2)};
  double sthet[3] = {swing_theta(),swing_w(swingang1),swing_wdot(swingang1, swingang2)};
  double AccT;
  double newbthetmax = maxHip;
  double newbthetmin = minHip;

  double AccTmax = (Ml*Ll*lthet[1]*sin(sthet[0]-lthet[0])*(sthet[1]-lthet[1]) + 
                Mb*Lb*bthet[1]*sin(sthet[0]-newbthetmax)*(sthet[1]-bthet[1]) -
                Ml*Ll*lthet[2]*cos(sthet[0]-lthet[0]) - 
                Mb*Lb*bthet[2]*cos(sthet[0]-newbthetmax) - 
                Ml*sthet[1]*Ll*lthet[1]*sin(sthet[0]-lthet[0]) - 
                Mb*Lb*bthet[1]*sthet[1]*sin(sthet[0] - newbthetmax) - 
                Mt*g*sin(sthet[0])
                )/(Mt*Ls);
  double AccTmin = (Ml*Ll*lthet[1]*sin(sthet[0]-lthet[0])*(sthet[1]-lthet[1]) + 
                Mb*Lb*bthet[1]*sin(sthet[0]-newbthetmin)*(sthet[1]-bthet[1]) -
                Ml*Ll*lthet[2]*cos(sthet[0]-lthet[0]) - 
                Mb*Lb*bthet[2]*cos(sthet[0]-newbthetmin) - 
                Ml*sthet[1]*Ll*lthet[1]*sin(sthet[0]-lthet[0]) - 
                Mb*Lb*bthet[1]*sthet[1]*sin(sthet[0] - newbthetmin) - 
                Mt*g*sin(sthet[0])
                )/(Mt*Ls);
   double bestForwardAngle;
   double bestBackwardAngle;
   if(AccTmax>AccTmin){
     bestForwardAngle = maxHip;
     bestBackwardAngle = minHip;
   }   
   else{
     bestForwardAngle = minHip;
     bestBackwardAngle = maxHip;
   }  
   if(direction){
     set_body_limit(bestForwardAngle);
   }
   else{
     set_body_limit(bestBackwardAngle);
   }
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
    case 'Q':
      open_elbows();
      break;
    case 'W':
      close_elbows();
      break;
    case 'E':
      open_shoulders();
      break;
    case 'R':
      close_shoulders();
      break;   
    case 'T':
      elbow_yaw();
      break;  
    case 'P':
      lie_down();
      break;
    case 'O':
      sit_up();
      break;
    case 'N':
      push_elbows();
      break;
    case 'M':
     pull_elbows();
      break;
    case 'Z':
     set_zero();
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
  
  
  
  double swingang = 0, bodyang = 0, kneeang = 0;
  double swingang1 = 0, bodyang1 = 0, kneeang1 = 0;
  double swingang2 = 0, bodyang2 = 0, kneeang2 = 0;
  
  int key = 0;
  int count = 0;
  double time = 0;
  bool lastDirection;
  bool travelling = 1;
  while (1) {
    key = wb_robot_keyboard_get_key();
    if(key){
      run_command(key);
    }
    count = count + 1;
    time = time + time_step/1000.;
    bool direction = checkDirection();
    if(legsToMax){
      check_legs_max();
    }
    if(legsToMin){
      check_legs_min();
    }
    if(bodyToMax){
      check_body_max();
    }
    if(bodyToMin){
      check_body_min();
    }
    if(count % 10 == 0){
      test_print(kneeang1, kneeang2, bodyang1, bodyang2, swingang1, swingang2);
      //calcAcc(kneeang1, kneeang2, bodyang1, bodyang2, swingang1, swingang2);
      //testAcc(kneeang1, kneeang2, bodyang1, bodyang2, swingang1, swingang2);
      bestAcc(direction,kneeang1, kneeang2, bodyang1, bodyang2, swingang1, swingang2);
      if(count == 10000){
        count = 0;
      }
    }
    swingang2 = swingang1;
    bodyang2 = bodyang1;
    kneeang2 = kneeang1;
    swingang1 = swingang;
    bodyang1 = bodyang;
    kneeang1 = kneeang;
    swingang = swing_theta();
    bodyang = body_theta();
    double legang = knee_theta();
    kneeang = atan((0.5 + sin(legang))/(-0.25+cos(legang)));
    //printf("%f %f \n", (kneeang/3.14)*90., ((atan((1+sin(kneeang))/cos(kneeang)))/3.14)*90);
    //printf("%d \n", direction);
    
    simulation_step();
    
  }
  
  return 0;
}
