/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * file:        epuck_crown.c
 * description: Base project with 3-channel emissions and 3 types of tasks
 *
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */


#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <unistd.h>

// #include <sstream>
// using namespace std;

#include <webots/robot.h>
#include <webots/differential_wheels.h>
#include <webots/emitter.h>
#include <webots/receiver.h>
#include <webots/camera.h>
#include <webots/distance_sensor.h>
#include <webots/radio.h>

#define DEBUG 1
#define TIME_STEP 64
#define FLOCK_SIZE 5


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/* e-Puck parameters */
#define NB_SENSORS           8
#define BIAS_SPEED           300
#define WIDTH                52  //pixel width of the camera
#define HEIGHT               39  //pixel height of the camera


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/* Problem Description */

// PROBLEM TYPE
#define DETERMINISTIC        1  // 0= infinite steepness 1= above fitness used
#define ADAPTIVE             0  // 0=fixed, 1=adaptive thresholds
#define PUBLIC               0  // 0=local estimation (no information sharing),
                                // 1= global dissemination (collaboration with neighbors)

// THRESHOLD BASED ALGORITHM PARAMETERS
#define THRESHOLD            3  // value of homogeneous threshold
#define STEEPNESS            10  // steepness of threshold cutoff
#define ABANDON              0.1 // probability of giving up task (unused)

// COLORS
#define NB_COLORS            3 // Number of colors
#define COLOR_BLIND          0 // 0=colors are ignored  1=colors are considered as different tasks
#define LOST_THRESHOLD       2 // number of pixels before a target color is considered lost (used to change FSM state)
#define NO_COLOR            -1 // Nothing special detected on screen (robot in front / gone through cylinder / wall)
#define RED                  0
#define GREEN                1
#define BLUE                 2

// STATES OF THE FINITE MACHINE
#define STATE1     1
#define STATE2     2

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/* Global variables */

// Braitenberg
int Interconn[16] = {5,-4,-6,3,5,4,4,-20,-20,-15,-5,5,3,-5,-15,-15};
//int Interconn[16] = {8,4,-16,3,-15,4,4,6,15,-15,-5,5,3,15,-15,-20};

//Robot
static WbDeviceTag ds[NB_SENSORS];    // Handle for the infrared distance sensors
static WbDeviceTag cam_tag;             // Camera
int robot_id;                       // Unique robot ID

// FSM
int state = STATE1; // state of FSM: either the robot is in state 1 (searching) or state 2 (going towards a color)

// Colors & Thresholds
int chosen_color = NO_COLOR; // color chosen by the robot
int pos_color[NB_COLORS] = {0, 0, 0}; // average position of the chosen cylinder (camera)
int size_color[NB_COLORS]; // size of largest colored cylinders
int threshold[NB_COLORS] = {THRESHOLD, THRESHOLD, THRESHOLD}; // stores the thresholds corresponding to the colors
int stimulus[NB_COLORS]; // stimuli corresponding to the colors

// Motors speed
int msr, msl;

// Proximity sensors
int distances[NB_SENSORS];  // array keeping the distance sensor readings

// Communication
int lmsg, rmsg; // Communication variables

WbDeviceTag receiver;		// Handle for the receiver node
WbDeviceTag emitter;

float rgb_emission[3] = {0.0, 0.0, 0.0};
float rgb_perception[3] = {0.0, 0.0, 0.0};

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/* Helper Functions */

//Generate random number in [0,1]
double rnd(void) {
  return ((double)rand())/((double)RAND_MAX);
}

// Thresholds Calculations
float sigmoid(float threshold, float stimulus){
  return (pow(stimulus,STEEPNESS))/(pow(stimulus,STEEPNESS)+pow(threshold,STEEPNESS));
}

int checkThreshold(double RAND, int c){
  if (threshold[c]<=0) return 1;
  if (DETERMINISTIC == 0){
    if(RAND < sigmoid(threshold[c], stimulus[c])) return 1;
    else return 0;
  }
  else{
    if(stimulus[c] > threshold[c]) return 1;
    else return 0;
  }
}

// Decision function
void updateRobot(){
  /* size_color est notre stimulus pour l'instant mais tu peux lui appliquer des
     transformations ici et les stocker dans stimulus[NB_COLORS]. On peut employer
     des boucles for mais on voulait que le code soit compréhensible dans un
     premier temps. De toutes manières, le fonctionnement de la décision est
     totalement débile pour l'instant. */

  if(ADAPTIVE==1){
    //adaptThresholds(); // Update thresholds based on time spent in search mode or receiver
  }

  if(PUBLIC==1){
    /* Ici coder la prise en compte des différentes émissions reçues */
    //receive_local_emission();
    //stimulus = "something";
  }
  else { //No communication, the stimulus is only the number of colored pixels seen
    int c;
    for(c=0; c<NB_COLORS; c++){
      stimulus[c] = size_color[c];
    }
  }

  // if in search mode, try to pick a color according to probabilities
  if(state == STATE1){
    double RAND = rnd();
    if(checkThreshold(RAND, RED)){
      chosen_color = RED;
    }
    else if(checkThreshold(RAND, GREEN)){
      chosen_color = GREEN;
    }
    else if(checkThreshold(RAND, BLUE)){
      chosen_color = BLUE;
    }
    else {
      chosen_color = NO_COLOR;
    }
    // if cylinder chosen, go chromataxis mode (STATE2)
    if(chosen_color != NO_COLOR){
      state = STATE2;
    }
  }

  // if in chromataxis mode, check if cylinder was lost
  else{ // state==STATE2
    if(chosen_color == RED && stimulus[RED] < LOST_THRESHOLD){
      chosen_color = NO_COLOR;
    }
    else if (chosen_color == GREEN && stimulus[GREEN] < LOST_THRESHOLD){
      chosen_color = NO_COLOR;
    }
    else if (chosen_color == BLUE && stimulus[BLUE] < LOST_THRESHOLD){
      chosen_color = NO_COLOR;
    }
    // if cylinder lost, go random search mode (STATE1)
    if(chosen_color == NO_COLOR){
      state = STATE1;
    }
  }

}
////////////////////////////////////////////////////////////////////////////////
// TO BE COMPLETED !
void processImage(const unsigned char *image) {

/* Il faut mettre à jour pos_color et size_color avec, respectivement, la position
   du milieu du cylindre le plus gros et sa taille (pour chaque couleur)*/

   int sizes[3] = {0, 0, 0};
   int min_colors[3] = {WIDTH, WIDTH, WIDTH};
   int max_colors[3] = {0, 0, 0};
   //int mean_color[3] = {WIDTH/2, WIDTH/2, WIDTH/2};

   int x;
   int color;

   // Analyze a row of pixels
   for(x = 0; x < WIDTH; x++) {
     int r = wb_camera_image_get_red(image, WIDTH, x, 18);
     int g = wb_camera_image_get_green(image, WIDTH, x, 18);
     int b = wb_camera_image_get_blue(image, WIDTH, x, 18);

     int scan[3] = {r, g, b};

     // Check if pixel is colored
     for (color = 0; color < NB_COLORS; color++) {

       if (scan[color] >= 250) {
         sizes[color]++; // update size of color

         // update min and max pixels
         if (x < min_colors[color]) {
           min_colors[color] = x;
         }
         else if (x > max_colors[color]) {
           max_colors[color] = x;
         }

       }
     }
   }

   // Compute the size of the cylinders and the position of their centers
   for (color = 0; color < NB_COLORS; color++) {
     size_color[color] = sizes[color];
     pos_color[color] = (int) (max_colors[color] + min_colors[color])/2;
   }

}
////////////////////////////////////////////////////////////////////////////////

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/* Motors Speed Calculations */

// Obstacle Avoidance
void obstacle_avoidance(int *d1, int *d2){
  int sensor_nb;           // FOR-loop counters
  for(sensor_nb=0;sensor_nb<NB_SENSORS;sensor_nb++)
  {  // read sensor values and calculate motor speeds
      distances[sensor_nb] = wb_distance_sensor_get_value(ds[sensor_nb]);
      *d1 += (distances[sensor_nb]-300) * Interconn[sensor_nb];
      *d2 += (distances[sensor_nb]-300) * Interconn[sensor_nb + NB_SENSORS];
  }
  *d1 /= 80; *d2 /= 80; // Normalizing speeds
}

// Chromataxis
void chromataxis(int pos_chosen_color){
  // compute motor speed to go towards the chosen cylinder
  int mr,ml;    // motor speed from chromataxis
  int d1 = 0;   // motor speed from braitenberg (right)
  int d2 = 0;  // motor speed from braitenberg (left)
  printf("#%i chose to go %d\n", robot_id, chosen_color);

  mr = -10 * (pos_chosen_color-26);
  ml = 10 * (pos_chosen_color-26);

  obstacle_avoidance(&d1, &d2);

  msr = d1 + BIAS_SPEED + mr;
  msl = d2 + BIAS_SPEED + ml;

  wb_differential_wheels_set_speed(msl,msr);
}

// Random Turn
void randomTurn(){
  int RAND = rnd();
  msr = -200*RAND - 50;
  msl = 200*RAND + 50;

  wb_differential_wheels_set_speed(msl,msr);
}

// Random Walk
void randomWalk(){
  int d1 = 0;   // motor speed from braitenberg (right)
  int d2 = 0;  // motor speed from braitenberg (left)

  obstacle_avoidance(&d1, &d2);

  msr = d1+BIAS_SPEED + 200*rnd(); msl = d2+BIAS_SPEED+ 200*rnd();
  msr -= BIAS_SPEED * rnd()-10; msl -= BIAS_SPEED *rnd();

  wb_differential_wheels_set_speed(msl,msr);
}

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

void reset(void) {
    int i; // FOR-loop counter

    char cam[7]="camera";
    char s[4]="ps0";

    for(i=0; i<NB_SENSORS;i++) {
        // the device name is specified in the world file
        ds[i]=wb_robot_get_device(s);
        s[2]++; // increases the device number
        wb_distance_sensor_enable(ds[i],64);
    }

    cam_tag=wb_robot_get_device(cam);
    wb_camera_enable(cam_tag,500);
    printf("%d , %d \n", wb_camera_get_width(cam_tag), wb_camera_get_height(cam_tag));

    // read robot id and state from the robot's name
    char* robot_name; robot_name=(char*) wb_robot_get_name();
    sscanf(robot_name,"epuck_%d_%d",&robot_id,(int*)&state);

    emitter = wb_robot_get_device("emitter");
    if (emitter==0) printf("missing emitter\n");

    //wb_emitter_enable(emitter,64);

    receiver = wb_robot_get_device("receiver");
    if (receiver==0) printf("missing receiver\n");
    wb_receiver_enable(receiver,64);

    srand(getpid()); // Seed random generator
}

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/* Communication */

void send_local_emission() {
  char buffer[255];
  int i;
  // compute emission strength for each task typedef
  for (i=0; i<3; i++)  rgb_emission[i] = rnd();  // random value for testing

  // send emission
  sprintf(buffer, "r:%f g:%f b:%f", rgb_emission[0], rgb_emission[1], rgb_emission[2]);
  wb_emitter_send(emitter,buffer,strlen(buffer));

}

void receive_local_emission() {
  char *inbuffer;
  float signal_strength;
  int count = 0;
  int i;
  float rgb_received[3] = {0.0, 0.0, 0.0};
  for (i=0; i<3; i++)  rgb_perception[i] = 0.0;
  while ((wb_receiver_get_queue_length(receiver) > 0)&& (count<FLOCK_SIZE)) {

    inbuffer = (char*) wb_receiver_get_data(receiver);
    signal_strength = wb_receiver_get_signal_strength(receiver);
    sscanf(inbuffer,"r:%f g:%f b:%f",&rgb_received[0], &rgb_received[1], &rgb_received[2]);

    //printf ("RSSI: %f ", signal_strength);
    for (i=0; i<3; i++) {
      // sum locally received concentration of emissions, weighted by signal strength
      // signal strength is 1/r^2 (r = distance between robots)
      // this function can be adapted to get a differently shaped potential field
      rgb_perception[i] += rgb_received[i] * signal_strength;
    }
    count++;
    wb_receiver_next_packet(receiver);
  }
  if (count>1) {
    for (i=0; i<3; i++)  rgb_perception[i] /= (float)count;
    //printf ("#%i received: (%f %f %f)\n", robot_id, rgb_perception[0], rgb_perception[1], rgb_perception[2]);
  }
}

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/* Run */

void run(int ms) {
    static int clock = 0;
    const unsigned char *image; //camera width*camera height*4

    image=wb_camera_get_image(cam_tag);
    processImage(image); // processing camera image and updating pos_color, size_color
    updateRobot();
    //printf("#%i : %d \n", robot_id, state);

    if (state == STATE1){
      randomTurn();
    }
    else if(state == STATE2){
      chromataxis(pos_color[chosen_color]);
    }

    // receive emissions from other robots
    receive_local_emission();

    send_local_emission();

    clock += ms;
    return;
}

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/* Main */

int main(int argc, char **argv) {
  wb_robot_init();
  reset();

  while (wb_robot_step(TIME_STEP) != -1) {
    run(TIME_STEP);
  }

  wb_robot_cleanup();
  return 0;
}
