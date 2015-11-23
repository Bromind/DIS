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

int Interconn[16] = {5,-4,-6,3,5,4,4,-20,-20,-15,-5,5,3,-5,-15,-15};
//int Interconn[16] = {8,4,-16,3,-15,4,4,6,15,-15,-5,5,3,15,-15,-20};

static WbDeviceTag ds[NB_SENSORS];    // Handle for the infrared distance sensors
static WbDeviceTag cam_tag;             // Camera
int robot_id;                       // Unique robot ID
char state;                         // Wall following state

int lmsg, rmsg;                     // Communication variables

WbDeviceTag receiver;		// Handle for the receiver node
WbDeviceTag emitter;

float rgb_emission[3] = {0.0, 0.0, 0.0};
float rgb_perception[3] = {0.0, 0.0, 0.0};

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/* helper functions */

// Generate random number in [0,1]
double rnd(void) {
  return ((double)rand())/((double)RAND_MAX);
}


void reset(void) 
{
    int i; // FOR-loop counter
  
    char cam[7]="camera";
    char s[4]="ps0";
    
    for(i=0; i<NB_SENSORS;i++) 
    {
        // the device name is specified in the world file
        ds[i]=wb_robot_get_device(s);      
        s[2]++; // increases the device number
        wb_distance_sensor_enable(ds[i],64);
    }
    
    cam_tag=wb_robot_get_device(cam);   
    wb_camera_enable(cam_tag,500);
    
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
    printf ("#%i received: (%f %f %f)\n", robot_id, rgb_perception[0], rgb_perception[1], rgb_perception[2]);
  }
}
    

void run(int ms)
{
    static int clock = 0; 
    int d1,d2;                  // motor speed 1 and 2
    int msl,msr;                // motor speed left and right
    int distances[NB_SENSORS];  // array keeping the distance sensor readings
    int sensor_nb;           // FOR-loop counters
    //const unsigned char *image; //camera width*camera height*4
    d1=0; d2=0;                
    
    //image=wb_camera_get_image(cam_tag);

    // receive emissions from other robots
    receive_local_emission();
 
    for(sensor_nb=0;sensor_nb<NB_SENSORS;sensor_nb++)
    {  // read sensor values and calculate motor speeds
        distances[sensor_nb] = wb_distance_sensor_get_value(ds[sensor_nb]);   
        d1 += (distances[sensor_nb]-300) * Interconn[sensor_nb];
        d2 += (distances[sensor_nb]-300) * Interconn[sensor_nb + NB_SENSORS];
    }

    d1 /= 80; d2 /= 80;        // Normalizing speeds
    msr = d1+BIAS_SPEED + 200*rnd(); msl = d2+BIAS_SPEED+ 200*rnd();  

    msr -= BIAS_SPEED * rnd()-10; msl -= BIAS_SPEED *rnd();  
    wb_differential_wheels_set_speed(msl,msr);

    send_local_emission();
  
    clock += ms;
    return;
}

int main(int argc, char **argv) {
  wb_robot_init();
  reset();
  
  while (wb_robot_step(TIME_STEP) != -1) {
    run(TIME_STEP);
  }
  
  wb_robot_cleanup();
  return 0;
}
