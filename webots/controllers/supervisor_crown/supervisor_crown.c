/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * file:        epuck_crown.c
 * description: Base project with 3-channel emissions and 3 types of tasks
 *
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */


#include <assert.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <stdio.h>
#include <unistd.h>
#include <time.h>

#include <webots/emitter.h>
#include <webots/receiver.h>
#include <webots/supervisor.h>
#include <webots/robot.h>

#define ROBOTS 5
#define MAX_EVENTS 10
#define STEP_SIZE 64
#define ARENA_SIZE 1.9
#define ROBOT_RADIUS 0.035 //radius of the epuck (used to detect collision)
#define EVENT_RADIUS 0.025 //radius of the events (used to detect collision)
#define WHEEL_RADIUS 0.0265 //radius of the wheel of the epuck (used to compute the metric)

#define EVENT_PRODUCTIVITY_STEP 0.02  // amount removed from event (out of 1.0) if robot is processing it

// Performance measure
#define EVENTS_TO_HANDLE 10 //Number of events handled to consider the simulation successful
#define MAX_DURATION 2 // Number of iterations to do before resetting

static WbNodeRef rob[ROBOTS];         // References to robots
static WbFieldRef robTrans[ROBOTS];   // Reference to track the position of the robots

typedef struct event_t {
  WbNodeRef event;     // Reference to the events
  WbFieldRef eventTrans;// Reference to track the position of the events
  double state;
} event_t;

event_t events[MAX_EVENTS];

WbDeviceTag      rec[ROBOTS];         // Supervisor receivers
char status[ROBOTS];

static double red[3] = {1.0,0.0,0.0};
static double green[3] = {0.0,1.0,0.0};
static double blue[3] = {0.0,0.0,1.0};

const char rob_prefix[] = "epuck_";   // robot naming convention
const char rec_prefix[] = "rec";      // supervisor receives from source
const char emi_prefix[] = "emi";      // supervisor emits to robot mics

bool printed;

// Performance measure
int events_handled = 0;
int iterations = 1;
double mean_perf = 0.0;
double distance_travelled[5] = {0.0, 0.0, 0.0, 0.0, 0.0};
double previous_x[5] = {0.0, 0.0, 0.0, 0.0, 0.0};
double previous_y[5] = {0.0, 0.0, 0.0, 0.0, 0.0};


// Generate random number in [0,1]
double rnd(void) {
  return ((double)rand())/((double)RAND_MAX);
}

void randomize_event_position(int event_index) {
  char s[50];
  char *eventprefix = (char *) "e";
  double pos[3];

  sprintf(s, "%s%d", eventprefix, event_index);
  //wb_supervisor_node_get_from_def(s);

  // Define position of event and place it there
  pos[0] = ARENA_SIZE*rnd()-ARENA_SIZE/2.0;
  pos[1] = 0.01;
  pos[2] = ARENA_SIZE*rnd()-ARENA_SIZE/2.0;

  wb_supervisor_field_set_sf_vec3f(wb_supervisor_node_get_field(wb_supervisor_node_get_from_def(s),"translation"), pos);
}

void randomize_event_color(int event_index) {
  char s[50];
  char *eventprefix = (char *) "e";

  double* color = red;
  int color_select = (int)(3*rnd());
  switch (color_select) {
    case 0:  color=red; break;
    case 1:  color=green; break;
    case 2:  color=blue;  break;
  }
  sprintf(s, "%s%d_material", eventprefix, event_index);
  WbNodeRef eventAppearance  = wb_supervisor_node_get_from_def(s);
  WbFieldRef eventColor =  wb_supervisor_node_get_field(eventAppearance,"diffuseColor");
  wb_supervisor_field_set_sf_color(eventColor, color);
  eventColor =  wb_supervisor_node_get_field(eventAppearance,"emissiveColor");
  wb_supervisor_field_set_sf_color(eventColor, color);
}

/* Detects if an robot has handled an event (i.e. has bounced into it) */
bool are_colliding(WbFieldRef robTransRef, WbFieldRef eveTransRef) {
  const double *p1 = wb_supervisor_field_get_sf_vec3f(robTransRef);
  const double *p2 = wb_supervisor_field_get_sf_vec3f(eveTransRef);
  double dx = p2[0] - p1[0];
  double dz = p2[2] - p1[2];
  // true if the distance between the center of the event and the center of the robots are closer than the sum of both radii
  return sqrt(dx * dx + dz * dz) < 0.95 *EVENT_RADIUS + ROBOT_RADIUS;
}

/* Reset the supervisor */
void reset(void) {

  int i;
  char s[50];
  char *eventprefix = (char *) "e";

  printed = false;

  for (i=0;i<ROBOTS;i++)
  {
    char aux[15];
    /* Get and save a reference to the robot. */
    sprintf(aux,"%s%d",rob_prefix,i+1);
//    rob_name << rob_prefix << i+1; // << "_0";

    rob[i] = wb_supervisor_node_get_from_def(aux);
    robTrans[i] = wb_supervisor_node_get_field(rob[i],"translation");
    previous_x[i] = wb_supervisor_field_get_sf_vec3f(robTrans[i])[0];
    previous_y[i] = wb_supervisor_field_get_sf_vec3f(robTrans[i])[2];

    /* Get robot emitters */
    strcpy(aux,emi_prefix);
    sprintf(aux,"%s%d",aux,i+1);
  }

  printf("Initializing events...\n");

  for (i=0; i<MAX_EVENTS; i++)
  {
    sprintf(s, "%s%d", eventprefix, i);
    // Define position of event and place it there
    events[i].event = wb_supervisor_node_get_from_def(s);
    events[i].eventTrans = wb_supervisor_node_get_field(events[i].event,"translation");
    events[i].state = 1.0;
    randomize_event_position(i);
    randomize_event_color(i);
  }

}


static int run(int ms) {



  static unsigned long long int clock = 0;
  static unsigned long long int temp_clock = 0;

  int i, j;
  // Computing the distance travelled by the robots
  for (i = 0; i < ROBOTS; i++) {
    const double *pos = wb_supervisor_field_get_sf_vec3f(robTrans[i]);
    distance_travelled[i] += sqrt(pow(previous_x[i]-pos[0],2) + pow(previous_y[i] - pos[2], 2));
    previous_x[i] = pos[0];
    previous_y[i] = pos[2];
  }


  // Check if iteration over
  if (events_handled == EVENTS_TO_HANDLE) {

    long long int duration = (clock - temp_clock) / 1000; // Duration in seconds

    // Events per unit of time
    double metric1 = ((double) events_handled)/((double)duration); // Compute performance

    // Total distance travelled per unit of time
    double total_distance = 0;
    for (i = 0; i < ROBOTS; i++) {
      total_distance += distance_travelled[i];
    }
    double metric2 = total_distance / ((double) duration);

    iterations++;

    mean_perf += metric1/MAX_DURATION;

    printf("Robots completed %d tasks in %llis, performance = %f \n", EVENTS_TO_HANDLE, duration, metric1);
    printf("Robots travelled %f meters in %llis, performance = %f \n", total_distance, duration, metric2);
    sleep(1);
    events_handled = 0;
    temp_clock = clock;
    if (iterations <= MAX_DURATION) reset();
  }

  // Stop simulating after a certain number of iterations
  if (iterations > MAX_DURATION) {
    printf("Simulation terminated in %llis. Mean performance = %f\n", clock/1000, mean_perf);
    sleep(5);
    wb_supervisor_simulation_revert();
  }

  /* Get data */
  for (i=0;i<ROBOTS;i++) {
        /* Test if a robot has bounced in an obstacle */
    for (j=0;j<MAX_EVENTS;j++){
       bool collision = are_colliding(robTrans[i], events[j].eventTrans);
       /* Update the number of events and remove/replace the handled event*/
       if(collision){
         events[j].state -= EVENT_PRODUCTIVITY_STEP;
         if (events[j].state<=0) {
           // reset event
           events[j].state = 1.0;
           randomize_event_position(j);
           randomize_event_color(j);
           events_handled++;
           printf("%d events handled \n", events_handled);
          }
       }
    }
  }

  clock+=STEP_SIZE;
  return STEP_SIZE;
}


// main loop
int main(void)
{
  srand(time(NULL));
  // initialization
  wb_robot_init();

  reset();
  wb_robot_step(2*STEP_SIZE);

  // start the controller
  printf("Starting main loop...\n");
  while (wb_robot_step(STEP_SIZE) != -1)
  {
    run(STEP_SIZE);
  }

  wb_robot_cleanup();
  return 0;

}
