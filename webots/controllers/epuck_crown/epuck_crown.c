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

//#define DEBUG 1
#define TIME_STEP 64
#define FLOCK_SIZE 5


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/* e-Puck parameters */
#define NB_SENSORS	8
#define BIAS_SPEED	300
#define WIDTH		52   //pixel width of the camera
#define HEIGHT		39   //pixel height of the camera
#define MAXSPEED		1000

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/* Problem Description */

// PROBLEM TYPE
#define DETERMINISTIC	0   // 0= infinite steepness 1= above fitness used
#define ADAPTIVE		1   // 0=fixed, 1=adaptive thresholds
#define PUBLIC		0   // 0=local estimation (no information sharing),
                               // 1= global dissemination (collaboration with neighbors)

// THRESHOLD BASED ALGORITHM PARAMETERS
#define THRESHOLD		10   // value of homogeneous threshold
#define STEEPNESS		30  // steepness of threshold cutoff
#define ABANDON		0   // probability of giving up task (unused)
#define LOST_THRESHOLD	3   // number of pixels before a target color is considered lost (used to change FSM state)
#define THRESHOLD_DELTA	0.01   // TODO valeur arbitraire

// COLORS
#define NB_COLORS		3   // Number of colors
#define COLOR_BLIND	1   // 1=colors are ignored  0=colors are considered as different tasks
//#define NO_COLOR               -1 // Nothing special detected on screen (robot in front / gone through cylinder / wall)
typedef enum {NO_COLOR=-1, RED, GREEN, BLUE} color;

// TASKS
#define PERFORM_THRESHOLD  48  // Number of pixels to consider the robot close enough to the cylinder
#define STEPS_IDLE         120 // Number of steps while the robot stops to perform a task

typedef enum {SEARCH, GOTO_TASK, STOP_MOVE} fsm_state;

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
fsm_state state = SEARCH; // state of FSM: either the robot is in state "searching" or state "going towards a color"
fsm_state previous_state = SEARCH;
int steps = 0; // Number of steps to stay in perform state
// Colors & Thresholds
int chosen_color = NO_COLOR; // color chosen by the robot
int pos_color[NB_COLORS] = {0, 0, 0}; // average position of the chosen cylinder (camera)
int max_size_color[NB_COLORS]; // size of largest colored cylinders for each color (camera)
float threshold[NB_COLORS] = {THRESHOLD, THRESHOLD, THRESHOLD}; // stores the thresholds corresponding to the colors
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

// Functions
void setSpeed(int, int);
void adaptThresholds(void);
void receive_local_emission(void);
void send_local_emission(void);

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

int checkThreshold(double RAND, color c){
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

void adaptThresholds(void)
{
int i;
#ifdef ADAPTIVE
#ifdef COLOR_BLIND
if(ADAPTIVE == 1 && COLOR_BLIND == 1) // Adaptive + color blind = Distance adaptation
{
	if(state == SEARCH && previous_state == SEARCH)
	{
		for(i = 0 ; i < NB_COLORS ; i++){
			threshold[i] -= THRESHOLD_DELTA;
			if (threshold[i] < LOST_THRESHOLD) threshold[i] = LOST_THRESHOLD;
		}
	} else {
		if (state == SEARCH && previous_state == STOP_MOVE){
			for(i = 0 ; i < NB_COLORS ; i++){
				threshold[i] += 1; // TODO eventually increase more
			}
		}
	}

}

if(ADAPTIVE == 1 && COLOR_BLIND == 0) // Adaptive + not color blind = specialization
{
	if(state == STOP_MOVE && previous_state == GOTO_TASK && chosen_color != NO_COLOR) // if chosen_color = no_color, i.e. signal lost => do not specialize
	{
		threshold[chosen_color] -= 3*THRESHOLD_DELTA;
		for(i = 0 ; i < NB_COLORS ; i++){
			threshold[i] += THRESHOLD_DELTA;
		}
	}
}

#else // !COLOR_BLIND
#endif
#else
	return;
#endif
}

// Decision function
void updateRobot(){
	/* max_size_color est notre stimulus pour l'instant mais tu peux lui appliquer des
	   transformations ici et les stocker dans stimulus[NB_COLORS] ou utiliser le pixel_count.
	   On peut employer des boucles for mais on voulait que le code soit compréhensible dans un
	   premier temps. De toutes manières, le fonctionnement de la décision est
	   totalement débile pour l'instant. */

	// In any case (PUBLIC or not), the base stimulus is what we observe. In case of public, we modify it after.
	int c;
	for(c=0; c<NB_COLORS; c++){
		stimulus[c] = max_size_color[c]; // the stimulus is the sizes of the closest cylinders
	}

	if(ADAPTIVE==1){
		adaptThresholds(); // Update thresholds based on time spent in search mode or receiver
	}

	if(PUBLIC==1){
		int i;
		/* Ici coder la prise en compte des différentes émissions reçues */
		receive_local_emission();
		for(i = 0; i < NB_COLORS ; i++)
		{
			// We received the "normalized average stimulus" modified by senders distances.
			// We decrease our stimulus when this perception increases.
			stimulus[i] -= stimulus[i]*rgb_perception[i];
		}
		// Send our local datas.
		send_local_emission();
#ifdef DEBUG
		if(robot_id == 5)
		{
			printf("#%i max values {%i, %i, %i}\n", robot_id, max_size_color[0], max_size_color[1], max_size_color[2]);
			printf("#%i received {%f, %f, %f}\n", robot_id, rgb_perception[0], rgb_perception[1], rgb_perception[2]);
			printf("#%i send {%f, %f, %f}\n", robot_id, rgb_emission[0], rgb_emission[1], rgb_emission[2]);
			printf("#%i stimulus {%i, %i, %i}\n", robot_id, stimulus[0], stimulus[1], stimulus[2]);
#ifdef ADAPTIVE
			printf("#%i thresholds {%f, %f, %f}\n", robot_id, threshold[0], threshold[1], threshold[2]);
#endif

		}
#endif
	}
	previous_state = state;

         double rand; //declarations for switch statements.
	// if in search mode, try to pick a color according to probabilities
	switch (state) {
		case SEARCH :
			rand = rnd();
			if(checkThreshold(rand, RED)){
				chosen_color = RED;
			}
			else if(checkThreshold(rand, GREEN)){
				chosen_color = GREEN;
			}
			else if(checkThreshold(rand, BLUE)){
				chosen_color = BLUE;
			}
			else {
				chosen_color = NO_COLOR;
			}
			// if cylinder chosen, go chromataxis mode (GOTO_TASK)
			if(chosen_color != NO_COLOR){
				state = GOTO_TASK;
			}
			break;

			// if in chromataxis mode, check if cylinder was lost
		case GOTO_TASK :
			// Check if color is still in field of view
			if(chosen_color != NO_COLOR && stimulus[chosen_color] < LOST_THRESHOLD){
				chosen_color = NO_COLOR;
			}
			if (chosen_color == NO_COLOR)
			{
              			state = SEARCH;
			}

			// Slow down to perform action
			if(chosen_color != NO_COLOR && stimulus[chosen_color] > PERFORM_THRESHOLD){
				state = STOP_MOVE;
			}
			break;

			// if gone into STATE3, start idling
		case STOP_MOVE:
			if (steps < STEPS_IDLE){
				//wait
				steps++;
			}
			else {
				state = SEARCH;
				steps = 0;
			}
	}

}

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/* Image Processing */

void processImage(const unsigned char *image) {
	// Reset global variables
	int c;
	int pixel_count[NB_COLORS] = {0,0,0};
	for(c=0;c<NB_COLORS;c++){
		max_size_color[c] = 0;
		pos_color[c] = 0;
	}
	// Reset temporary variables
	int min_colors[3] = {WIDTH, WIDTH, WIDTH};
	int max_colors[3] = {0, 0, 0};
	int temp_size[3] = {0, 0, 0};
	int previous_color = NO_COLOR;

	// Analyze a row of pixels
	int x;
	int current_color;

	for(x = 0; x < WIDTH; x++) {
		int r = wb_camera_image_get_red(image, WIDTH, x, 18);
		int g = wb_camera_image_get_green(image, WIDTH, x, 18);
		int b = wb_camera_image_get_blue(image, WIDTH, x, 18);

		int scan[3] = {r, g, b};

		// Begin analysis
		for (current_color = 0; current_color < NB_COLORS; current_color++) {
			if(scan[current_color] >= 250) {
				pixel_count[current_color]++; // Update pixel count for current color

				if (previous_color == current_color){ // Good case, continue as usual
					temp_size[current_color]++;
					if (x > max_colors[current_color]) max_colors[current_color] = x;
					if (x < min_colors[current_color]) min_colors[current_color] = x;
				}
				else { // Change color
					// Start new cluster
					temp_size[current_color] = 1;
					max_colors[current_color] = x;
					min_colors[current_color] = x;
					previous_color = current_color;
				}
				// If the cluster is the biggest (yet), update size and mean position
				if (max_size_color[current_color] < temp_size[current_color]){
					max_size_color[current_color] = temp_size[current_color];
					pos_color[current_color] = (int) (max_colors[current_color] + min_colors[current_color])/2;
				}
			}
		}
		// If no color at all is found : reset color
		if (scan[RED] < 250 && scan[GREEN] < 250 && scan[BLUE] < 250) {
			previous_color = NO_COLOR;
		}
	}

}

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

#ifdef DEBUG
         printf("#%i chose to go %d\n", robot_id, chosen_color);
#endif

	mr = -10 * (pos_chosen_color-26);
	ml = 10 * (pos_chosen_color-26);

	obstacle_avoidance(&d1, &d2);

	msr = d1 + BIAS_SPEED + mr;
	msl = d2 + BIAS_SPEED + ml;

	setSpeed(msl,msr);
}

// Random Turn
void randomTurn(){
	msr = -200;
	msl = 200;

	setSpeed(msl,msr);
}

// Random Walk
void randomWalk(){
	int d1 = 0;   // motor speed from braitenberg (right)
	int d2 = 0;  // motor speed from braitenberg (left)

	obstacle_avoidance(&d1, &d2);

	msr = d1+BIAS_SPEED + 200*rnd(); msl = d2+BIAS_SPEED+ 200*rnd();
	msr -= BIAS_SPEED * rnd()-10; msl -= BIAS_SPEED *rnd();

	setSpeed(msl,msr);
}

// Slow motion
void slowMotion(){
  int d1 = 0;   // motor speed from braitenberg (right)
  int d2 = 0;  // motor speed from braitenberg (left)

  obstacle_avoidance(&d1, &d2);
  
  msr = d1 + BIAS_SPEED/3;
  msl = d2 + BIAS_SPEED/3;
  
  setSpeed(msl,msr);
}

void setSpeed(int LeftSpeed, int RightSpeed)
{
	if (LeftSpeed < -MAXSPEED) {LeftSpeed = -MAXSPEED;}
	if (LeftSpeed >  MAXSPEED) {LeftSpeed =  MAXSPEED;}
	if (RightSpeed < -MAXSPEED) {RightSpeed = -MAXSPEED;}
	if (RightSpeed >  MAXSPEED) {RightSpeed =  MAXSPEED;}

  wb_differential_wheels_set_speed(LeftSpeed,RightSpeed);
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
	for (i=0; i<3; i++)
		rgb_emission[i] = (float)max_size_color[i] / WIDTH; // We send our normalized stimulus for each color.

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
	while ((wb_receiver_get_queue_length(receiver) > 0) && (count<FLOCK_SIZE)) {

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

#ifdef DEBUG
		printf("#%i : %d \n", robot_id, state);
#endif

         switch (state)
         {
           case SEARCH :
		randomTurn();
           break;
           case GOTO_TASK :
		chromataxis(pos_color[chosen_color]);
           break;
           case STOP_MOVE :
		slowMotion(); // slow approach (may be optional)
           break;
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
