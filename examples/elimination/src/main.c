/* Kilobot Edge following demo
 * 
 * Ivica Slavkov, Fredrik Jansson  2015
 */

#include <math.h>

#include <kilombo.h>

#include "edge.h"
#include "nest.h"
#include "food.h"
#include "robot.h"


typedef struct
{
  Neighbor_t neighbors[MAXN];

  int N_Neighbors;
  uint8_t bot_type;
  uint8_t bot_state;
  uint8_t move_type;
  uint8_t bhv_state;
  uint8_t dist_state;
  uint8_t selfgradient;
  uint8_t maxgradient;
  uint8_t selfchainidmaxgradient;

  message_t transmit_msg;
  char message_lock;

  received_message_t RXBuffer[RB_SIZE];
  uint8_t RXHead, RXTail;

} MyUserdata;

REGISTER_USERDATA(MyUserdata)
// declare constants
static const uint8_t TOOCLOSE_DISTANCE = 30; // 40 mm
static const uint8_t DESIRED_DISTANCE = 50; // 60 mm

#ifdef SIMULATOR
#include <stdio.h>    // for printf
#else
#define DEBUG         // for printf to serial port
#include "debug.h"
#endif
void smooth_set_motors(uint8_t ccw, uint8_t cw)
{
  // OCR2A = ccw;  OCR2B = cw;  
#ifdef KILOBOT 
  uint8_t l = 0, r = 0;
  if (ccw && !OCR2A) // we want left motor on, and it's off
    l = 0xff;
  if (cw && !OCR2B)  // we want right motor on, and it's off
    r = 0xff;
  if (l || r)        // at least one motor needs spin-up
    {
      set_motors(l, r);
      delay(15);
    }
#endif
  // spin-up is done, now we set the real value
  set_motors(ccw, cw);
}


uint8_t colorNum[] = {
  RGB(0,0,0),  //0 - off
  RGB(1,0,0),  //1 - red
  RGB(0,1,0),  //2 - green
  RGB(0,0,1),  //3 - blue
  RGB(1,1,0),  //4 - yellow
  RGB(0,1,1),  //5 - cyan
  RGB(1,0,1),  //6 - purple
  RGB(2,1,0),  //7  - orange
  RGB(1,1,1),  //8  - white
  RGB(3,3,3)   //9  - bright white
};

// message rx callback function. Pushes message to ring buffer.
void rxbuffer_push(message_t *msg, distance_measurement_t *dist) {
    received_message_t *rmsg = &RB_back();
    rmsg->msg = *msg;
    rmsg->dist = *dist;
    RB_pushback();
}

message_t *message_tx()
{
  if (mydata->message_lock)
    return 0;
  return &mydata->transmit_msg;
}

void set_bot_state(int state)
{
  mydata->bot_state = state;
}

void set_bhv_state(int state)
{
  mydata->bhv_state = state;
}

void set_dist_state(int state)
{
  mydata->dist_state = state;
}

int get_bot_state(void)
{
  return mydata->bot_state;
}
int get_bhv_state(void)
{
	return mydata->bhv_state;
}
int get_dist_state(void)
{
	return mydata->dist_state;
}

void set_move_type(int type)
{
  mydata->move_type = type;
}

int get_move_type(void)
{
  return mydata->move_type;
}

/* Process a received message at the front of the ring buffer.
 * Go through the list of neighbors. If the message is from a bot
 * already in the list, update the information, otherwise
 * add a new entry in the list
 */

void process_message()
{
  uint8_t i;
  uint16_t ID;

  uint8_t *data = RB_front().msg.data;
  ID = data[0] | (data[1] << 8);
  uint8_t d = estimate_distance(&RB_front().dist);

  // search the neighbor list by ID
  for (i = 0; i < mydata->N_Neighbors; i++)
    if (mydata->neighbors[i].ID == ID)
      {// found it
    	break;
      }

  if (i == mydata->N_Neighbors){  // this neighbor is not in list
    if (mydata->N_Neighbors < MAXN-1) // if we have too many neighbors,
      mydata->N_Neighbors++;          // we overwrite the last entry
                                      // sloppy but better than overflow
  }

  // i now points to where this message should be stored
  mydata->neighbors[i].ID = ID;
  mydata->neighbors[i].timestamp = kilo_ticks;
  mydata->neighbors[i].dist = d;
  mydata->neighbors[i].N_Neighbors = data[2];
  mydata->neighbors[i].n_bot_state = data[3];
  mydata->neighbors[i].bhv_state = data[4];
  mydata->neighbors[i].bhv_state = data[5];
}

/* Go through the list of neighbors, remove entries older than a threshold,
 * currently 2 seconds.
 */
void purgeNeighbors(void)
{
  int8_t i;

  for (i = mydata->N_Neighbors-1; i >= 0; i--)
    if (kilo_ticks - mydata->neighbors[i].timestamp  > 64) //32 ticks = 1 s
      { //this one is too old.
	mydata->neighbors[i] = mydata->neighbors[mydata->N_Neighbors-1];
	//replace it by the last entry
	mydata->N_Neighbors--;
      }
}

void setup_message(void)
{
  mydata->message_lock = 1;  //don't transmit while we are forming the message
  mydata->transmit_msg.type = NORMAL;
  mydata->transmit_msg.data[0] = kilo_uid & 0xff;     // 0 low  ID
  mydata->transmit_msg.data[1] = kilo_uid >> 8;       // 1 high ID
  mydata->transmit_msg.data[2] = mydata->N_Neighbors; // 2 number of neighbors
  mydata->transmit_msg.data[3] = get_bot_state();     // 3 bot state
  mydata->transmit_msg.data[4] = get_bhv_state();

  mydata->transmit_msg.crc = message_crc(&mydata->transmit_msg);
  mydata->message_lock = 0;
}

void setup()
{
  rand_seed(kilo_uid + 1); //seed the random number generator
  
  mydata->message_lock = 0;

  mydata->N_Neighbors = 0;
  set_move_type(STOP);
  set_bot_state(LISTEN);
  set_bhv_state(EXPLORER);
  setup_message();
}

void receive_inputs()
{
  while (!RB_empty())
    {
      process_message();
      RB_popfront();
    }
  purgeNeighbors();
}

uint8_t get_dist_by_ID(uint16_t bot)
{
  uint8_t i;
  uint8_t dist = 255;
  
  for(i = 0; i < mydata->N_Neighbors; i++)
    {
      if(mydata->neighbors[i].ID == bot)
	{
	  dist = mydata->neighbors[i].dist;
	  break;
	}
    }
  return dist;
}
int find_node(){

  uint8_t i;
  for(i = 0; i < mydata->N_Neighbors; i++)
    {
      if(mydata->neighbors[i].bhv_state == NODE) return 1;
    }
  return 0;
}
int find_nest() // 近くにnestがいたら1、いなかったら０を返す
{
  uint8_t i;

  for(i = 0; i < mydata->N_Neighbors; i++)
    {
      if(mydata->neighbors[i].ID == 0) return 1;
    }
  return 0;
}

int find_food()
{

  uint8_t i;
  for(i = 0; i < mydata->N_Neighbors; i++)
    {
      if(mydata->neighbors[i].ID == 1) return 1;
    }
  return 0;
}

uint8_t find_node_num()
{
	uint8_t i;
	uint8_t node_num = 0;
	for(i = 0; i < mydata->N_Neighbors; i++)
	{
		if(mydata->neighbors[i].bhv_state == NODE ) node_num++;
	}
	return node_num;
}
uint8_t find_nearest_N_dist()
{
  uint8_t i;
  uint8_t dist = 90;

  for(i = 0; i < mydata->N_Neighbors; i++)
    {
      if(mydata->neighbors[i].dist < dist)
	{
	  dist = mydata->neighbors[i].dist;
	}
    }
  return dist;
}
void orbit_normal() 
{
  if (find_nearest_N_dist() < TOOCLOSE_DISTANCE) {
        mydata->dist_state = TOOCLOSEDIST;
    } else{
        if (find_nearest_N_dist() < DESIRED_DISTANCE)
		{
            set_motors(kilo_turn_left, 0);
            set_move_type(LEFT);
		}else{

	        set_motors(0, kilo_turn_right);
  		    set_move_type(RIGHT);
		}
    }
}

void orbit_tooclose() {
  if (find_nearest_N_dist() >= DESIRED_DISTANCE)
    mydata->dist_state = NORMALDIST;
  else{
	  set_motors(kilo_turn_left,kilo_turn_right);
	  set_move_type(FORWARD);
  }
}
void follow_edge()
{
  uint8_t desired_dist = 55;
//   if(find_nearest_N_dist() > desired_dist)
//     {
//       if(get_move_type() == LEFT)
// 	spinup_motors();
//       set_motors(0, kilo_turn_right);
//       set_move_type(RIGHT);
//     }

//   //if(find_nearest_N_dist() < desired_dist)
//   else
//     {
//       if(get_move_type() == RIGHT)
// 	spinup_motors();
//       set_motors(kilo_turn_left, 0);
//       set_move_type(LEFT);
//     }
  if(mydata->dist_state == NORMALDIST){
	  orbit_normal();
	  return;
  }else{
	  orbit_tooclose();
	  return;
  } 
   
}
void robot_node_behavior(){
	if(mydata->bhv_state == DETECT){
		set_color(RGB(0,3,3));
	}else{

	set_color(RGB(3,3,3));
	}

	set_motors(0, 0);
	set_move_type(STOP);
}

void robot_explorer_behavior(){
	set_color(RGB(0,0,3));

	if(kilo_ticks % 1000 !=( kilo_uid * 50 ) ){
		return;
	}


	// if(kilo_ticks < 160){
	// 	if(find_nest()){
	// 		set_bhv_state(NODE);
	// 		return;
	// 	}	

	// }
	// set_color(RGB(0,0,3));
	// follow_edge();
	// // printf("time : %d\n",kilo_ticks);

	// if(find_node()){
	// 	set_bhv_state(NODE);
	// 	return;
	// }
	// if(kilo_uid == 5)
	// printf("kilo_tick : %d\nfind_node_num : %d\n", kilo_ticks,find_node_num());
	if(find_food()){
		set_bhv_state(DETECT);
		return;
	}

	if(  (!find_nest() && find_node_num() == 1) || (find_nest() && find_node_num() == 0))
	{
	// if(kilo_uid == 5)
	// printf("true\n");
		set_bhv_state(NODE);		
		return;	
	}
	follow_edge();

	

}


void loop()
{
  //receive messages
  receive_inputs();
  if(kilo_uid == 0) // nest
    {
      set_robot_nest_color();
	  robot_nest_behavior();
    }
  else if(kilo_uid == 1) // food
  {
	  set_robot_food_color();
	  robot_food_behavior();
  }
  else
  {
	  if(mydata->bhv_state == EXPLORER){
		robot_explorer_behavior();  
	  }else if(mydata->bhv_state == NODE || mydata->bhv_state == DETECT){
		  robot_node_behavior();
	  }else if(mydata->bhv_state == LOSTCHAIN){
		  
	  }else{
		  printf("ERROR");
	  }
  }
  setup_message();
}

extern char* (*callback_botinfo) (void);
char *botinfo(void);

int main(void)
{
  kilo_init();

#ifdef DEBUG
  // setup debugging, i.e. printf to serial port, in real Kilobot
  debug_init();
#endif

  SET_CALLBACK(botinfo, botinfo);
  SET_CALLBACK(reset, setup);

  RB_init();                       // initialize ring buffer
  kilo_message_rx = rxbuffer_push;
  kilo_message_tx = message_tx;    // register our transmission function

  kilo_start(setup, loop);

  return 0;
}

#ifdef SIMULATOR
// provide a text string for the status bar, about this bot
static char botinfo_buffer[10000];
char *botinfo(void)
{
  int n;
  char *p = botinfo_buffer;
  n = sprintf (p, "ID: %d ", kilo_uid);
  p += n;

  n = sprintf (p, "Ns: %d, dist: %d\n ", mydata->N_Neighbors, find_nearest_N_dist());
  p += n;

  return botinfo_buffer;
}
#endif
