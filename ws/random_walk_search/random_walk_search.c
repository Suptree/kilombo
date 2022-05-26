
#include <math.h>

#include <kilombo.h>

#include "random_walk_search.h"
enum
{
  X,
  Y
};
enum
{
  FALSE,
  TRUE
};
enum
{
  STOP,
  LEFT,
  RIGHT,
  STRAIGHT
};

typedef struct
{
  Neighbor_t neighbors[MAXN];

  int N_Neighbors;
  uint8_t bot_type;          //{NEST, FOOD, NODE, EXPLORER}
  uint8_t move_type;         //{STOP, FORWARD, LEFT, RIGHT}
  double body_angle;         //体の向き use EXPLORER bot
  double pos[2];             // rベクトル use EXPLORER bot
  uint8_t walk_type;
  double food_pos[2];
  uint8_t path_integration;
  uint32_t return_time;

  message_t transmit_msg;
  char message_lock;

  received_message_t RXBuffer[RB_SIZE];
  uint8_t RXHead, RXTail;

} MyUserdata;

REGISTER_USERDATA(MyUserdata)

#ifdef SIMULATOR
#include <stdio.h> // for printf
#else
#define DEBUG // for printf to serial port
#include "debug.h"
#endif

uint8_t colorNum[] = {
    RGB(0, 0, 0), // 0 - off
    RGB(3, 0, 0), // 1 - red
    RGB(0, 1, 0), // 2 - green
    RGB(0, 0, 1), // 3 - blue
    RGB(1, 1, 0), // 4 - yellow
    RGB(0, 3, 3), // 5 - cyan
    RGB(1, 0, 1), // 6 - purple
    RGB(2, 1, 0), // 7  - orange
    RGB(1, 1, 1), // 8  - white
    RGB(3, 3, 3)  // 9  - bright white
};

// message rx callback function. Pushes message to ring buffer.
void rxbuffer_push(message_t *msg, distance_measurement_t *dist)
{
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
void set_bot_type(int type)
{
  mydata->bot_type = type;
}
int get_bot_type(void)
{
  return mydata->bot_type;
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
    { // found it
      break;
    }

  if (i == mydata->N_Neighbors)
  {                                     // this neighbor is not in list
    if (mydata->N_Neighbors < MAXN - 1) // if we have too many neighbors,
      mydata->N_Neighbors++;            // we overwrite the last entry
                                        // sloppy but better than overflow
  }

  // i now points to where this message should be stored
  mydata->neighbors[i].ID = ID;
  mydata->neighbors[i].timestamp = kilo_ticks;
  mydata->neighbors[i].dist = d;
  mydata->neighbors[i].N_Neighbors = data[2];
  mydata->neighbors[i].n_bot_type = data[4];
  mydata->neighbors[i].food_pos[X] = data[5];
  mydata->neighbors[i].food_pos[Y] = data[6];

}

/* Go through the list of neighbors, remove entries older than a threshold,
 * currently 2 seconds.
 */
void purgeNeighbors(void)
{
  int8_t i;

  for (i = mydata->N_Neighbors - 1; i >= 0; i--)
    if (kilo_ticks - mydata->neighbors[i].timestamp > 64) // 32 ticks = 1 s
    {                                                     // this one is too old.
      mydata->neighbors[i] = mydata->neighbors[mydata->N_Neighbors - 1];
      // replace it by the last entry
      mydata->N_Neighbors--;
    }
}

void setup_message(void)
{
  mydata->message_lock = 1; // don't transmit while we are forming the message
  mydata->transmit_msg.type = NORMAL;
  mydata->transmit_msg.data[0] = kilo_uid & 0xff;     // 0 low  ID
  mydata->transmit_msg.data[1] = kilo_uid >> 8;       // 1 high ID
  mydata->transmit_msg.data[2] = mydata->N_Neighbors; // 2 number of neighbors
  mydata->transmit_msg.data[4] = mydata->bot_type;
  mydata->transmit_msg.data[5] = mydata->food_pos[X];
  mydata->transmit_msg.data[6] = mydata->food_pos[Y];
  mydata->transmit_msg.crc = message_crc(&mydata->transmit_msg);
  mydata->message_lock = 0;
}
double angle_trim(double a)
{
  if (a < 0)
    a = 360.0 + a;
  if (a > 360)
    a = a - 360.0;

  return a;
}

//////////////////////////////////////   SETUP   ///////////////////////////////////////////
void setup()
{
  rand_seed(kilo_uid + 1); // seed the random number generator
  if (kilo_uid == 0)       // NEST bot
  {
    set_bot_type(NEST);
    set_move_type(STOP);
    set_color(colorNum[0]); // black
  }
  else if (kilo_uid == 1) // NODE bot
  {

    set_bot_type(FOOD);
    set_move_type(STOP);
    set_color(colorNum[5]); // cyan

  }
  // else if(kilo_uid <= 5){
  //   set_bot_type(NODE);
  //   set_move_type(STOP);
  //   set_color(colorNum[8]); // white
  // }
  else // EXPLORER bot
  {
    set_bot_type(EXPLORER);
    set_move_type(STOP);
    set_color(colorNum[1]);
    mydata->body_angle = 90.0;
    
    mydata->pos[X] = (kilo_uid - 1) * 60.0;
    mydata->pos[Y] = 0.0;
  }
  mydata->message_lock = 0;
  mydata->N_Neighbors = 0;
  mydata->food_pos[X] = 0.0;
  mydata->food_pos[Y] = 0.0;
  mydata->return_time = 0;
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

  for (i = 0; i < mydata->N_Neighbors; i++)
  {
    if (mydata->neighbors[i].ID == bot)
    {
      dist = mydata->neighbors[i].dist;
      break;
    }
  }
  return dist;
}
double get_food_pos_x()
{
  uint8_t i;
  double pos_x = 0.0;
  for (i = 0; i < mydata->N_Neighbors; i++)
  {
    if (mydata->neighbors[i].food_pos[X] != 0.0)
    {
      pos_x = mydata->neighbors[i].food_pos[X];
      break;
    }
  }
  return pos_x;
}

double get_food_pos_y()
{
  uint8_t i;
  double pos_y = 0.0;
  for (i = 0; i < mydata->N_Neighbors; i++)
  {
    if (mydata->neighbors[i].food_pos[Y] != 0.0)
    {
      pos_y = mydata->neighbors[i].food_pos[Y];
      break;
    }
  }
  return pos_y;
}
uint8_t find_Explorer()
{
  uint8_t i;
  for (i = 0; i < mydata->N_Neighbors; i++)
  {
    if (mydata->neighbors[i].n_bot_type == EXPLORER && mydata->neighbors[i].ID > kilo_uid)
    {
      return 1;
    }
  }
  return 0;
}

uint8_t find_Nest()
{
  uint8_t i;
  for (i = 0; i < mydata->N_Neighbors; i++)
  {
    if (mydata->neighbors[i].n_bot_type == NEST)
    {
      return 1;
    }
  }
  return 0;
}
uint8_t find_Node()
{
  uint8_t i;
  for (i = 0; i < mydata->N_Neighbors; i++)
  {
    if (mydata->neighbors[i].n_bot_type == NODE)
    {
      return 1;
    }
  }
  return 0;
}

uint8_t find_Food()
{
  uint8_t i;
  for (i = 0; i < mydata->N_Neighbors; i++)
  {
    if (mydata->neighbors[i].n_bot_type == FOOD)
    {
      return 1;
    }
  }
  return 0;
}

uint8_t find_Only_Food()
{
  uint8_t i;
  uint8_t only = 0;
  for (i = 0; i < mydata->N_Neighbors; i++)
  {
    if (mydata->neighbors[i].n_bot_type == NODE){

      only = 0;
      return only;
    }
    if (mydata->neighbors[i].n_bot_type == FOOD)
    {
      only = 1;
    }
  }
  return only;
}
uint8_t find_Only_Nest()
{
  uint8_t i;
  uint8_t only = 0;
  for (i = 0; i < mydata->N_Neighbors; i++)
  {
    if (mydata->neighbors[i].n_bot_type == NODE){

      only = 0;
      return only;
    }
    if (mydata->neighbors[i].n_bot_type == NEST)
    {
      only = 1;
    }
  }
  return only;
}

uint8_t find_nearest_N_dist()
{
  uint8_t i;
  uint8_t dist = 90;

  for (i = 0; i < mydata->N_Neighbors; i++)
  {
    if (mydata->neighbors[i].dist < dist)
    {
      dist = mydata->neighbors[i].dist;
    }
  }
  return dist;
}
void follow_edge()
{
  uint8_t desired_dist = 40;
   if (find_nearest_N_dist() > desired_dist)
  {
    if (get_move_type() == LEFT)
      spinup_motors();
    set_motors(0, kilo_turn_right);
    set_move_type(RIGHT);
    mydata->body_angle = mydata->body_angle - ONE_STEP_ROTATE_ANGLE;
    if (mydata->body_angle < 0)
      mydata->body_angle = 360.0 + mydata->body_angle;
    if (mydata->body_angle > 360)
      mydata->body_angle = mydata->body_angle - 360.0;

    mydata->pos[X] = mydata->pos[X] + ONE_STEP_MOVE_DIST * cos(mydata->body_angle * M_PI / 180.0);
    mydata->pos[Y] = mydata->pos[Y] + ONE_STEP_MOVE_DIST * sin(mydata->body_angle * M_PI / 180.0);
  }

  // if(find_nearest_N_dist() < desired_dist)
  else
  {
    if (get_move_type() == RIGHT)
      spinup_motors();
    set_motors(kilo_turn_left, 0);
    set_move_type(LEFT);

    mydata->body_angle = mydata->body_angle + ONE_STEP_ROTATE_ANGLE;

    if (mydata->body_angle < 0)
      mydata->body_angle = 360.0 + mydata->body_angle;
    if (mydata->body_angle > 360)
      mydata->body_angle = mydata->body_angle - 360.0;
    mydata->pos[X] = mydata->pos[X] + ONE_STEP_MOVE_DIST * cos(mydata->body_angle * M_PI / 180.0);
    mydata->pos[Y] = mydata->pos[Y] + ONE_STEP_MOVE_DIST * sin(mydata->body_angle * M_PI / 180.0);
  }
}
void go_straight()
{
  if(get_move_type() == LEFT) {
    if (get_move_type() == LEFT)
      spinup_motors();
    set_motors(0, kilo_turn_right);
    set_move_type(RIGHT);
    mydata->body_angle = mydata->body_angle - ONE_STEP_ROTATE_ANGLE;
    if (mydata->body_angle < 0)
      mydata->body_angle = 360.0 + mydata->body_angle;
    if (mydata->body_angle > 360)
      mydata->body_angle = mydata->body_angle - 360.0;

    mydata->pos[X] = mydata->pos[X] + ONE_STEP_MOVE_DIST * cos(mydata->body_angle * M_PI / 180.0);
    mydata->pos[Y] = mydata->pos[Y] + ONE_STEP_MOVE_DIST * sin(mydata->body_angle * M_PI / 180.0);


  }else{
    if (get_move_type() == RIGHT)
      spinup_motors();
    set_motors(kilo_turn_left, 0);
    set_move_type(LEFT);

    mydata->body_angle = mydata->body_angle + ONE_STEP_ROTATE_ANGLE;

    if (mydata->body_angle < 0)
      mydata->body_angle = 360.0 + mydata->body_angle;
    if (mydata->body_angle > 360)
      mydata->body_angle = mydata->body_angle - 360.0;
    mydata->pos[X] = mydata->pos[X] + ONE_STEP_MOVE_DIST * cos(mydata->body_angle * M_PI / 180.0);
    mydata->pos[Y] = mydata->pos[Y] + ONE_STEP_MOVE_DIST * sin(mydata->body_angle * M_PI / 180.0);

  }


}

void random_walk(){
  // srand((unsigned int)time(NULL)+kilo_ticks+kilo_uid);
  srand(kilo_ticks+kilo_uid);
  if(kilo_ticks % 50 == 0)
  mydata->walk_type= rand()%3 +1;
  if(mydata->walk_type == LEFT) {
    if (get_move_type() == LEFT)
      spinup_motors();
    set_motors(0, kilo_turn_right);
    set_move_type(RIGHT);
    mydata->body_angle = mydata->body_angle - ONE_STEP_ROTATE_ANGLE;
    if (mydata->body_angle < 0)
      mydata->body_angle = 360.0 + mydata->body_angle;
    if (mydata->body_angle > 360)
      mydata->body_angle = mydata->body_angle - 360.0;

    mydata->pos[X] = mydata->pos[X] + ONE_STEP_MOVE_DIST * cos(mydata->body_angle * M_PI / 180.0);
    mydata->pos[Y] = mydata->pos[Y] + ONE_STEP_MOVE_DIST * sin(mydata->body_angle * M_PI / 180.0);


  }else if(mydata->walk_type == RIGHT ){
    if (get_move_type() == RIGHT)
      spinup_motors();
    set_motors(kilo_turn_left, 0);
    set_move_type(LEFT);

    mydata->body_angle = mydata->body_angle + ONE_STEP_ROTATE_ANGLE;

    if (mydata->body_angle < 0)
      mydata->body_angle = 360.0 + mydata->body_angle;
    if (mydata->body_angle > 360)
      mydata->body_angle = mydata->body_angle - 360.0;
    mydata->pos[X] = mydata->pos[X] + ONE_STEP_MOVE_DIST * cos(mydata->body_angle * M_PI / 180.0);
    mydata->pos[Y] = mydata->pos[Y] + ONE_STEP_MOVE_DIST * sin(mydata->body_angle * M_PI / 180.0);

  }else{
    go_straight();
  }
}
void stop_straight()
{
  set_motors(0, 0);
}

void bhv_nest(){

  mydata->food_pos[X] = get_food_pos_x();
  mydata->food_pos[Y] = get_food_pos_y();

  if(mydata->food_pos[X] != 0.0){
    set_color(RGB(3,3,0));//yellow
  }
}

void bhv_explorer()
{
  // printf("==========\n");
 
  set_color(colorNum[1]);
  if(find_Explorer() == TRUE){
    stop_straight();
    return;
  }
  // printf("(x, y) = (%f, %f)\n", mydata->pos[X],mydata->pos[Y]);
  double r = atan2(mydata->pos[Y], mydata->pos[X]);

  if (r < 0)
  {
    r = r + 2.0 * M_PI;
  }

  r = r * 360.0 / (2.0 * M_PI);
//  printf("theta = %f\n", r);


  double angle_acos = acos(mydata->pos[X] / sqrt(pow(mydata->pos[X], 2) + pow(mydata->pos[Y], 2)) * sqrt(pow(1.0, 2) + pow(0.0, 2))) * 180.0 / M_PI;
  if (mydata->pos[Y] < 0)
    angle_acos = 360.0 - angle_acos;
  double vec_theta = fabs(angle_trim(180 + angle_acos));
  // printf("vec_theta = %f\n", vec_theta);

  // printf("vector : %f\n", vector);
  // printf("mydata->body_angle : %f\n", mydata->body_angle);


  if(mydata->path_integration == TRUE){
    go_straight();
    if(find_Nest() == TRUE || find_Node() == TRUE){
      mydata->path_integration = FALSE;
      mydata->return_time = 0;
      if(mydata->food_pos[X] != 0.0){
        set_bot_type(NODE);
        stop_straight();
      }
    }
    
    // printf("========= PIPIPI=========\n");
    return;
  }else if(find_Food() == TRUE){
    mydata->food_pos[X] = mydata->pos[X];
    mydata->food_pos[Y] = mydata->pos[Y];

    if (fabs(angle_trim(180 + angle_acos) - mydata->body_angle) < 0.5 && kilo_ticks > 100)
    {
      mydata->path_integration = TRUE;
    }       

    follow_edge();
    return;
  // }else if(mydata->food_pos[X] != 0 && (find_Food() == TRUE || find_Node() == TRUE)){
  //   double food_theta = atan2(mydata->food_pos[Y], mydata->food_pos[X]);

  //   if (food_theta < 0)
  //   {
  //     food_theta = food_theta + 2.0 * M_PI;
  //   }

  //   food_theta = food_theta * 360.0 / (2.0 * M_PI);

  //   if(fabs(food_theta - mydata->body_angle) < 0.5){
  //     go_straight();
  //   }else{
  //     follow_edge();
  //   }
  }
  else if(find_Nest() == TRUE || find_Node() == TRUE){
    mydata->food_pos[X] = get_food_pos_x();
    mydata->food_pos[Y] = get_food_pos_y();
    follow_edge();
  }else{
    mydata->return_time++;  
    if(mydata->return_time > 5000 &&fabs(angle_trim(180 + angle_acos) - mydata->body_angle) < 0.5){
      mydata->path_integration = TRUE;
    }
    random_walk();
    
  }

  // if(find_Food()){
    // follow_edge();
  // }else{
  // }

}
void loop()
{
  // receive messages
  receive_inputs();
  if (get_bot_type() == NEST)
  {
    bhv_nest();
  }
  else if (get_bot_type() == NODE)
  {
    set_move_type(STOP);
    set_color(colorNum[9]); // white

  }
  else if (get_bot_type() == FOOD)
  {
  }
  else if (get_bot_type() == EXPLORER)
  {
    bhv_explorer();
  }
  setup_message();
}

extern char *(*callback_botinfo)(void);
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

  RB_init(); // initialize ring buffer
  kilo_message_rx = rxbuffer_push;
  kilo_message_tx = message_tx; // register our transmission function

  kilo_start(setup, loop);

  return 0;
}

#ifdef SIMULATOR
// provide a text string for the status bar, about this bot
static char botinfo_buffer[15000];
char *botinfo(void)
{
  int n;
  char *p = botinfo_buffer;
  n = sprintf(p, "ID: %d, Food pos : (%f, %f)\n", kilo_uid, mydata->food_pos[X], mydata->food_pos[Y]);
  p += n;


  return botinfo_buffer;
}
#endif