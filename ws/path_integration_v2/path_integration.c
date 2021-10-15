
#include <math.h>

#include <kilombo.h>

#include "path_integration.h"
enum
{
  X,
  Y
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
  uint8_t bot_type;              //{NEST, FOOD, NODE, EXPLORER}
  uint8_t move_type;             //{STOP, FORWARD, LEFT, RIGHT}
  uint8_t gradient;              //勾配 use NODE bot
  uint8_t max_gradient;          //最大勾配 use NODE bot
  Harfway_bot_t halfway_bot;     // 中間地点の角度 use EXPLORER bot
  double body_angle;             //体の向き use EXPLORER bot
  double vec[2];                 // rベクトル use EXPLORER bot
  uint8_t turn_around_mode_flag; // 反対周りにするかどうか use EXPLOERE bot
  uint8_t is_tail;               //末尾かどうか use NODE bot
  int around_tick;
  uint8_t is_past_food;
  uint8_t straight_flag;

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
  mydata->neighbors[i].n_gradient = data[3];
  mydata->neighbors[i].n_bot_type = data[4];
  mydata->neighbors[i].max_gradient = data[5];
  mydata->neighbors[i].n_is_tail = data[6];
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
  mydata->transmit_msg.data[3] = mydata->gradient;
  mydata->transmit_msg.data[4] = mydata->bot_type;
  mydata->transmit_msg.data[5] = mydata->max_gradient;
  mydata->transmit_msg.data[6] = mydata->is_tail;

  mydata->transmit_msg.crc = message_crc(&mydata->transmit_msg);
  mydata->message_lock = 0;
}
uint8_t get_neibors_max_gradient()
{
  uint8_t i;
  uint8_t max_gradient = 0;
  for (i = 0; i < mydata->N_Neighbors; i++)
  {
    if (mydata->neighbors[i].n_gradient > mydata->gradient)
    {
      max_gradient = mydata->neighbors[i].n_gradient;
    }
  }
  return max_gradient;
}
void update_gradient()
{
  uint8_t i;
  uint8_t min_gradient = UINT8_MAX;

  for (i = 0; i < mydata->N_Neighbors; i++)
  {
    // if (mydata->neighbors[i].n_gradient == UINT8_MAX)
    //   continue;
    if (mydata->neighbors[i].n_bot_type == EXPLORER)
      continue;
    if (mydata->neighbors[i].n_bot_type == FOOD)
      continue;
    if (mydata->neighbors[i].n_gradient < min_gradient)
    {
      min_gradient = mydata->neighbors[i].n_gradient;
    }
  }
  if (min_gradient == UINT8_MAX)
    return;

  mydata->gradient = min_gradient + 1;

  return;
}
void update_max_gradient()
{
  uint8_t i;
  uint8_t max_gradient = 0;

  for (i = 0; i < mydata->N_Neighbors; i++)
  {
    if (mydata->neighbors[i].max_gradient == UINT8_MAX)
      continue;
    if (mydata->neighbors[i].n_bot_type == NEST)
      continue;
    if (mydata->neighbors[i].n_bot_type == EXPLORER)
      continue;
    if (mydata->neighbors[i].n_bot_type == FOOD)
      continue;
    if (mydata->neighbors[i].n_gradient > max_gradient)
    {
      max_gradient = mydata->neighbors[i].n_gradient;
    }
    if (mydata->gradient > max_gradient)
    {
      max_gradient = mydata->gradient;
    }
    if (mydata->neighbors[i].max_gradient > max_gradient)
    {
      max_gradient = mydata->neighbors[i].max_gradient;
    }
  }
  if (max_gradient == 20)
    return;

  mydata->max_gradient = max_gradient;

  return;
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
    mydata->gradient = 0;
    mydata->max_gradient = 0;
  }
  else if (kilo_uid >= 1 && kilo_uid <= 18) // NODE bot
  {
    set_bot_type(NODE);
    set_move_type(STOP);
    set_color(colorNum[9]); // white
    mydata->gradient = UINT8_MAX;
    mydata->max_gradient = 0;
    mydata->is_tail = 0;
  }
  else if (kilo_uid == 19) // FOOD bot
  {
    set_bot_type(FOOD);
    set_move_type(STOP);
    set_color(colorNum[5]); // cyan
    mydata->gradient = UINT8_MAX;
    mydata->max_gradient = 0;
  }
  else // EXPLORER bot
  {
    set_bot_type(EXPLORER);
    set_move_type(STOP);
    set_color(colorNum[1]);
    mydata->gradient = UINT8_MAX;
    mydata->max_gradient = 0;
    mydata->body_angle = 90.0;
    mydata->vec[X] = 0.0;
    mydata->vec[Y] = 0.0;
    mydata->turn_around_mode_flag = 0;
  }
  mydata->message_lock = 0;

  mydata->N_Neighbors = 0;

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
uint8_t find_Tail()
{
  uint8_t i;
  for (i = 0; i < mydata->N_Neighbors; i++)
  {
    if (mydata->neighbors[i].n_is_tail == 1)
    {
      return 1;
    }
  }
  return 0;
}
uint8_t Past_Tail()
{
  uint8_t i;
  for (i = 0; i < mydata->N_Neighbors; i++)
  {
    if (mydata->neighbors[i].n_bot_type == NEW_NODE)
      continue;
    if (mydata->neighbors[i].n_is_tail == 1)
    {
      return 1;
    }
  }
  return 0;
}

uint8_t find_NewNode()
{
  uint8_t i;
  for (i = 0; i < mydata->N_Neighbors; i++)
  {
    if (mydata->neighbors[i].n_bot_type == NEW_NODE)
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
  for (i = 0; i < mydata->N_Neighbors; i++)
  {
    if (mydata->neighbors[i].n_bot_type == NODE)
      return 0;
    if (mydata->neighbors[i].n_bot_type == FOOD)
    {
      return 1;
    }
  }
  return 0;
}

uint8_t find_nearest_N_dist()
{
  uint8_t i;
  uint8_t dist = 90;

  for (i = 0; i < mydata->N_Neighbors; i++)
  {
    // if (mydata->neighbors[i].n_bot_type == EXPLORER)
    //   continue;
    if (mydata->neighbors[i].dist < dist)
    {
      dist = mydata->neighbors[i].dist;
    }
  }
  return dist;
}
void update_harfway_info()
{
  uint8_t i;

  for (i = 0; i < mydata->N_Neighbors; i++)
  {
    if (mydata->neighbors[i].n_bot_type != NODE)
      continue;
    if (mydata->neighbors[i].max_gradient / 2 == mydata->neighbors[i].n_gradient)
    {
      double halfway_acos = acos(mydata->vec[X] / sqrt(pow(mydata->vec[X], 2) + pow(mydata->vec[Y], 2)) * sqrt(pow(1.0, 2) + pow(0.0, 2))) * 180.0 / M_PI;
      if (mydata->vec[Y] < 0)
        halfway_acos = 360.0 - halfway_acos;
      mydata->halfway_bot.angle = halfway_acos;
      mydata->halfway_bot.pos[X] = mydata->vec[X];
      mydata->halfway_bot.pos[Y] = mydata->vec[Y];
    }
  }
  return;
}
void follow_edge()
{
  uint8_t desired_dist = 55;
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

    mydata->vec[X] = mydata->vec[X] + ONE_STEP_MOVE_DIST * cos(mydata->body_angle * M_PI / 180.0);
    mydata->vec[Y] = mydata->vec[Y] + ONE_STEP_MOVE_DIST * sin(mydata->body_angle * M_PI / 180.0);
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
    mydata->vec[X] = mydata->vec[X] + ONE_STEP_MOVE_DIST * cos(mydata->body_angle * M_PI / 180.0);
    mydata->vec[Y] = mydata->vec[Y] + ONE_STEP_MOVE_DIST * sin(mydata->body_angle * M_PI / 180.0);
  }
}
uint8_t is_reverse()
{ // 0 : 通常, 1 : Reverse

  if (mydata->vec[Y] > 0)
  {
    if (mydata->halfway_bot.pos[Y] > 0)
    {
      if (mydata->halfway_bot.pos[X] < mydata->vec[X])
      {
        // printf("通常周り\n");
        // printf("[1]\n");
        return 0;
      }
      else
      {
        // printf("反対周り\n");
        // printf("[2]\n");
        return 1;
      }
    }
    else // if (mydata->halfway_bot.pos[Y] < 0)
    {
      if (mydata->halfway_bot.pos[X] < 0.0)
      {
        // printf("通常周り\n");
        // printf("[3]\n");
        return 0;
      }
      else
      {
        // printf("反対周り\n");
        // printf("[4]\n");
        return 1;
      }
    }
  }
  else //   if (mydata->vec[Y] < 0)
  {

    if (mydata->halfway_bot.pos[Y] > 0)
    {
      if (mydata->halfway_bot.pos[X] > 0.0)
      {
        // printf("通常周り\n");
        // printf("[5]\n");

        return 0;
      }
      else
      {
        // printf("反対周り\n");
        // printf("[6]\n");

        return 1;
      }
    }
    else // if (mydata->halfway_bot.pos[Y] < 0)
    {
      if (mydata->halfway_bot.pos[X] > mydata->vec[X])
      {
        // printf("通常周り\n");
        // printf("[7]\n");

        return 0;
      }
      else
      {
        // printf("反対周り\n");
        // printf("[8]\n");

        return 1;
      }
    }
  }
}
void rotate_move()
{
  mydata->body_angle = mydata->body_angle + ONE_STEP_ROTATE_ANGLE;

  if (mydata->body_angle < 0)
    mydata->body_angle = 360.0 + mydata->body_angle;
  if (mydata->body_angle > 360)
    mydata->body_angle = mydata->body_angle - 360.0;
  mydata->vec[X] = mydata->vec[X] + ONE_STEP_MOVE_DIST * cos(mydata->body_angle * M_PI / 180.0);
  mydata->vec[Y] = mydata->vec[Y] + ONE_STEP_MOVE_DIST * sin(mydata->body_angle * M_PI / 180.0);

  // printf("uくるくｒ\n");
  set_motors(kilo_turn_left, 0);
  set_move_type(LEFT);
}
void go_straight()
{
  set_motors(kilo_turn_left, kilo_turn_right);
}
uint8_t is_there_higher_gradient()
{

  uint8_t i;
  uint8_t flag = 0;
  for (i = 0; i < mydata->N_Neighbors; i++)
  {
    if (mydata->neighbors[i].n_bot_type == EXPLORER)
      continue;
    if (mydata->neighbors[i].n_bot_type == FOOD)
      continue;

    if (mydata->neighbors[i].n_gradient > mydata->gradient)
    {
      flag = 1;
      return flag;
    }
  }
  return flag;
}

uint8_t turn_arround_mode()
{
  if (find_Tail() == 1 && kilo_ticks > (kilo_uid - 20 + 1) * 15000)
  {
    double angle_acos = acos(mydata->vec[X] / sqrt(pow(mydata->vec[X], 2) + pow(mydata->vec[Y], 2)) * sqrt(pow(1.0, 2) + pow(0.0, 2))) * 180.0 / M_PI;

    if (angle_acos < mydata->halfway_bot.angle)
    {
      return 1;
    }
  }
  return 0;
}
void update_detect_food()
{
  // if (find_Food())
  // {
  //   mydata->is_past_food = 1;
  // }
  if (find_Only_Food())
  {
    mydata->is_past_food = 1;
  }
}
uint8_t past_Food()
{
  if (mydata->is_past_food == 1)
  {
    return 1;
  }
  else
  {
    return 0;
  }
}

void update_self_info()
{
}

void bhv_explorer()
{
  update_harfway_info();
  update_detect_food();

  double angle_acos = acos(mydata->vec[X] / sqrt(pow(mydata->vec[X], 2) + pow(mydata->vec[Y], 2)) * sqrt(pow(1.0, 2) + pow(0.0, 2))) * 180.0 / M_PI;
  if (mydata->vec[Y] < 0)
    angle_acos = 360.0 - angle_acos;

  // printf("angle_tes : %f, angle_acos : %f, angle_trim : %f,  harfway_angle : %f, harfway_pos : (%f,%f)\n", angle_tes, angle_acos, angle_trim(180 + angle_acos), mydata->halfway_bot.angle, mydata->halfway_bot.pos[X], mydata->halfway_bot.pos[Y]);

  if (past_Food())
  {
    if (fabs(angle_trim(180 + angle_acos) - mydata->body_angle) < 1.0 && kilo_ticks > (kilo_uid - 20 + 1) * 5000)
    {
      go_straight();
      if ((find_Nest() || find_NewNode()))
        set_bot_type(NEW_NODE);
    }
    else if (is_reverse())
    {
      rotate_move();
    }
    else
    {
      follow_edge();
    }
  }
  else
  {
    follow_edge();
  }
}
void loop()
{
  // receive messages
  receive_inputs();
  if (get_bot_type() == NEST)
  {
  }
  else if (get_bot_type() == NODE || get_bot_type() == NEW_NODE)
  {
    set_move_type(STOP);
    set_color(colorNum[9]); // white
    mydata->gradient = UINT8_MAX;
    mydata->max_gradient = 0;
    set_motors(0, 0);

    update_gradient();
    update_max_gradient();
    if (is_there_higher_gradient() == 1)
    {
      mydata->is_tail = 0;
    }
    else
    {
      mydata->is_tail = 1;
    }
  }
  else if (get_bot_type() == FOOD)
  {
  }
  else if (get_bot_type() == EXPLORER)
  {
    if ((kilo_uid - 20) * 15000 < kilo_ticks)
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
  n = sprintf(p, "ID: %d ", kilo_uid);
  p += n;

  n = sprintf(p, "Ns: %d, dist: %d, Gradient : %d, max_gradient : %d, is_tail : %d\n ", mydata->N_Neighbors, find_nearest_N_dist(), mydata->gradient, mydata->max_gradient, mydata->is_tail);
  p += n;

  return botinfo_buffer;
}
#endif
