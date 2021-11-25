
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
  uint8_t bot_type;          //{NEST, FOOD, NODE, EXPLORER}
  uint8_t move_type;         //{STOP, FORWARD, LEFT, RIGHT}
  uint8_t belong_type;       //{NEW, OLD, PI}
  uint8_t gradient;          //勾配 use NODE bot
  uint8_t max_gradient;      //最大勾配 use NODE bot
  Harfway_bot_t halfway_bot; // 中間地点の角度 use EXPLORER bot
  double body_angle;         //体の向き use EXPLORER bot
  double pos[2];             // rベクトル use EXPLORER bot
  uint8_t is_past_food;
  uint8_t past_food_count;
  double start_pos[2];
  double goal_pos[2];

  uint8_t is_detected_nest;
  uint8_t is_detected_half;
  uint8_t detected_nest_count;
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
  mydata->neighbors[i].n_belong_type = data[6];
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
  mydata->transmit_msg.data[6] = mydata->belong_type;

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
  // else if (kilo_uid >= 1 && kilo_uid <= 18) // NODE bot
  else if (kilo_uid >= 1 && kilo_uid <= 40) // NODE bot D=1000 increase robot
  {
    set_bot_type(NODE);
    set_move_type(STOP);
    set_color(colorNum[9]); // white
    mydata->gradient = UINT8_MAX;
    mydata->max_gradient = 0;
  }
  // else if (kilo_uid == 19) // FOOD bot
  else if (kilo_uid == 41) // FOOD bot D = 1000 increase robot
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
    mydata->pos[X] = 0.0;
    mydata->pos[Y] = 0.0;
    // if (kilo_uid == 20)
    // {
    //   mydata->pos[X] = 50.0;
    //   mydata->pos[Y] = -9.0;
    // }
    // else if (kilo_uid == 21)
    // {
    //   mydata->pos[X] = 0.0;
    //   mydata->pos[Y] = 70.0;
    // }
    // else if (kilo_uid == 22)
    // {
    //   mydata->pos[X] = 80.0;
    //   mydata->pos[Y] = 0.0;
    // }
    // else if (kilo_uid == 23)
    // {
    //   mydata->pos[X] = 80.0;
    //   mydata->pos[Y] = 10.0;
    // }
    // else if (kilo_uid == 24)
    // {
    //   mydata->pos[X] = 80.0;
    //   mydata->pos[Y] = 50.0;
    // }
    // else if (kilo_uid == 25)
    // {
    //   mydata->pos[X] = 80.0;
    //   mydata->pos[Y] = 0.0;
    // }
    // else if (kilo_uid == 26)
    // {
    //   mydata->pos[X] = 25.0;
    //   mydata->pos[Y] = 10.0;
    // }
    // else if (kilo_uid == 27)
    // {
    //   mydata->pos[X] = 10.0;
    //   mydata->pos[Y] = 10.0;
    // }
  }
  mydata->message_lock = 0;
  mydata->is_detected_nest = 0;
  mydata->N_Neighbors = 0;
  mydata->belong_type = OLD;
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
uint8_t is_there_explorer_with_higher_id()
{

  uint8_t i;
  for (i = 0; i < mydata->N_Neighbors; i++)
  {
    if (mydata->neighbors[i].n_bot_type != EXPLORER)
      continue;
    if (mydata->neighbors[i].n_belong_type == PI)
      continue;
    if (mydata->neighbors[i].ID > kilo_uid)
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
uint8_t find_Explorer()
{
  uint8_t i;
  for (i = 0; i < mydata->N_Neighbors; i++)
  {
    if (mydata->neighbors[i].n_bot_type == EXPLORER && mydata->neighbors[i].n_belong_type == PI && mydata->neighbors[i].ID > kilo_uid)
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
uint8_t find_Only_Nest()
{
  uint8_t i;
  for (i = 0; i < mydata->N_Neighbors; i++)
  {
    if (mydata->neighbors[i].n_bot_type == NODE)
      return 0;
    if (mydata->neighbors[i].n_bot_type == NEST)
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
      double halfway_acos = acos(mydata->pos[X] / sqrt(pow(mydata->pos[X], 2) + pow(mydata->pos[Y], 2)) * sqrt(pow(1.0, 2) + pow(0.0, 2))) * 180.0 / M_PI;
      if (mydata->pos[Y] < 0)
        halfway_acos = 360.0 - halfway_acos;
      mydata->halfway_bot.angle = halfway_acos;
      mydata->halfway_bot.pos[X] = mydata->pos[X];
      mydata->halfway_bot.pos[Y] = mydata->pos[Y];
      mydata->is_detected_half = 1;
      // printf("update half : %d\n", kilo_ticks);
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
uint8_t is_reverse()
{ // 0 : 通常, 1 : Reverse

  // if (mydata->pos[Y] > 0)
  // {
  //   if (mydata->halfway_bot.pos[Y] > 0)
  //   {
  //     if (mydata->halfway_bot.pos[X] < mydata->pos[X])
  //     {
  //       // printf("通常周り\n");
  //       // printf("[1]\n");
  //       return 0;
  //     }
  //     else
  //     {
  //       // printf("反対周り\n");
  //       // printf("[2]\n");
  //       return 1;
  //     }
  //   }
  //   else // if (mydata->halfway_bot.pos[Y] < 0)
  //   {
  //     if (mydata->halfway_bot.pos[X] < 0.0)
  //     {
  //       // printf("通常周り\n");
  //       // printf("[3]\n");
  //       return 0;
  //     }
  //     else
  //     {
  //       // printf("反対周り\n");
  //       // printf("[4]\n");
  //       return 1;
  //     }
  //   }
  // }
  // else //   if (mydata->pos[Y] < 0)
  // {

  //   if (mydata->halfway_bot.pos[Y] > 0)
  //   {
  //     if (mydata->halfway_bot.pos[X] > 0.0)
  //     {
  //       // printf("通常周り\n");
  //       // printf("[5]\n");

  //       return 0;
  //     }
  //     else
  //     {
  //       // printf("反対周り\n");
  //       // printf("[6]\n");

  //       return 1;
  //     }
  //   }
  //   else // if (mydata->halfway_bot.pos[Y] < 0)
  //   {
  //     if (mydata->halfway_bot.pos[X] > mydata->pos[X])
  //     {
  //       // printf("通常周り\n");
  //       // printf("[7]\n");

  //       return 0;
  //     }
  //     else
  //     {
  //       // printf("反対周り\n");
  //       // printf("[8]\n");

  //       return 1;
  //     }
  //   }
  // }
  //// Foodの角度計算やラジアン計算
  double food_rad = atan2(mydata->goal_pos[Y], mydata->goal_pos[X]);
  if (food_rad < 0)
  {
    food_rad = food_rad + 2 * M_PI;
  }
  double food_angle = food_rad * 360.0 / (2.0 * M_PI);
  
  // printf("Foodの角度 : %f\n", food_angle);


  //// 中間勾配の角度計算やラジアン計算
  double harf_rad = atan2(mydata->halfway_bot.pos[Y], mydata->halfway_bot.pos[X]);
  if (harf_rad < 0)
  {
    harf_rad = harf_rad + 2 * M_PI;
  }
  double harf_angle = harf_rad * 360.0 / (2.0 * M_PI);
  // if(kilo_uid == 59)
  // printf("回転する前の中央勾配角度 : %f\n", harf_angle);

  double cal1 = 360.0 - food_angle;
  double cal2 = cal1 + harf_angle;
  if(cal2 > 360.0){
    cal2 = cal2 - 360.0;
  }else if(cal2 < 0){
    cal2 = 360 + cal2;
  }
//  printf("回転後の中央勾配角度 : %f\n", cal2);


  if (cal2 > 180.0)
  {
    // printf("リバース\n");
    return 1;
  }
  else
  {
    // printf("通常\n");
    return 0;
  }
  // if (r > before_harf_r)
  // {
  //   // printf("リバース\n");
  //   return 1;
  // }
  // else
  // {
  //   // printf("通常\n");
  //   return 0;
  // }
}
void rotate_move()
{
  mydata->body_angle = mydata->body_angle + ONE_STEP_ROTATE_ANGLE;

  if (mydata->body_angle < 0)
    mydata->body_angle = 360.0 + mydata->body_angle;
  if (mydata->body_angle > 360)
    mydata->body_angle = mydata->body_angle - 360.0;
  mydata->pos[X] = mydata->pos[X] + ONE_STEP_MOVE_DIST * cos(mydata->body_angle * M_PI / 180.0);
  mydata->pos[Y] = mydata->pos[Y] + ONE_STEP_MOVE_DIST * sin(mydata->body_angle * M_PI / 180.0);

  // printf("uくるくｒ\n");
  set_motors(kilo_turn_left, 0);
  set_move_type(LEFT);
}
void go_straight()
{
  set_motors(kilo_turn_left, kilo_turn_right);
}
void stop_straight()
{
  set_motors(0, 0);
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
void update_detect_nest()
{
  if (mydata->is_detected_nest == 1)
  {
    return;
  }
  if (find_Only_Nest())
  {
    mydata->detected_nest_count++;
  }
  else
  {
    mydata->detected_nest_count = 0;
  }
  // if(find_Nest()){
  if (mydata->detected_nest_count > 128)
  {
    mydata->is_detected_nest = 1;
    mydata->pos[X] = 0.0;
    mydata->pos[Y] = 0.0;
  }
}
void update_detect_food()
{

  if (mydata->is_past_food == 1)
  {
    return;
  }
  // if (find_Food())
  // {
  //   mydata->is_past_food = 1;
  // }
  if (find_Only_Food())
  {
    mydata->past_food_count++;
  }
  else
  {
    mydata->past_food_count = 0;
  }
  if (mydata->past_food_count > 128)
  {
    mydata->is_past_food = 1;
    mydata->goal_pos[X] = mydata->pos[X];
    mydata->goal_pos[Y] = mydata->pos[Y];
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
  if (mydata->belong_type == PI)
    return;
  if (find_NewNode())
  {
    mydata->belong_type = NEW;
  }
  else
  {
    mydata->belong_type = OLD;
  }
}
uint8_t do_stop()
{

  if (mydata->belong_type == PI)
  {

    uint8_t i;
    for (i = 0; i < mydata->N_Neighbors; i++)
    {
      if (mydata->neighbors[i].n_bot_type != EXPLORER)
        continue;

      if (mydata->neighbors[i].n_belong_type == NEW)
      {
        if (kilo_uid == 43)
        {
          // printf("[1]\n");
        }
        return 1; // stop
      }
    }
  }
  else if (mydata->belong_type == NEW)
  {
    if (is_there_explorer_with_higher_id())
    {
      if (kilo_uid == 43)
      {
        // printf("[2]\n");
      }
      return 1;
    }
  }
  else
  { // belong type == OLD
    if (is_there_explorer_with_higher_id())
    {
      if (kilo_uid == 43)
      {
        // printf("[3]\n");
      }
      return 1;
    }

    uint8_t i;
    for (i = 0; i < mydata->N_Neighbors; i++)
    {
      if (mydata->neighbors[i].n_bot_type != EXPLORER)
        continue;

      if (mydata->neighbors[i].n_belong_type == PI)
      {
        if (kilo_uid == 43)
        {
          // printf("[4]\n");
        }
        return 1; // stop
      }
    }
  }
  return 0;
}

void bhv_explorer()
{
  /*
    double test_pos[2];
    test_pos[X] = 5.0;
    test_pos[Y] =-5.0;
    //// Foodの角度計算やラジアン計算
    double test_rad = atan2(test_pos[Y], test_pos[X]);
    if (test_rad < 0)
    {
      test_rad = test_rad + 2 * M_PI;
    }
    double test_angle = test_rad * 360.0 / (2.0 * M_PI);
    printf("Foodの角度 : %f\n" ,test_angle);
    double test_radian = test_angle * (M_PI / 180.0);
    // printf("before radian : %f, after radian : %f\n",test_rad,test_radian);

    // 回転行列をかけたFoodの角度
    double after_test_rad = atan2( \
       cos(-test_radian) *test_pos[X]  + sin(-test_radian) * test_pos[Y],\
       cos(-test_radian) * test_pos[X] - sin(-test_radian) * test_pos[Y]);
    if (after_test_rad < 0)
    {
      after_test_rad = after_test_rad + 2 * M_PI;
    }
    double after_test_angle = after_test_rad * 360.0 / (2.0 * M_PI);
    // if(kilo_uid == 59)
    printf("回転したあとのFoodの角度 : %f\n", after_test_angle);

  */

  double r = atan2(mydata->pos[Y], mydata->pos[X]);
  if (r < 0)
  {
    r = r + 2 * M_PI;
  }
  r = (r * 360.0) / (2.0 * M_PI);
  // if (kilo_uid == 59)
    // printf("(x, y) = (%f, %f), 原点からの角度 : %f\n", mydata->pos[X], mydata->pos[Y], r);

  update_harfway_info();
  if (mydata->is_detected_nest == 1)
    update_detect_food();
  update_detect_nest();
  update_self_info();
  if (do_stop() == 1)
  {
    if (mydata->belong_type == PI)
    {

      if ((find_Nest() || find_NewNode()))
      {
        set_bot_type(NEW_NODE);
      }
    }
    // printf("Kilo Uid : %d\n", kilo_uid);
    set_motors(0, 0);
    return;
  }

  // printf("pos(x, y) = (%f, %f)\n", mydata->pos[X], mydata->pos[Y]);

  double angle_acos = acos(mydata->pos[X] / sqrt(pow(mydata->pos[X], 2) + pow(mydata->pos[Y], 2)) * sqrt(pow(1.0, 2) + pow(0.0, 2))) * 180.0 / M_PI;
  if (mydata->pos[Y] < 0)
    angle_acos = 360.0 - angle_acos;

  if (past_Food() && mydata->is_detected_half == 1)
  {
    if (fabs(angle_trim(180 + angle_acos) - mydata->body_angle) < 1.0)
    {

      if (find_Explorer())
      {
        stop_straight();
      }
      else
      {
        mydata->belong_type = PI;
        go_straight();
      }
      if ((find_Nest() || find_NewNode()))
      {
        set_bot_type(NEW_NODE);
      }
    }
    else if (is_reverse())
    {
      rotate_move();
      mydata->belong_type = PI;
    }
    else
    {
      follow_edge();
    }
    if ((find_Nest() || find_NewNode()))
    {
      set_bot_type(NEW_NODE);
    }
  }
  else
  {
    follow_edge();
    // if (is_there_explorer_with_higher_id())
    // {
    //   set_motors(0, 0);
    //   set_move_type(STOP);
    //   return;
    // }
  }
  if ((find_Food() && find_NewNode()))
  {
    set_bot_type(NEW_NODE);
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
  }
  else if (get_bot_type() == FOOD)
  {
  }
  else if (get_bot_type() == EXPLORER)
  {
    // printf("%d\n", mydata->N_Neighbors);
    // if ((kilo_uid - 40) * 15000 < kilo_ticks) // increase robot
    // if ((kilo_uid - 20) * 15000 < kilo_ticks)
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
  n = sprintf(p, "ID: %d, belong to %d. bot_type : %d\n", kilo_uid, mydata->belong_type, mydata->bot_type);
  p += n;

  n = sprintf(p, "Ns: %d, dist: %d, Gradient : %d, max_gradient : %d\n ", mydata->N_Neighbors, find_nearest_N_dist(), mydata->gradient, mydata->max_gradient);
  p += n;

  return botinfo_buffer;
}
#endif
