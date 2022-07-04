
#include <math.h>
#include <kilombo.h>
#include "random_walk_search.h"
enum
{
  X,
  Y
}; //座標を扱うときに使用
enum
{
  FALSE,
  TRUE
}; // BOOL型のように使用
enum
{
  LEFT,
  RIGHT,
  STRAIGHT,
  STOP
};

typedef struct
{
  Neighbor_t neighbors[MAXN];

  int N_Neighbors;
  uint8_t bot_type;              //{NEST, FOOD, NODE, EXPLORER}
  uint8_t move_type;             //{LEFT, RIGHT}
  uint8_t random_walk_move_type; //{LEFT, RIGHT, STARIGHT}
  uint32_t edge_follow_time;
  uint32_t random_walk_time;
  uint8_t detect_food;
  uint8_t received_food_info;
  uint8_t homing_flag;
  double body_angle;           //体の向き use EXPLORER bot
  double pos[2];               // rベクトル use EXPLORER bot
  double food_pos[2];
  uint8_t food_msg_angle;      // message で使用
  uint8_t food_msg_angle_sign; // messageで使用
  uint8_t food_msg_dist;       // messageで使用
  message_t transmit_msg;
  char message_lock;
  int cover_rate[10000][10000];
  FILE *fp;

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
    RGB(3, 3, 0), // 4 - yellow
    RGB(0, 3, 3), // 5 - cyan
    RGB(3, 0, 3), // 6 - purple
    RGB(3, 1, 0), // 7  - orange
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
  mydata->neighbors[i].n_bot_type = data[3];
  mydata->neighbors[i].food_msg_angle = data[4];
  mydata->neighbors[i].food_msg_angle_sign = data[5];
  mydata->neighbors[i].food_msg_dist = data[6];
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
  mydata->transmit_msg.data[3] = mydata->bot_type;
  mydata->transmit_msg.data[4] = mydata->food_msg_angle;
  mydata->transmit_msg.data[5] = mydata->food_msg_angle_sign;
  mydata->transmit_msg.data[6] = mydata->food_msg_dist;

  mydata->transmit_msg.crc = message_crc(&mydata->transmit_msg);
  mydata->message_lock = 0;
}

//////////////////////////////////////   SETUP   ///////////////////////////////////////////
void setup()
{
  rand_seed(kilo_uid + 1); // seed the random number generator
  if (kilo_uid == 0)       // NEST bot
  {
    mydata->received_food_info = 0;
    mydata->food_msg_angle = 0;
    mydata->food_msg_angle_sign = 0;
    set_bot_type(NEST);
    set_color(colorNum[0]); // black
  }
  else if (kilo_uid == 1) // NODE bot
  {
    set_bot_type(FOOD);
    set_color(colorNum[5]); // cyan
  }
  else // EXPLORER bot
  {
    set_color(colorNum[1]);
    set_bot_type(EXPLORER);
    mydata->body_angle = 90.0;
    mydata->pos[X] = (kilo_uid - 1) * 60.0;
    mydata->pos[Y] = 0.0;
    mydata->food_pos[X] = 0.0;
    mydata->food_pos[Y] = 0.0;
    mydata->edge_follow_time = 0;
    mydata->random_walk_time = 0;
    mydata->detect_food = 0;
    mydata->homing_flag = 0;
    mydata->received_food_info = 0;
    mydata->food_msg_angle = 0;
    mydata->food_msg_angle_sign = 0;
    mydata->food_msg_dist = 0;
    if(kilo_uid == 2){
      mydata->fp = fopen("random_walk.dat","w");
    }
  }

  set_move_type(STOP);
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

uint8_t find_Explorer_by_ID()
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

uint8_t find_Explorer()
{
  uint8_t i;
  for (i = 0; i < mydata->N_Neighbors; i++)
  {
    if (mydata->neighbors[i].n_bot_type == EXPLORER)
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

uint8_t find_NodeNest()
{
  uint8_t i;
  for (i = 0; i < mydata->N_Neighbors; i++)
  {
    if (mydata->neighbors[i].n_bot_type == NODENEST)
    {
      return 1;
    }
  }
  return 0;
}

uint8_t find_NodeFood()
{
  uint8_t i;
  for (i = 0; i < mydata->N_Neighbors; i++)
  {
    if (mydata->neighbors[i].n_bot_type == NODEFOOD)
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
double angle_trim(double a)
{
  if (a < 0)
    a = 360.0 + a;
  if (a > 360)
    a = a - 360.0;

  return a;
}
void get_food_info()
{
  uint8_t i;
  for (i = 0; i < mydata->N_Neighbors; i++)
  {
    if (mydata->neighbors[i].food_msg_angle != 0)
    {
      mydata->food_msg_angle = mydata->neighbors[i].food_msg_angle;
      mydata->food_msg_angle_sign = mydata->neighbors[i].food_msg_angle_sign;
      mydata->food_msg_dist = mydata->neighbors[i].food_msg_dist;
      mydata->received_food_info = TRUE;
    }
  }
}

void CalculateLocalCordinateSystem(int move_type)
{ // 0:LEFT, 1:RIGHT
  if (move_type == LEFT)
  { //左に移動したい！
    mydata->body_angle = mydata->body_angle - ONE_STEP_ROTATE_ANGLE;
  }
  else
  { // movetype == RIGHT //右に移動したい！
    mydata->body_angle = mydata->body_angle + ONE_STEP_ROTATE_ANGLE;
  }

  if (mydata->body_angle < 0)
    mydata->body_angle = 360.0 + mydata->body_angle;
  if (mydata->body_angle > 360)
    mydata->body_angle = mydata->body_angle - 360.0;
  mydata->pos[X] = mydata->pos[X] + ONE_STEP_MOVE_DIST * cos(mydata->body_angle * M_PI / 180.0);
  mydata->pos[Y] = mydata->pos[Y] + ONE_STEP_MOVE_DIST * sin(mydata->body_angle * M_PI / 180.0);
}
void move_left()
{

  set_motors(0, kilo_turn_right);
  set_move_type(LEFT);
  CalculateLocalCordinateSystem(LEFT);
}

void move_right()
{

  set_motors(kilo_turn_left, 0);
  set_move_type(RIGHT);
  CalculateLocalCordinateSystem(RIGHT);
}

void move_straight()
{

  if (get_move_type() == LEFT)
  {
    set_motors(kilo_turn_left, 0);
    set_move_type(RIGHT);
    CalculateLocalCordinateSystem(RIGHT);
    printf("move_straight - RIGHT_MOVE\n");
  }
  else
  { // get_move_type() == RIGHT
    set_motors(0, kilo_turn_right);
    set_move_type(LEFT);
    CalculateLocalCordinateSystem(LEFT);
    printf("move_straight - LEFT_MOVE\n");
  }
}

void move_stop()
{

  set_motors(0, 0);
  set_move_type(STOP);

  printf("move_stop - STOP_MOVE\n");
}
void random_walk()
{
  mydata->random_walk_time++;
  srand(kilo_ticks + kilo_uid);
  if (kilo_ticks % 50 == 0)
  {                                             // 50kilo_ticksは同じ行動を取り続ける
    mydata->random_walk_move_type = rand() % 3; // {LEFT, RIGHT, STRAIGHT}のどれかを選択
  }

  if (mydata->random_walk_move_type == LEFT)
  {

    printf("random_walk - move_left\n");
    move_left();
  }
  else if (mydata->random_walk_move_type == RIGHT)
  {
    move_right();
    printf("random_walk - move_right\n");
  }
  else
  { // mydata->random_walk_move_type == STRAIGHT
    move_straight();

    printf("random_walk - move_straight\n");
  }
}

void edge_follow()
{
  double desired_dist = 55.0;
  if (find_nearest_N_dist() > desired_dist)
  {
    move_left();
    printf("edge_follow - move_left\n");
  }
  // if(find_nearest_N_dist() < desired_dist)
  else
  {
    move_right();
    printf("edge_follow - move_right\n");
  }
}

void get_out_edge_follow()
{
  double desired_dist = 55.0;
  mydata->edge_follow_time++;
  desired_dist += mydata->edge_follow_time * 0.007;
  printf("%f\n", desired_dist);
  if (find_nearest_N_dist() > desired_dist)
  {
    move_left();
    printf("get_out_edge_follow - move_left\n");
  }
  // if(find_nearest_N_dist() < desired_dist)
  else
  {
    move_right();
    printf("get_out_edge_follow - move_right\n");
  }
}
//messageで取得したFOODの極座標情報を用いて直交座標系におけるFOODの位置を計算
void calculate_msg_food_info(){
  if(mydata->received_food_info == FALSE){
    return;
  }
  double food_angle = 0;
  if(mydata->food_msg_angle_sign == 0){
    food_angle = mydata->food_msg_angle + 180;
  }else{
    food_angle = mydata->food_msg_angle;
  }
  double food_dist = (double)(mydata->food_msg_dist)*10.0; 
  mydata->food_pos[X] = food_dist * cos(food_angle);
  mydata->food_pos[Y] = food_dist * sin(food_angle);
}

//自身の位置からFOODまでの角度を取得
double calculate_self_to_food_angle(){
  if(mydata->received_food_info == FALSE){
    return 0;
  }   

  double food_pos_x = mydata->food_pos[X] - mydata->pos[X];
  double food_pos_y = mydata->food_pos[Y] - mydata->pos[Y];
  double food_angle = atan2(food_pos_y, food_pos_x);
  if (food_angle < 0)
  {
    food_angle = food_angle + 2.0 * M_PI;
  }

  food_angle = food_angle * 360.0 / (2.0 * M_PI);

  if (food_angle > 180)
  {
    food_angle = 360 - food_angle;
  }
  return food_angle;
}

double calculate_nest_angle()
{
  double nest_angle = acos(mydata->pos[X] / sqrt(pow(mydata->pos[X], 2) + pow(mydata->pos[Y], 2)) * sqrt(pow(1.0, 2) + pow(0.0, 2))) * 180.0 / M_PI;
  if (mydata->pos[Y] < 0)
  {
    nest_angle = 360.0 - nest_angle;
  }

  nest_angle = angle_trim(180 + nest_angle);

  return nest_angle;
}
void set_food_pos()
{
  mydata->food_pos[X] = mydata->pos[X];
  mydata->food_pos[Y] = mydata->pos[Y];
}
void set_msg_food_info()
{
  double food_pos_x = mydata->food_pos[X];
  double food_pos_y = mydata->food_pos[Y];
  double food_angle = atan2(food_pos_y, food_pos_x);
  if (food_angle < 0)
  {
    food_angle = food_angle + 2.0 * M_PI;
  }

  food_angle = food_angle * 360.0 / (2.0 * M_PI);
  int food_msg_angle = (int)food_angle;

  if (food_angle > 180)
  {
    food_angle = 360 - food_angle;
    mydata->food_msg_angle = 360 - food_msg_angle;
    mydata->food_msg_angle_sign = 0;
  }
  else
  {
    mydata->food_msg_angle = food_msg_angle;
    mydata->food_msg_angle_sign = 1;
  }
  double food_dist = sqrt(pow(food_pos_x,2)+pow(food_pos_y,2));
  mydata->food_msg_dist = (int)(food_dist / 10.0);
  printf("food_msg_dist : %d\n", mydata->food_msg_dist);

}

void path_integration()
{
  set_color(colorNum[6]); // purple
  if (fabs(calculate_nest_angle() - mydata->body_angle) < 0.5)
  {
    move_straight();
    printf("path_integration - move_straight\n");
  }
  else
  {
    edge_follow();
    if(find_Food()==TRUE){
      set_food_pos();
      set_msg_food_info();
    }
    printf("path_integration - edge_follow\n");
  }

  if (find_Nest() == TRUE || find_NodeNest() == TRUE)
  {
    set_bot_type(NODENEST);
  }
}

void explore()
{

  if (fabs(calculate_nest_angle() - mydata->body_angle) < 1.0 && mydata->random_walk_time > 5000)
  {
    mydata->homing_flag = TRUE;
  }

  if (find_NodeNest() == TRUE || find_Nest() == TRUE)
  {

    get_food_info();
    if (mydata->received_food_info == TRUE)
    {
      calculate_msg_food_info();
      set_color(colorNum[7]); // orage
    }

    get_out_edge_follow();
    mydata->homing_flag = FALSE;
    mydata->random_walk_time = 0;

    printf("explore - get_out_edge_follow\n");
  }
  else if (mydata->homing_flag == TRUE)
  {
      move_straight();
      printf("explore - move_straight\n");

  }
  else
  {
    random_walk();
    printf("explore - random_walk\n");
  }
}
void target_path_integration(){

  if (fabs(calculate_self_to_food_angle() - mydata->body_angle) < 0.5)
  {
    move_straight();
    printf("target_path_integration - move_straight\n");
  }
  else
  {
    edge_follow();
    printf("target_path_integration - edge_follow\n");
  }

  if(find_Food() == TRUE || find_NodeFood() == TRUE){
    set_bot_type(NODEFOOD);
  }

}
void bhv_explorer()
{
  // if(kilo_uid == 2 ){
  //   fprintf(mydata->fp, "%f,%f\n",mydata->pos[X],mydata->pos[Y]);
  //   mydata->cover_rate[(int)mydata->pos[X]+5000][(int)mydata->pos[Y]+5000] = 1;
  //   // int i = 0, j = 0;
  //   int count = 0;
  //   for(int i = 4500; i < 5500; i++){
  //     for(int j = 4500; j < 5500; j++){
  //       if(mydata->cover_rate[i][j]==1){
  //         count++;
  //       }
  //     }
  //   }
  //   printf("cover rate : %f\n", (double)count / 1000000.0);
  // }


  // edge_follow();
  // get_out_edge_follow();
  if (find_Food() == TRUE)
  {
    mydata->detect_food = TRUE;
  }

  if (find_Explorer_by_ID() == TRUE)
  {
    move_stop();
  }
  else if (mydata->received_food_info == TRUE){
    target_path_integration();
  }
  else if (mydata->detect_food == TRUE)
  {
    path_integration();
  }
  else
  {
    explore();
  }
}

void loop()
{
  // receive messages
  receive_inputs();
  if (get_bot_type() == NEST)
  {
    get_food_info();
    if (mydata->received_food_info == TRUE)
    {
      set_color(colorNum[4]); // yellow
    }
  }
  else if (get_bot_type() == NODENEST || get_bot_type() == NODEFOOD)
  {
    set_color(colorNum[9]); // white
    set_motors(0, 0);
    set_move_type(STOP);
  }
  else if (get_bot_type() == FOOD)
  {
  }
  else if (get_bot_type() == EXPLORER)
  {
    printf("(pos_x, pos_y) = (%f, %f), body_angle : %f, nest_angle : %f, food_angle : %f\n", mydata->pos[X], mydata->pos[Y], mydata->body_angle, calculate_nest_angle(), calculate_self_to_food_angle());
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
  n = sprintf(p, "ID: %d, dist: %d, body_angle  : %f, food_angle : %f\n \
                  food_msg_angle : %d, food_msg_dist : %d\n", kilo_uid, find_nearest_N_dist(), mydata->body_angle, calculate_self_to_food_angle(),mydata->food_msg_angle, mydata->food_msg_dist);

  p += n;

  return botinfo_buffer;
}
#endif
