#ifndef KILOBOT_H
#define KILOBOT_H

#include <kilombo.h>
#define MAXN 20
#define RB_SIZE 16  // Ring buffer size. Choose a power of two for faster code

typedef struct
{
  Neighbor_t neighbors[MAXN];

  int N_Neighbors;
  uint8_t bot_type;  // NEST, FOOD, ROBOT
  uint8_t bot_state; // EXPLORER, NODE

  message_t transmit_msg;
  char message_lock;

  received_message_t RXBuffer[RB_SIZE];
  uint8_t RXHead, RXTail;

} MyUserdata;

typedef struct {
  uint16_t ID;
  uint8_t dist;
//  int delta_dist;

  uint8_t n_bot_state;
  uint8_t N_Neighbors;
  uint32_t timestamp;

} Neighbor_t;
typedef struct {
    message_t msg;
    distance_measurement_t dist;
} received_message_t;

#define RB_init() {	\
    mydata->RXHead = 0; \
    mydata->RXTail = 0;\
}

#define RB_empty() (mydata->RXHead == mydata->RXTail)

#define RB_full()  ((mydata->RXHead+1)%RB_SIZE == mydata->RXTail)

#define RB_front() mydata->RXBuffer[mydata->RXHead]

#define RB_back() mydata->RXBuffer[mydata->RXTail]

#define RB_popfront() mydata->RXHead = (mydata->RXHead+1)%RB_SIZE;

#define RB_pushback() {\
    mydata->RXTail = (mydata->RXTail+1)%RB_SIZE;\
    if (RB_empty())\
      { mydata->RXHead = (mydata->RXHead+1)%RB_SIZE;\
  printf("Full.\n"); }\
}

#endif