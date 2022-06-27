#ifndef GRN_H
#define GRN_H


#include <math.h>

#ifndef M_PI
#define M_PI 3.141592653589793238462643383279502884197169399375105820974944
#endif


#define GENES 6

#define DIFFGENES 6  // Maximal number of diffusing genes, limited by the message length.

#define MAXN 20
#define RB_SIZE 16  // Ring buffer size. Choose a power of two for faster code
                    // memory usage: 16*RB_SIZE
                    // 8 works too, but complains in the simulator
                    // when the bots are very dense

extern uint8_t NGenes;

enum BOTTYPE {NEST, FOOD, NODE, EXPLORER, NODEFOOD, NODENEST};
enum BOTSTATES {WAIT, LISTEN, MOVE};

const double ONE_STEP_MOVE_DIST = 33.0 * M_PI / 660.0; //円周の長さを1kilo_tickで割ったもの
const double ONE_STEP_ROTATE_ANGLE = 360.0 / 660.0;



typedef struct {
  uint16_t ID;
  uint8_t dist;
  double food_pos[2];

  uint8_t food_msg_angle;
  uint8_t food_msg_angle_sign;

  uint8_t n_bot_type;
  uint8_t N_Neighbors;
  uint32_t timestamp;
} Neighbor_t;

typedef struct {
    message_t msg;
    distance_measurement_t dist;
} received_message_t;



// Ring buffer operations. Taken from kilolib's ringbuffer.h
// but adapted for use with mydata->

// Ring buffer operations indexed with head, tail
// These waste one entry in the buffer, but are interrupt safe:
//   * head is changed only in popfront
//   * tail is changed only in pushback
//   * RB_popfront() is to be called AFTER the data in RB_front() has been used
//   * head and tail indices are uint8_t, which can be updated atomically
//     - still, the updates need to be atomic, especially in RB_popfront()

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
      { mydata->RXHead = (mydata->RXHead+1)%RB_SIZE;	\
	printf("Full.\n"); }				\
  }


/*
// Ring buffer operations indexed with head, count
// These save one entry in the buffer, but are less interrupt safe due to the shared count
#define RB_init() {	\
    mydata->RXHead = 0; \
    mydata->RXCount = 0;\
}
#define RB_empty() (mydata->RXCount == 0)
#define RB_full()  (mydata->RXCount == RB_SIZE)
#define RB_front() (mydata->RXBuffer[mydata->RXHead])
#define RB_back()  (mydata->RXBuffer[(mydata->RXHead+mydata->RXCount)%RB_SIZE])
#define RB_popfront() {\
    mydata->RXHead = (mydata->RXHead+1)%RB_SIZE;\
    mydata->RXCount--;\
}
#define RB_pushback() {\
    if (RB_full())\
        mydata->RXHead = (mydata->RXHead+1)%RB_SIZE;\
    else\
      mydata->RXCount++;\
}
*/


#endif
