#include <math.h>

#include <kilombo.h>

#include "kilobot.h"



REGISTER_USERDATA(MyUserdata)

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


void setup()
{
  rand_seed(kilo_uid + 1); //seed the random number generator
  mydata->message_lock = 0;
  mydata->N_Neighbors = 0;

}

void loop()
{
}


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

////////////////////////////////////////////////
//////////////// SIMULATOR VIEW ////////////////
////////////////////////////////////////////////
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
