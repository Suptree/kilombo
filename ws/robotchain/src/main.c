#include <math.h>

#include <kilombo.h>

#include "kilobot.h"



REGISTER_USERDATA(MyUserdata)



void setup()
{
}

void loop()
{
}


int main(void)
{
  kilo_init();

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
