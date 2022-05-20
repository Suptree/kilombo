#include <stdio.h>
#include <math.h>
#ifndef M_PI
#define M_PI 3.141592653589793238462643383279502884197169399375105820974944
#endif

int main(){
 double r = atan2(-0.0, 0.0);
   if (r < 0)
  {
    r = r + 2.0 * M_PI;
  }
  r = r * 360.0 / (2.0 * M_PI);
 printf("theta = %f\n", r);


 double r1 = atan2(100.0, -100.0);
   if (r1 < 0)
  {
    r1= r1 + 2.0 * M_PI;
  }
  r1 = r1 * 360.0 / (2.0 * M_PI);
 printf("theta = %f\n", r1);

}