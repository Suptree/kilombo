#include <stdio.h>
#include <math.h>
#ifndef M_PI
#define M_PI 3.141592653589793238462643383279502884197169399375105820974944
#endif
const double ONE_STEP_MOVE_DIST = 33.0 * M_PI / 660.0; //円周の長さを1kilo_tickで割ったもの
const double ONE_STEP_ROTATE_ANGLE = 360.0 / 660.0;

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
  int i = 0;
  double x= 0;
  double y = 0;
  double body_angle = 90;
 for(i = 0; i < 50; i++){
   printf("%d\n", i % 2);
   if(i % 2 == 0){
    body_angle += ONE_STEP_ROTATE_ANGLE; 
    x = x + ONE_STEP_MOVE_DIST * cos(body_angle * M_PI / 180.0);
    y = y + ONE_STEP_MOVE_DIST * sin(body_angle * M_PI / 180.0);
   }else{

    body_angle -= ONE_STEP_ROTATE_ANGLE; 
    x = x + ONE_STEP_MOVE_DIST * cos(body_angle * M_PI / 180.0);
    y = y + ONE_STEP_MOVE_DIST * sin(body_angle * M_PI / 180.0);
   }
 }
 printf("x, y = %f, %f\n", x , y);
 printf("angle :%f\n", body_angle);
}