#include "movement_calc.h"


double Cos(double a,double b,double c)
{
    return (a*a+b*b-c*c)/(2.0*a*b);
}
double calc_left(double x,double y)
{
		int l1=150;
		int l2=288;
    return acos(Cos(l1,sqrt(y*y+(Length/2.0+x)*(Length/2.0+x)),l2))+atan2(y,Length/2.0+x);
}
double calc_right(double x,double y)
{
		int l1=150;
		int l2=288;
    return acos(Cos(l1,sqrt(y*y+(Length/2.0-x)*(Length/2.0-x)),l2))+atan2(y,Length/2.0-x);
}

