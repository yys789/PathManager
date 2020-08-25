#include "include/base.h"

bool equald( double x, double y )
{
    return (x >= (y - precision)) && (x <= (y + precision)) ;
}

bool lessd( double x, double y )
{
   return !equald(x, y) && (x - y < 0 + precision) ;
}

bool greaterd( double x, double y )
{
   return !equald(x, y) && (x - y > 0 + precision) ;
}

bool lessequald( double x, double y )
{
   return x - y <= 0 + precision ;
}

bool greaterequald( double x, double y )
{
   return x - y >= 0 + precision ;
}

