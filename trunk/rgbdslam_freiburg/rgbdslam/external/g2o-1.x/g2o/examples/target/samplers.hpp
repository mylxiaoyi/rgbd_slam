#ifndef __SAMPLERS_HPP__
#define __SAMPLERS_HPP__

#include <tr1/random>

class Sample
{
  static std::tr1::ranlux_base_01 gen_real;
  static std::tr1::mt19937 gen_int;
public:
  static int uniform(int from, int to);
  
  static double uniform();

  static double gaussian(double sigma);

  static void seed();
};

#endif
