#include "samplers.hpp"

#include <time.h>
using namespace std;


tr1::ranlux_base_01 Sample::gen_real;
tr1::mt19937 Sample::gen_int;

int Sample::uniform(int from, int to)
{
  tr1::uniform_int<int> unif(from, to);
  int sam = unif(gen_int);
  return  sam;
}

double Sample::uniform()
{
  std::tr1::uniform_real<double> unif(0.0, 1.0);
  double sam = unif(gen_real);
  return  sam;
}

double Sample::gaussian(double sigma)
{
  std::tr1::normal_distribution<double> gauss(0.0, sigma);
  double sam = gauss(gen_real);
  return  sam;
}

void Sample::seed()
{
  time_t t = time(0);
  gen_int.seed(t);
  gen_real.seed(t);
}
