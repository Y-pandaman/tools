#include <math.h>
#include <omp.h>

unsigned long long fastPow1(int x, int y) {
  long exponent = (long)floor(y * log2(x));
  double extraTerm = (y * log2(x)) - exponent;
  unsigned long long baseValue = (unsigned long)2 << (exponent - 1);
  double multiplier = pow(2, extraTerm);
  long double answerBeforeRounding = baseValue * multiplier;
  return (unsigned long)round(answerBeforeRounding);
}

long long fastPower(long long base, long long power) {
  long long result = 1;
  while (power > 0) {
    if (power & 1) { ///等价于：power % 2 == 1
      //指数为奇数时，将分离出来的底数一次方收集，结果对1000取模，即取后三位整数
      result = result * base % 1000;
    }
    power >>= 1; //等价于：power = power / 2;因为指数为整数，可将power = power -
                 // 1; 和power = power / 2;进行合并
    //根据取模运算法则，乘数先取模再相乘再取模跟最终结果取模结果一致
    base = (base * base) % 1000;
  }
  return result;
}

int power_fun(int x, int y) {
  int ans = x;
  if (y == 0) {
    return 1;
  }
  ans *= power_fun(x, y - 1);
  return ans;
}

int fast_power(int a, int b) {
  if (b == 0)
    return 1;
  int ans = fast_power(a, b / 2);
  ans *= ans;
  if (b & 1)
    return a * ans;
  return ans;
}

int fastPowerBitmasking(int a, int n) {
  int ans = 1;
  while (n > 0) {
    if (n & 1) {
      ans *= a;
    }
    a *= a;
    n >>= 1;
  }
  return ans;
}

float Sqr(float number) {

  float a = 0.0;
  for (int j = 0; j < number; j++) {
    a = a + number;
  }

  return a;
}