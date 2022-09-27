#include <iostream>
#include <cmath>
#include <chrono>

int performanceTestLoopAmount;
float accuracyTestValue;

float sqrt1(const float &n) {
    union {
        int i;
        float x;
    } u;

    u.x =  n;
    u.i =  536870912 + (u.i >> 1) - 4194304;
    u.x += n / u.x;

    return 0.25f * u.x + n / u.x;
}

float sqrt2(const float &n) {
    int i = *(int*)&n;
    i = 0x5f3759df - (i >> 1);
    float r = *(float*)&i;
    r *= (1.5f - 0.5f*n*r*r); // Repeating increases accuracy
    return r * n;
}

float sqrt3(const float &n) {
    union {
        int i;
        float x;
    } u;

    u.x = n;
    u.i = 536870912 + (u.i >> 1) - 4194304;

    return u.x;
}

float sqrt4(const float &n) {
    int i = 0;
    while(i * i <= n)
        ++i;
    --i;
    float p = (n - i*i) / (2*i);
    float a = i + p;
    return a-(p*p) / (2*a);
}

float sqrt5(const float &n) {
    float i = 0;
    while(i * i <= n)
        i += 0.1f;

    float x1 = i, x2;
    for(int j = 0; j < 10; ++j) {
        x2 = n;
        x2 /= x1;
        x2 += x1;
        x2 *= 0.5f;
        x1 = x2;
    }
    return x2;
}

float sqrt6(const float &n) {
    unsigned int i = *(unsigned int*) &n;
    i += 1065353216;
    i >>= 1;
    return *(float*) &i;
}

double sqrt7(const float &n) {
    double x = n * 0.5f;
    double lstX = 0.0;
    while(x != lstX) {
        lstX = x;
        x = (x + n/x) * 0.5f;
    }
    return x;
}

double sqrt8(const float &n) {
    double number = n * 0.5f;
    do {
        number = (number + n / number) * 0.5f;
    } while(std::abs(number * number - n) > 1.0e-7);

    return number;
}

double sqrt9(const float &n) {
    double low, high, guess;

    if (n < 1) {
        low = n;
        high = 1;
    } else {
        low = 1;
        high = n;
    }

    while ((high-low) > .001) { // .001 = accuracy
        guess = (low + high) * 0.5f;

        if (guess * guess > n)
            high = guess;
        else
            low = guess;
    }
    return (low + high) * 0.5f;
}


double sqrt10(const float &n) { 
    double x = 1;

    for(int i = 0; i < n; ++i)
        x = 0.5f * (x + n/x);

    return x;
}

float sqrt11(const float &n) {
    int x = 0x1FBD1DF5 + (*(int*)&n >> 1);
    return *(float*)&x;
}

float sqrt12(const float &n) {
    return std::sqrt(n);
}

float sqrt13(const float &n)
{
	long	i;
	float	x2, y;
	const float	xhalfs = n/2;
	x2 = n * 0.5;
	y = n;
	i = *(long *)&y;
	i = 0x1FBD3EE7ul + (i >> 1);
	y = *(float *)&i;
	y = y / 2 + xhalfs / y;
	y = y / 2 + xhalfs / y;
	return	y;
}

unsigned int getTime(auto func) {
    auto t1 = std::chrono::high_resolution_clock::now();
    for (int i = 0; i <= performanceTestLoopAmount; ++i) {
        func(i);
    }
    auto t2 = std::chrono::high_resolution_clock::now();
    return std::chrono::duration_cast<std::chrono::nanoseconds>(t2 - t1).count();
}

int main() {

    performanceTestLoopAmount = 100;
    accuracyTestValue = 19;

    std::cout << "----Execution time (in nanoseconds)----\n";
    std::cout << "sqrt():   " << getTime(sqrt) << "ns\n\n";
    std::cout << "sqrt1():  " << getTime(sqrt1) << "ns\n";
    std::cout << "sqrt2():  " << getTime(sqrt2) << "ns\n";
    std::cout << "sqrt3():  " << getTime(sqrt3) << "ns\n";
    std::cout << "sqrt4():  " << getTime(sqrt4) << "ns\n";
    std::cout << "sqrt5():  " << getTime(sqrt5) << "ns\n";
    std::cout << "sqrt6():  " << getTime(sqrt6) << "ns\n";
    std::cout << "sqrt7():  " << getTime(sqrt7) << "ns\n";
    std::cout << "sqrt8():  " << getTime(sqrt8) << "ns\n";
    std::cout << "sqrt9():  " << getTime(sqrt9) << "ns\n";
    std::cout << "sqrt10(): " << getTime(sqrt10) << "ns\n";
    std::cout << "sqrt11(): " << getTime(sqrt11) << "ns\n";
    std::cout << "sqrt12(): " << getTime(sqrt12) << "ns\n";
    std::cout << "sqrt13(): " << getTime(sqrt13) << "ns\n";

    std::cout << "----Accuracy (relative to sqrt())----\n";
    std::cout.precision(30);
    std::cout << "sqrt:   " << sqrt(accuracyTestValue) << "\n\n";
    std::cout << "sqrt1:  " << sqrt1(accuracyTestValue) << '\n';
    std::cout << "sqrt2:  " << sqrt2(accuracyTestValue) << '\n';
    std::cout << "sqrt3:  " << sqrt3(accuracyTestValue) << '\n';
    std::cout << "sqrt4:  " << sqrt4(accuracyTestValue) << '\n';
    std::cout << "sqrt5:  " << sqrt5(accuracyTestValue) << '\n';
    std::cout << "sqrt6:  " << sqrt6(accuracyTestValue) << '\n';
    std::cout << "sqrt7:  " << sqrt7(accuracyTestValue) << '\n';
    std::cout << "sqrt8:  " << sqrt8(accuracyTestValue) << '\n';
    std::cout << "sqrt9:  " << sqrt9(accuracyTestValue) << '\n';
    std::cout << "sqrt10: " << sqrt10(accuracyTestValue) << '\n';
    std::cout << "sqrt11: " << sqrt11(accuracyTestValue) << '\n';
    std::cout << "sqrt12: " << sqrt12(accuracyTestValue) << '\n';
    std::cout << "sqrt13: " << sqrt13(accuracyTestValue) << '\n';

}