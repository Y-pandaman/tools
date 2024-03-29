#pragma once

#include <algorithm>
#include <random>

namespace yalk {

/// \brief Generator of pseudo-random numbers with standard normal distribution
/// based on Box-Muller transformation
/// \details Outputs two real values that are random, independent, and have
/// standard normal distribution (with mean 0 and variance 1)
/// \details Uses standard C++ random generator rand()
/// \details To reseed the generator rand(), call
///     srand ( (unsigned)time(NULL) );
/// before you start calling this function
/// \tparam T type of the data
/// \param x the first random real value
/// \param y the second random real value
template <typename T>
void RandomNormalPair(T &x, T &y) {
  T rand1, rand2, wrand;

  // version 1, by direct calculation (slower)
  // T pi = 3.141592653589793;
  // rand1 = (T)rand() / RAND_MAX;
  // rand2 = (T)rand() / RAND_MAX;
  // x = sqrt(-2.0 * log(rand1)) * cos(2.0 * pi * rand2);
  // y = sqrt(-2.0 * log(rand1)) * sin(2.0 * pi * rand2);

  // version 2, in polar form
  // (faster and more stable numerically)
  do {
    rand1 = 2.0 * rand() / RAND_MAX - 1.0;
    rand2 = 2.0 * rand() / RAND_MAX - 1.0;
    wrand = rand1 * rand1 + rand2 * rand2;
  } while (wrand >= 1.0);
  wrand = sqrt((-2.0 * log(wrand)) / wrand);
  x = rand1 * wrand;
  y = rand2 * wrand;
}

/// \brief Generate a random number with uniform distribution
/// \tparam T type of the data
/// \param min minimum value
/// \param max maximum value
/// \return a random number with uniform distribution
template <typename T>
inline T RandomUniform(T min, T max) {
  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_real_distribution<> dis(min, max);
  return dis(gen);
}

/// \brief Generate a random integer with uniform distribution
/// \tparam T type of the data
/// \param min minimum value
/// \param max maximum value
/// \return a random integer with uniform distribution
template <typename T>
inline T RandomUniformInt(T min, T max) {
  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_int_distribution<T> dis(min, max);
  return dis(gen);
}

/// \brief Choose a random element from a vector with uniform distribution
/// \tparam T type of the data
/// \param choices a vector of choices
/// \return a random element from the vector
template <typename T>
inline T RandomChoice(const std::vector<T> &choices) {
  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_int_distribution<> dis(0, choices.size() - 1);
  return choices[dis(gen)];
}

/// \brief Choose random elements from a vector with uniform distribution
/// \tparam T type of the data
/// \param choices a vector of choices
/// \param num_choices number of choices
/// \return a vector of random elements from the vector
template <typename T>
inline std::vector<T> RandomChoices(const std::vector<T> &choices,
                                    const std::size_t num_choices) {
  std::random_device rd;
  std::mt19937 gen(rd());

  std::vector<T> results;
  while (results.size() < num_choices) {
    std::uniform_int_distribution<> dis(0, choices.size() - 1);
    auto chosen = choices[dis(gen)];
    if (std::find(results.begin(), results.end(), chosen) == results.end()) {
      results.push_back(chosen);
    }
  }

  return results;
}

/// \brief Generate a random string with the given length.
/// \param length The length of the random string.
/// \return The random string.
inline std::string RandomString(const std::size_t length) {
  std::string str(
      "0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz!@#$%^&*()"
      "_+|-=\\{}[]:;\"'<>,.?/");
  std::random_device rd;
  std::mt19937 generator(rd());
  std::shuffle(str.begin(), str.end(), generator);
  return str.substr(0, length);
}

}  // namespace yalk
