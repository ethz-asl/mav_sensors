//
// Created by acey on 07.07.23.
//

#pragma once

#include <string>
#include <sstream>

template<typename T>
struct vec3 {
  T x;
  T y;
  T z;

  std::string toString() {
    std::stringstream ss;
    ss << "X: " << x << " Y: " << y << " Z: " << z;
    return ss.str();
  }
};

template<typename T>
inline vec3<T> operator/(const vec3<T> t, T num) {
  return {t.x / num, t.y / num, t.z / num};
}

template<typename T>
inline vec3<T> operator/=(vec3<T> &t, T num) {
  return {t.x /= num, t.y /= num, t.z /= num};
}

template<typename T>
inline vec3<T> operator*(const vec3<T> t, T num) {
  return {t.x * num, t.y * num, t.z * num};
}

template<typename T>
inline vec3<T> operator*=(vec3<T> &t, T num) {
  return {t.x *= num, t.y *= num, t.z *= num};
}

template<typename T>
inline vec3<T> operator+(const vec3<T> t, T num) {
  return {t.x + num, t.y + num, t.z + num};
}

template<typename T>
inline vec3<T> operator+=(vec3<T> &t, T num) {
  return {t.x += num, t.y += num, t.z += num};
}

template<typename T>
inline vec3<T> operator-(const vec3<T> t, T num) {
  return {t.x - num, t.y - num, t.z - num};
}

template<typename T>
inline vec3<T> operator-=(vec3<T> &t, T num) {
  return {t.x -= num, t.y -= num, t.z -= num};
}