// Copyright (c) 2024 ETH Zurich, Autonomous Systems Lab, Mariano Biasio, Rik Girod

//
// Created by acey on 07.07.23.
//

#pragma once

#include <sstream>
#include <string>

namespace mav_sensors {

template <typename T>
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

template <typename T>
inline vec3<T> operator/(const vec3<T> t, T num) {
  return {t.x / num, t.y / num, t.z / num};
}

template <typename T>
inline vec3<T> operator/=(vec3<T> &t, T num) {
  return {t.x /= num, t.y /= num, t.z /= num};
}

template <typename T>
inline vec3<T> operator*(const vec3<T> t, T num) {
  return {t.x * num, t.y * num, t.z * num};
}

template <typename T>
inline vec3<T> operator*=(vec3<T> &t, T num) {
  return {t.x *= num, t.y *= num, t.z *= num};
}

template <typename T>
inline vec3<T> operator+(const vec3<T> t, T num) {
  return {t.x + num, t.y + num, t.z + num};
}

template <typename T>
inline vec3<T> operator+=(vec3<T> &t, T num) {
  return {t.x += num, t.y += num, t.z += num};
}

template <typename T>
inline vec3<T> operator+(const vec3<T> t1, const vec3<T> t2) {
  return {t1.x + t2.x, t1.y + t2.y, t1.z + t2.z};
}

template <typename T>
inline vec3<T> operator+=(vec3<T> &t1, const vec3<T> t2) {
  return {t1.x += t2.x, t1.y += t2.y, t1.z += t2.z};
}

template <typename T>
inline vec3<T> operator-(const vec3<T> t, T num) {
  return {t.x - num, t.y - num, t.z - num};
}

template <typename T>
inline vec3<T> operator-=(vec3<T> &t, T num) {
  return {t.x -= num, t.y -= num, t.z -= num};
}

}  // namespace mav_sensors