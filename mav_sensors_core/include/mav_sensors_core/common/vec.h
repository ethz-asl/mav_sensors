/*
BSD 3-Clause License

Copyright (c) 2024, ETH Zurich, Autonomous Systems Lab, Mariano Biasio, Rik Girod

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this
   list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its
   contributors may be used to endorse or promote products derived from
   this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

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