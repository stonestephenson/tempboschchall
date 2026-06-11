// Vec2.h — minimal 2D vector used across the core (no raylib dependency).
#pragma once

#include <cmath>

namespace cps {

struct Vec2 {
    float x = 0.0f;
    float y = 0.0f;
};

inline Vec2 operator+(Vec2 a, Vec2 b) { return {a.x + b.x, a.y + b.y}; }
inline Vec2 operator-(Vec2 a, Vec2 b) { return {a.x - b.x, a.y - b.y}; }
inline Vec2 operator*(Vec2 a, float s) { return {a.x * s, a.y * s}; }

inline float length(Vec2 a) { return std::sqrt(a.x * a.x + a.y * a.y); }

inline Vec2 normalized(Vec2 a) {
    const float l = length(a);
    return l > 1e-9f ? Vec2{a.x / l, a.y / l} : Vec2{0.0f, 0.0f};
}

// 90° counter-clockwise rotation (the left-hand normal of a forward tangent).
inline Vec2 perp(Vec2 a) { return {-a.y, a.x}; }

}  // namespace cps
