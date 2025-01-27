#pragma once
#include "Core.h"
#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <math.h>
#include <raylib.h>
#include <sys/types.h>
#include <tuple>
#include <vector>

namespace bMath {
class BMathFunctions {
public:
  static badger::real Magnitude(const badger::Vector3 &v) {
    return sqrtf(v.x * v.x + v.y * v.y + v.z * v.z);
  }

  static badger::real SquareMagnitude(const badger::Vector3 &v) {
    return (v.x * v.x + v.y * v.y + v.z * v.z);
  }

  static badger::real Magnitude(const badger::Point &v) {
    return sqrtf(v.x * v.x + v.y * v.y + v.z * v.z);
  }
  static badger::Vector3 Normalize(const badger::Vector3 &v) {
    badger::real mag = Magnitude(v);
    badger::Vector3 normalized = badger::Vector3(v.x / mag, v.y / mag, v.z / mag);
    return normalized;
  }

  static badger::Vector3 CrossProduct(const badger::Vector3 &a,
                                     const badger::Vector3 &b) {
    badger::real x = a.y * b.z - a.z * b.y;
    badger::real y = a.z * b.x - a.x * b.z;
    badger::real z = a.x * b.y - a.y * b.x;

    return badger::Vector3(x, y, z);
  }

  static badger::Point CrossProduct(const badger::Point &a,
                                   const badger::Point &b) {
    badger::real x = a.y * b.z - a.z * b.y;
    badger::real y = a.z * b.x - a.x * b.z;
    badger::real z = a.x * b.y - a.y * b.x;

    return badger::Point{.x = x, .y = y, .z = z};
  }

  static badger::real CrossProduct2D(badger::real x1, badger::real y1,
                                    badger::real x2, badger::real y2) {
    return (x1 * y2) - (x2 * y1);
  }

  static badger::real DotProduct(const badger::Point &a, const badger::Point &b) {
    badger::real x = a.x * b.x;
    badger::real y = a.y * b.y;
    badger::real z = a.z * b.z;
    return x + y + z;
  }

  static badger::Point Normalize(const badger::Point &a) {
    badger::real magnitude = Magnitude(a);
    badger::real x = a.x / magnitude;
    badger::real y = a.y / magnitude;
    badger::real z = a.z / magnitude;

    return badger::Point{.x = x, .y = y, .z = z};
  }

  // computes barycentric cordinates for triangle
  // From u=PBC/ABC , v=PCA/ABC
  static std::tuple<badger::real, badger::real, badger::real>
  BaryCentric(const badger::Point &p, const badger::Point &a,
              const badger::Point &b, const badger::Point &c) {

    // uA + vB + wC = P
    // u = PBC/ABC
    // v = PCA/ABC
    badger::real u = 0.f;
    badger::real v = 0.f;
    badger::real w = 0.f;
    // area of actual triangle ABC
    badger::Point AB{.x = b.x - a.x, .y = b.y - a.y, .z = b.z - a.z};
    badger::Point AC{.x = c.x - a.x, .y = c.y - a.y, .z = c.z - a.z};
    badger::Point ABC = CrossProduct(AB, AC);
    // determine best view plane
    badger::real x = std::abs(ABC.x), y = std::abs(ABC.y), z = std::abs(ABC.z);
    badger::real PBC = 0.f;
    badger::real PCA = 0.f;
    badger::real denom = 0.f;
    // yz plane
    if (x >= y && x >= z) {
      // PBC
      badger::Vector3 PB(0.f, (p.y - b.y), (p.z - b.z));
      badger::Vector3 BC(0.f, (b.y - c.y), (b.z - c.z));
      PBC = CrossProduct2D(PB.y, PB.z, BC.y, BC.z);
      // PCA
      badger::Vector3 PC(0.f, (p.y - c.y), (p.z - c.z));
      badger::Vector3 CA(0.f, (c.y - a.y), (c.z - a.z));
      PCA = CrossProduct2D(PC.y, PC.z, CA.y, CA.z);
      denom = ABC.x;
    }
    // xz plane
    else if (y >= x && y >= z) {
      // PBC
      badger::Vector3 PB((p.x - b.x), 0, (p.z - b.z));
      badger::Vector3 BC((b.x - c.x), 0, (b.z - c.z));
      PBC = CrossProduct2D(PB.x, PB.z, BC.x, BC.z);
      // PCA
      badger::Vector3 PC((p.x - c.x), 0, (p.z - c.z));
      badger::Vector3 CA((c.x - a.x), 0, (c.z - a.z));
      PCA = CrossProduct2D(PC.x, PC.z, CA.x, CA.z);
      denom = -ABC.y;
    }
    // xy planes
    else {
      // PBC
      badger::Vector3 PB((p.x - b.x), (p.y - b.y), 0);
      badger::Vector3 BC((b.x - c.x), (b.y - c.y), 0);
      PBC = CrossProduct2D(PB.x, PB.y, BC.x, BC.y);
      // PCA
      badger::Vector3 PC((p.x - c.x), (p.y - c.y), 0);
      badger::Vector3 CA((c.x - a.x), (c.y - a.y), 0);
      PCA = CrossProduct2D(PC.x, PC.y, CA.x, CA.y);
      denom = ABC.z;
    }
    u = PBC / denom;
    v = PCA / denom;
    w = (1 - u - v);
    return {u, v, w};
  }

  static bool TestPointInTriangle(const badger::Point &p, const badger::Point &a,
                                  const badger::Point &b,
                                  const badger::Point &c) {
    auto [u, v, w] = BaryCentric(p, a, b, c);
    return u >= 0 && v >= 0 && (u + v) <= 1;
  }

  // Computes plane equation given three points forming a triangle in ccw
  static badger::Plane ComputePlane(const badger::Point &a, const badger::Point &b,
                                   const badger::Point &c) {
    badger::Plane p;
    badger::Point _n = Normalize(CrossProduct(b - a, c - a));
    badger::Vector3 n(_n.x, _n.y, _n.z);
    p.n = n;
    p.d = bMath::BMathFunctions::DotProduct(_n, a);
    return p;
  }
  // Checks if Quad is convex
  // returns true if convex
  static bool isConvexQuad(const badger::Point &a, const badger::Point &b,
                           const badger::Point &c, const badger::Point &d) {
    // dot(normal(ABC), normal(ACD) )
    badger::real dot = DotProduct(CrossProduct((d - b), (a - b)),
                                 CrossProduct((d - b), (c - b)));
    if (dot >= 0.0f) {
      return false;
    }
    return true;
  }

  static bool ispointIsRightOfEdge2D(const badger::Point &A,
                                     const badger::Point B,
                                     const badger::Point &p) {
    badger::Point AB = B - A;
    badger::Point AP = p - A;
    badger::real cross = CrossProduct2D(AB.x, AB.y, AP.x, AP.y);
    return cross > 0.0f;
  }
  // Constructs convex hull on x y coordinate
  //
  static std::vector<badger::Point>
  AndrewConvexHullXY(std::vector<badger::Point> points, bool upper = true) {
    // sort points
    std::sort(points.begin(), points.end(),
              [](const badger::Point &a, const badger::Point &b) {
                return a.x <= b.x;
              });

    std::vector<badger::Point> convexHullPoints = {points[0], points[1]};
    badger::Point currentEdge;
    badger::Point A;
    badger::Point B;
    badger::Point p;
    for (size_t i = 2; i < points.size(); i++) {
      A = convexHullPoints[convexHullPoints.size() - 1];
      B = convexHullPoints[convexHullPoints.size() - 2];
      p = points[i];
      if (upper) {
        if (ispointIsRightOfEdge2D(A, B, p) || convexHullPoints.size() < 2) {
          convexHullPoints.push_back(p);
          continue;
        }
      } else {
        if (!ispointIsRightOfEdge2D(A, B, p) || convexHullPoints.size() < 2) {
          convexHullPoints.push_back(p);
          continue;
        }
      }

      convexHullPoints.pop_back();
      i--; // check the current point again on next iteration for the current
           // last edge after removal above
    }
    return convexHullPoints;
  }

  // Finds point farthest from edge ab
  static int32_t pointFarthestFromEdge(const badger::Point &a,
                                       const badger::Point &b,
                                       std::vector<badger::Point> &setOfPoints,
                                       bool upper = true) {
    badger::Point edge = (b - a), edgePerp = {-edge.y, edge.x, 0.f};
    if (upper) {
      edgePerp = {edge.y, -edge.x, 0.f};
    }
    int32_t bestIndex = -1;
    float dMax = -INFINITY;
    float rMax = -INFINITY;
    for (size_t i = 0; i < setOfPoints.size(); i++) {
      badger::Point p = setOfPoints[i];
      badger::real d = DotProduct(edgePerp, p);
      badger::real r = DotProduct(edge, p - a);
      if (d > dMax || (d == dMax && r > rMax)) {
        bestIndex = i;
        dMax = d;
        rMax = r;
      }
    }
    return bestIndex;
  }
};
} // namespace badgerMath
