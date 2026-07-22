// AUTO-COMMENT: file overview
// This file belongs to the ROS2 sentry workspace codebase.
// Keep behavior and interface changes synchronized with related modules.

//re-build 不用改
#pragma once

#include <algorithm>
#include <cmath>
#include <vector>
#include <limits>
#include <cstdint>
#include <stdexcept>
#include <utility>
#include "BasicTypes.hpp"

using namespace LangYa;


namespace BehaviorTree {

namespace Area {

    inline bool SwitchPointEnabled{false};

    inline void SetSwitchPoint(const bool enabled) noexcept {
        SwitchPointEnabled = enabled;
    }

    inline bool IsSwitchPointEnabled() noexcept {
        return SwitchPointEnabled;
    }

    inline UnitTeam OppositeUnitTeam(const UnitTeam team) noexcept {
        if (team == UnitTeam::Red) {
            return UnitTeam::Blue;
        }
        if (team == UnitTeam::Blue) {
            return UnitTeam::Red;
        }
        return team;
    }

    inline UnitTeam PointLookupTeam(const UnitTeam team) noexcept {
        return IsSwitchPointEnabled() ? OppositeUnitTeam(team) : team;
    }

    // 定义 concept，限制 T 是基本算术类型（int、double、float 等）
    template<typename T>
    concept Arithmetic = std::is_arithmetic_v<T>;

    template<Arithmetic T>
    struct Point {
        T x;
        T y;
    };

    template<Arithmetic T>
    struct Point3 {
        T x;
        T y;
        T z;
    };

    enum class ShapeType : std::uint8_t {
        Polygon,
        Polyline,
        CircleRing,
    };

    inline constexpr int kDefaultPolylineAreaWidthCm = 5;

    inline const char* ShapeTypeName(const ShapeType shape) noexcept {
        switch (shape) {
            case ShapeType::Polygon: return "polygon";
            case ShapeType::Polyline: return "polyline";
            case ShapeType::CircleRing: return "circle_ring";
            default: return "unknown";
        }
    }

    struct AreaShapeView {
        ShapeType Shape{ShapeType::Polygon};
        const std::vector<Point<int>>* Points{nullptr};
        int WidthCm{0};
    };

    inline AreaShapeView PolygonShape(const std::vector<Point<int>>& points) noexcept {
        return AreaShapeView{
            .Shape = ShapeType::Polygon,
            .Points = &points,
            .WidthCm = 0,
        };
    }

    inline AreaShapeView PolylineShape(
        const std::vector<Point<int>>& points,
        const int width_cm = kDefaultPolylineAreaWidthCm) noexcept {
        return AreaShapeView{
            .Shape = ShapeType::Polyline,
            .Points = &points,
            .WidthCm = width_cm,
        };
    }

    template<Arithmetic T>
    struct CircleRing {
        Point<T> center;
        T innerRadiusCm;
        T outerRadiusCm;
        int segmentCount{8};
        T startAngleDeg{0};
    };

    template<Arithmetic T>
    class Location {
    public:
        Location() = default;
        explicit Location(const Point<T>& pointRed, const Point<T>& pointBlue)
            : pointRed_(pointRed), pointBlue_(pointBlue) {}

        Point<T> operator()(const UnitTeam team) const {
            const auto lookup_team = PointLookupTeam(team);
            return lookup_team == UnitTeam::Red ? pointRed_ : pointBlue_;
        }

        // 判断给定点 (x, y) 是否位于该点的区域内
        bool near(const T x, const T y, const T distance, UnitTeam team) const {
            const auto lookup_team = PointLookupTeam(team);
            const auto& point = (lookup_team == UnitTeam::Red) ? pointRed_ : pointBlue_;
            T dx = x - point.x;
            T dy = y - point.y;
            T distanceSquared =  dx * dx + dy * dy;
            return distanceSquared < distance * distance;
        }

    private:
        Point<T> pointRed_;
        Point<T> pointBlue_;
    };

    template<Arithmetic T>
    class Line {
    public:
        Line() = default;
        explicit Line(const Location<T>& pointA, const Location<T>& pointB)
            : pointA_(pointA), pointB_(pointB) {}

        Point<T> A(const UnitTeam team) const {
            return pointA_(team);
        }

        Point<T> B(const UnitTeam team) const {
            return pointB_(team);
        }

    private:
        Location<T> pointA_;
        Location<T> pointB_;
    };

    template<Arithmetic T>
    class Location3 {
    public:
        Location3() = default;
        explicit Location3(const Point3<T>& pointRed, const Point3<T>& pointBlue)
            : pointRed_(pointRed), pointBlue_(pointBlue) {}

        Point3<T> operator()(const UnitTeam team) const {
            const auto lookup_team = PointLookupTeam(team);
            return lookup_team == UnitTeam::Red ? pointRed_ : pointBlue_;
        }

    private:
        Point3<T> pointRed_;
        Point3<T> pointBlue_;
    };

    template<Arithmetic T>
    class CircleRingLocation {
    public:
        CircleRingLocation() = default;
        explicit CircleRingLocation(const CircleRing<T>& ringRed, const CircleRing<T>& ringBlue)
            : ringRed_(ringRed), ringBlue_(ringBlue) {}

        CircleRing<T> operator()(const UnitTeam team) const {
            const auto lookup_team = PointLookupTeam(team);
            return lookup_team == UnitTeam::Red ? ringRed_ : ringBlue_;
        }

    private:
        CircleRing<T> ringRed_;
        CircleRing<T> ringBlue_;
    };

    template<Arithmetic T>
    class Area {
    public:
        Area() = default;
        explicit Area(std::vector<Point<T>> points)
            : boundaryPoints_(std::move(points)) {
            if (boundaryPoints_.empty()) {
                throw std::invalid_argument("Points vector cannot be empty.");
            }
        }
        ~Area() = default;

        void addBoundaryPoint(T x, T y) {
            boundaryPoints_.emplace_back(x, y);
        }

        bool isPointInside(T px, T py) const {
            if (boundaryPoints_.empty()) return false;

            bool inside = false;
            for (size_t i = 0, j = boundaryPoints_.size() - 1; i < boundaryPoints_.size(); j = i++) {
                T xi = boundaryPoints_[i].x, yi = boundaryPoints_[i].y;
                T xj = boundaryPoints_[j].x, yj = boundaryPoints_[j].y;

                // 避免除以零的情况
                if (yi == yj) continue;

                if ((yi > py) != (yj > py)) {
                    // 判断点是否在边的左侧
                    if (px < (xj - xi) * (py - yi) / static_cast<T>(yj - yi) + xi) {
                        inside = !inside;
                    }
                }
            }
            return inside;
        }

    private:
        std::vector<Point<T>> boundaryPoints_;
    };

    static const std::vector<Point<std::uint16_t>> CastleRedPoints = {
        { 632, 804 },
        { 691, 813 },
        { 724, 753 },
        { 691, 698 },
        { 632, 696 },
        { 600, 749 }
    }; // 红方堡垒

    static const std::vector<Point<std::uint16_t>> CastleBluePoints = {
        { 2168, 696 },
        { 2109, 687 },
        { 2076, 747 },
        { 2109, 802 },
        { 2168, 804 },
        { 2200, 751 }
    }; // 蓝方堡垒

    static const std::vector<Point<std::uint16_t>> CentralHighlandRedPoints = {
        { 1031, 527 },
        { 1027, 947 },
        { 1257, 1263 },
        { 1605, 1242 },
        { 1597, 1016 },
        { 1346, 794 }

    }; // 中部高地-红方

    static const std::vector<Point<std::uint16_t>> CentralHighlandBluePoints = {
        { 1726, 963 },
        { 1756, 563 },
        { 1544, 268 },
        { 1181, 268 },
        { 1193, 498 },
        { 1439, 705 }
    }; // 中部高地-蓝方

    static const std::vector<Point<std::uint16_t>> roadLandRedPoints = {
        { 522, 207 },
        { 1201, 207 },
        { 1201, 21 },
        { 522, 21 }
    }; // 公路区-红方

    static const std::vector<Point<std::uint16_t>> roadLandBluePoints = {
        { 2280, 1311 },
        { 1589, 1311 },
        { 1589, 1477 },
        { 2280, 1477}
    }; // 公路区-蓝方

    static const std::vector<Point<std::uint16_t>> baseRedPoints = {
        { 17, 947 },
        { 494, 947 },
        { 494, 527 },
        { 17, 527 }
    }; // 基地-红方

    static const std::vector<Point<std::uint16_t>> baseBluePoints = {
        { 2764, 555 },
        { 2300, 555 },
        { 2300, 980 },
        { 2764, 980}
    }; // 基地-蓝方

    static const std::vector<Point<std::uint16_t>> flyLandRedPoints = {
        { 1261, 118 },
        { 1783, 118 },
        { 1783, 17 },
        { 1261, 17}
    }; // 飞坡区-红方

    static const std::vector<Point<std::uint16_t>> flyLandBluePoints = {
        { 1532, 1384 },
        { 1019, 1384 },
        { 1019, 1489 },
        { 1532, 1489}
    }; // 飞坡区-蓝方

    // 特殊区域
    static const Area<std::uint16_t> CastleRed{CastleRedPoints};
    static const Area<std::uint16_t> CastleBlue{CastleBluePoints};
    static const Area<std::uint16_t> CentralHighLandRed{CentralHighlandRedPoints};
    static const Area<std::uint16_t> CentralHighLandBlue{CentralHighlandBluePoints};
    static const Area<std::uint16_t> RoadLandRed{roadLandRedPoints};
    static const Area<std::uint16_t> RoadLandBlue{roadLandBluePoints};
    static const Area<std::uint16_t> BaseRed{baseRedPoints};
    static const Area<std::uint16_t> BaseBlue{baseBluePoints};
    static const Area<std::uint16_t> FlyLandRed{flyLandRedPoints};
    static const Area<std::uint16_t> FlyLandBlue{flyLandBluePoints};

    inline const Area<std::uint16_t>& CastleAreaForTeam(const UnitTeam team) {
        return PointLookupTeam(team) == UnitTeam::Blue ? CastleBlue : CastleRed;
    }

    inline bool IsPointInsideCastleAreaWithMargin(
        const UnitTeam team,
        const int x,
        const int y,
        const int margin_cm) {
        if ((team != UnitTeam::Red && team != UnitTeam::Blue) || margin_cm < 0) {
            return false;
        }
        const auto& points = PointLookupTeam(team) == UnitTeam::Blue
            ? CastleBluePoints
            : CastleRedPoints;
        const auto point_inside = [&]() {
            bool inside = false;
            for (std::size_t i = 0, j = points.size() - 1; i < points.size(); j = i++) {
                const auto& pi = points[i];
                const auto& pj = points[j];
                const long long cross =
                    static_cast<long long>(x - static_cast<int>(pj.x)) *
                        static_cast<long long>(static_cast<int>(pi.y) - static_cast<int>(pj.y)) -
                    static_cast<long long>(y - static_cast<int>(pj.y)) *
                        static_cast<long long>(static_cast<int>(pi.x) - static_cast<int>(pj.x));
                if (cross == 0 &&
                    x >= std::min(static_cast<int>(pi.x), static_cast<int>(pj.x)) &&
                    x <= std::max(static_cast<int>(pi.x), static_cast<int>(pj.x)) &&
                    y >= std::min(static_cast<int>(pi.y), static_cast<int>(pj.y)) &&
                    y <= std::max(static_cast<int>(pi.y), static_cast<int>(pj.y))) {
                    return true;
                }
                if ((static_cast<int>(pi.y) > y) != (static_cast<int>(pj.y) > y)) {
                    const double intersection_x =
                        static_cast<double>(static_cast<int>(pj.x) - static_cast<int>(pi.x)) *
                            static_cast<double>(y - static_cast<int>(pi.y)) /
                            static_cast<double>(static_cast<int>(pj.y) - static_cast<int>(pi.y)) +
                        static_cast<double>(pi.x);
                    if (static_cast<double>(x) < intersection_x) {
                        inside = !inside;
                    }
                }
            }
            return inside;
        };
        if (point_inside()) {
            return true;
        }
        const double max_distance_sq = static_cast<double>(margin_cm) * margin_cm;
        for (std::size_t i = 0; i < points.size(); ++i) {
            const auto& start = points[i];
            const auto& end = points[(i + 1) % points.size()];
            const double vx = static_cast<double>(end.x) - start.x;
            const double vy = static_cast<double>(end.y) - start.y;
            const double wx = static_cast<double>(x) - start.x;
            const double wy = static_cast<double>(y) - start.y;
            const double length_sq = vx * vx + vy * vy;
            const double t = length_sq > 0.0
                ? std::clamp((wx * vx + wy * vy) / length_sq, 0.0, 1.0)
                : 0.0;
            const double dx = static_cast<double>(x) - (start.x + t * vx);
            const double dy = static_cast<double>(y) - (start.y + t * vy);
            if (dx * dx + dy * dy <= max_distance_sq) {
                return true;
            }
        }
        return false;
    }

    inline const Area<std::uint16_t>& CentralHighLandAreaForTeam(const UnitTeam team) {
        return PointLookupTeam(team) == UnitTeam::Blue ? CentralHighLandBlue : CentralHighLandRed;
    }

    inline const Area<std::uint16_t>& RoadLandAreaForTeam(const UnitTeam team) {
        return PointLookupTeam(team) == UnitTeam::Blue ? RoadLandBlue : RoadLandRed;
    }

    inline const Area<std::uint16_t>& FlyLandAreaForTeam(const UnitTeam team) {
        return PointLookupTeam(team) == UnitTeam::Blue ? FlyLandBlue : FlyLandRed;
    }

    enum class MainAreaKind : std::uint8_t {
        Base = 0,
        Highland = 1,
        PreRoadland = 2,
        ReadyRoadland = 3,
        Central = 4
    };

    // 正式道路前段主区域。与 ReadyRoadland 共用的边界点按 AreaManager 解析顺序归 ReadyRoadland。
    static const std::vector<Point<int>> RedPreRoadlandPoints = {
        { 687, 380 },
        { 758, 235 },
        { 510, 235 },
        { 510, 205 },
        { 510, 19 },
        { 389, 15 },
        { 391, 373 }
    };

    static const std::vector<Point<int>> BluePreRoadlandPoints = {
        { 2113, 1120 },
        { 2042, 1265 },
        { 2290, 1265 },
        { 2290, 1295 },
        { 2290, 1481 },
        { 2411, 1485 },
        { 2409, 1127 }
    };

    // 正式道路后段主区域，使用旧 ReadyRoadLand 的边界。
    static const std::vector<Point<int>> RedMainAreaReadyRoadlandPoints = {
        { 510, 235 },
        { 510, 19 },
        { 1251, 17 },
        { 1333, 221 }
    };

    static const std::vector<Point<int>> BlueMainAreaReadyRoadlandPoints = {
        { 2290, 1265 },
        { 2290, 1481 },
        { 1549, 1483 },
        { 1467, 1279 }
    };

    static const std::vector<Point<int>> RedRecoveryAreaPoints = {
        { 109, 286 },
        { 306, 292 },
        { 304, 205 },
        { 111, 208 }
    };

    static const std::vector<Point<int>> BlueRecoveryAreaPoints = {
        { 2691, 1214 },
        { 2494, 1208 },
        { 2496, 1295 },
        { 2689, 1292 }
    };

    static const std::vector<Point<std::uint16_t>> RedRecoveryProbePoints = {
        { 208, 248 },
        { 250, 249 },
        { 166, 247 },
        { 208, 272 },
        { 208, 225 }
    };

    static const std::vector<Point<std::uint16_t>> BlueRecoveryProbePoints = {
        { 2592, 1252 },
        { 2550, 1251 },
        { 2634, 1253 },
        { 2592, 1228 },
        { 2592, 1275 }
    };

    static const std::vector<Point<int>> RedCentralLeftLinePoints = {
        { 1205, 1260 },
        { 1026, 1006 }
    };

    static const std::vector<Point<int>> BlueCentralLeftLinePoints = {
        { 1595, 240 },
        { 1774, 494 }
    };

    static const std::vector<Point<int>> RedSettleAreaPoints = {
        { 1210, 774 },
        { 1210, 811 },
        { 1320, 900 },
        { 1437, 864 },
        { 1283, 701 }
    };

    static const std::vector<Point<int>> BlueSettleAreaPoints = {
        { 1590, 726 },
        { 1590, 689 },
        { 1480, 600 },
        { 1363, 636 },
        { 1517, 799 }
    };

    static const std::vector<Point<int>> RedMainAreaHighlandPoints = {
        { 313, 1065 },
        { 315, 1497 },
        { 1221, 1494 },
        { 1226, 1400 },
        { 976, 1403 },
        { 744, 1077 }
    };

    static const std::vector<Point<int>> BlueMainAreaHighlandPoints = {
        { 2487, 435 },
        { 2485, 3 },
        { 1579, 6 },
        { 1574, 100 },
        { 1824, 97 },
        { 2056, 423 }
    };

    static const std::vector<Point<int>> RedProtectHeroPoints = {
        { 315, 1219 },
        { 579, 1217 },
        { 577, 1079 },
        { 311, 1070 }
    };

    static const std::vector<Point<int>> BlueProtectHeroPoints = {
        { 2485, 281 },
        { 2221, 283 },
        { 2223, 421 },
        { 2489, 430 }
    };

    static const std::vector<Point<int>> RedMainAreaBasePoints = {
        { 1, 1001 },
        { 781, 1013 },
        { 1040, 1384 },
        { 1249, 1386 },
        { 1249, 1308 },
        { 916, 841 },
        { 916, 703 },
        { 985, 508 },
        { 1047, 419 },
        { 1035, 325 },
        { 733, 329 },
        { 687, 421 },
        { 347, 428 },
        { 352, 22 },
        { 146, 15 },
        { 150, 203 },
        { 100, 198 },
        { 100, 306 },
        { 1, 306 }
    };

    static const std::vector<Point<int>> BlueMainAreaBasePoints = {
        { 2799, 499 },
        { 2019, 487 },
        { 1760, 116 },
        { 1551, 114 },
        { 1551, 192 },
        { 1884, 659 },
        { 1884, 797 },
        { 1815, 992 },
        { 1753, 1081 },
        { 1765, 1175 },
        { 2067, 1171 },
        { 2113, 1079 },
        { 2453, 1072 },
        { 2448, 1478 },
        { 2654, 1485 },
        { 2650, 1297 },
        { 2700, 1302 },
        { 2700, 1194 },
        { 2799, 1194 }
    };

    static const std::vector<Point<int>> RedMainAreaCentralPoints = {
        { 1187, 269 },
        { 1182, 471 },
        { 1047, 469 },
        { 1029, 533 },
        { 1022, 953 },
        { 1235, 1247 },
        { 1618, 1247 },
        { 1606, 1031 },
        { 1753, 1033 },
        { 1778, 990 },
        { 1755, 558 },
        { 1565, 260 }
    };

    static const std::vector<Point<int>> BlueMainAreaCentralPoints = {
        { 1613, 1231 },
        { 1618, 1029 },
        { 1753, 1031 },
        { 1771, 967 },
        { 1778, 547 },
        { 1565, 253 },
        { 1182, 253 },
        { 1194, 469 },
        { 1047, 467 },
        { 1022, 510 },
        { 1045, 942 },
        { 1235, 1240 }
    };

    static const std::vector<Point<int>> CommonMainAreaCentralPoints = {
        { 1187, 269 },
        { 1182, 471 },
        { 1047, 469 },
        { 1029, 533 },
        { 1022, 953 },
        { 1235, 1247 },
        { 1613, 1231 },
        { 1618, 1029 },
        { 1753, 1031 },
        { 1771, 967 },
        { 1778, 547 },
        { 1565, 253 }
    };

    inline const char* MainAreaKindName(const MainAreaKind kind) {
        switch (kind) {
            case MainAreaKind::Base: return "base";
            case MainAreaKind::Highland: return "highland";
            case MainAreaKind::PreRoadland: return "pre_roadland";
            case MainAreaKind::ReadyRoadland: return "ready_roadland";
            case MainAreaKind::Central: return "central";
            default: return "unknown";
        }
    }

    inline const std::vector<Point<int>>& MainAreaBoundary(
        const UnitTeam team,
        const MainAreaKind kind) {
        const bool is_blue = PointLookupTeam(team) == UnitTeam::Blue;
        switch (kind) {
            case MainAreaKind::Base:
                return is_blue ? BlueMainAreaBasePoints : RedMainAreaBasePoints;
            case MainAreaKind::Highland:
                return is_blue ? BlueMainAreaHighlandPoints : RedMainAreaHighlandPoints;
            case MainAreaKind::PreRoadland:
                return is_blue ? BluePreRoadlandPoints : RedPreRoadlandPoints;
            case MainAreaKind::ReadyRoadland:
                return is_blue ? BlueMainAreaReadyRoadlandPoints : RedMainAreaReadyRoadlandPoints;
            case MainAreaKind::Central:
                return CommonMainAreaCentralPoints;
            default:
                return is_blue ? BlueMainAreaBasePoints : RedMainAreaBasePoints;
        }
    }

    inline bool IsMainAreaBoundaryPointOnSegment(
        const Point<int>& point,
        const Point<int>& start,
        const Point<int>& end) {
        const long long cross =
            static_cast<long long>(point.x - start.x) * static_cast<long long>(end.y - start.y) -
            static_cast<long long>(point.y - start.y) * static_cast<long long>(end.x - start.x);
        if (cross != 0) {
            return false;
        }
        return point.x >= std::min(start.x, end.x) &&
               point.x <= std::max(start.x, end.x) &&
               point.y >= std::min(start.y, end.y) &&
               point.y <= std::max(start.y, end.y);
    }

    inline bool IsPointInsideMainAreaBoundary(
        const std::vector<Point<int>>& boundary,
        const int x,
        const int y) {
        if (boundary.size() < 3) {
            return false;
        }

        const Point<int> point{x, y};
        bool inside = false;
        for (std::size_t i = 0, j = boundary.size() - 1; i < boundary.size(); j = i++) {
            const auto& pi = boundary[i];
            const auto& pj = boundary[j];
            if (IsMainAreaBoundaryPointOnSegment(point, pj, pi)) {
                return true;
            }
            if ((pi.y > y) != (pj.y > y)) {
                const double intersect_x =
                    static_cast<double>(pj.x - pi.x) *
                    static_cast<double>(y - pi.y) /
                    static_cast<double>(pj.y - pi.y) +
                    static_cast<double>(pi.x);
                if (static_cast<double>(x) < intersect_x) {
                    inside = !inside;
                }
            }
        }
        return inside;
    }

    inline double DistanceSqToPolylineSegment(
        const Point<int>& point,
        const Point<int>& start,
        const Point<int>& end) noexcept {
        const double px = static_cast<double>(point.x);
        const double py = static_cast<double>(point.y);
        const double ax = static_cast<double>(start.x);
        const double ay = static_cast<double>(start.y);
        const double bx = static_cast<double>(end.x);
        const double by = static_cast<double>(end.y);
        const double vx = bx - ax;
        const double vy = by - ay;
        const double length_sq = vx * vx + vy * vy;
        if (length_sq <= 1e-9) {
            const double dx = px - ax;
            const double dy = py - ay;
            return dx * dx + dy * dy;
        }
        const double wx = px - ax;
        const double wy = py - ay;
        const double t = std::clamp((wx * vx + wy * vy) / length_sq, 0.0, 1.0);
        const double cx = ax + t * vx;
        const double cy = ay + t * vy;
        const double dx = px - cx;
        const double dy = py - cy;
        return dx * dx + dy * dy;
    }

    inline bool IsPointInsidePolylineArea(
        const std::vector<Point<int>>& polyline,
        const int x,
        const int y,
        const int width_cm = kDefaultPolylineAreaWidthCm) noexcept {
        if (polyline.size() < 2) {
            return false;
        }
        const Point<int> point{x, y};
        const double half_width = static_cast<double>(std::max(0, width_cm)) * 0.5;
        const double half_width_sq = half_width * half_width;
        for (std::size_t i = 1; i < polyline.size(); ++i) {
            if (DistanceSqToPolylineSegment(point, polyline[i - 1], polyline[i]) <= half_width_sq) {
                return true;
            }
        }
        return false;
    }

    inline bool IsPointInsideAreaShape(
        const AreaShapeView& shape,
        const int x,
        const int y) noexcept {
        if (shape.Points == nullptr) {
            return false;
        }
        switch (shape.Shape) {
            case ShapeType::Polygon:
                return IsPointInsideMainAreaBoundary(*shape.Points, x, y);
            case ShapeType::Polyline:
                return IsPointInsidePolylineArea(*shape.Points, x, y, shape.WidthCm);
            default:
                return false;
        }
    }

    inline bool IsPointInsideAreaShapes(
        const std::vector<AreaShapeView>& shapes,
        const int x,
        const int y) noexcept {
        for (const auto& shape : shapes) {
            if (IsPointInsideAreaShape(shape, x, y)) {
                return true;
            }
        }
        return false;
    }

    inline std::vector<AreaShapeView> MainAreaShapes(
        const UnitTeam team,
        const MainAreaKind kind) {
        std::vector<AreaShapeView> shapes{PolygonShape(MainAreaBoundary(team, kind))};
        return shapes;
    }

    inline bool IsPointInsideMainArea(
        const UnitTeam team,
        const MainAreaKind kind,
        const int x,
        const int y) {
        if (team != UnitTeam::Red && team != UnitTeam::Blue) {
            return false;
        }
        return IsPointInsideAreaShapes(MainAreaShapes(team, kind), x, y);
    }

    inline const std::vector<Point<int>>& ProtectHeroBoundary(const UnitTeam team) {
        return PointLookupTeam(team) == UnitTeam::Blue
            ? BlueProtectHeroPoints
            : RedProtectHeroPoints;
    }

    inline std::vector<AreaShapeView> ProtectHeroShapes(const UnitTeam team) {
        std::vector<AreaShapeView> shapes{PolygonShape(ProtectHeroBoundary(team))};
        return shapes;
    }

    inline bool IsPointInsideProtectHeroArea(
        const UnitTeam team,
        const int x,
        const int y) {
        if (team != UnitTeam::Red && team != UnitTeam::Blue) {
            return false;
        }
        return IsPointInsideAreaShapes(ProtectHeroShapes(team), x, y);
    }

    inline const std::vector<Point<int>>& PreRoadlandBoundary(const UnitTeam team) {
        return PointLookupTeam(team) == UnitTeam::Blue
            ? BluePreRoadlandPoints
            : RedPreRoadlandPoints;
    }

    inline std::vector<AreaShapeView> PreRoadlandShapes(const UnitTeam team) {
        std::vector<AreaShapeView> shapes{PolygonShape(PreRoadlandBoundary(team))};
        return shapes;
    }

    inline bool IsPointInsidePreRoadlandArea(
        const UnitTeam team,
        const int x,
        const int y) {
        if (team != UnitTeam::Red && team != UnitTeam::Blue) {
            return false;
        }
        return IsPointInsideAreaShapes(PreRoadlandShapes(team), x, y);
    }

    inline const std::vector<Point<int>>& RecoveryAreaBoundary(const UnitTeam team) {
        return PointLookupTeam(team) == UnitTeam::Blue
            ? BlueRecoveryAreaPoints
            : RedRecoveryAreaPoints;
    }

    inline std::vector<AreaShapeView> RecoveryAreaShapes(const UnitTeam team) {
        std::vector<AreaShapeView> shapes{PolygonShape(RecoveryAreaBoundary(team))};
        return shapes;
    }

    inline bool IsPointInsideRecoveryArea(
        const UnitTeam team,
        const int x,
        const int y) {
        if (team != UnitTeam::Red && team != UnitTeam::Blue) {
            return false;
        }
        return IsPointInsideAreaShapes(RecoveryAreaShapes(team), x, y);
    }

    inline const std::vector<Point<std::uint16_t>>& RecoveryProbePoints(const UnitTeam team) {
        return PointLookupTeam(team) == UnitTeam::Blue
            ? BlueRecoveryProbePoints
            : RedRecoveryProbePoints;
    }

    inline const std::vector<Point<int>>& CentralLeftLineBoundary(const UnitTeam team) {
        return PointLookupTeam(team) == UnitTeam::Blue
            ? BlueCentralLeftLinePoints
            : RedCentralLeftLinePoints;
    }

    inline std::vector<AreaShapeView> CentralLeftLineShapes(const UnitTeam team) {
        std::vector<AreaShapeView> shapes{
            PolylineShape(CentralLeftLineBoundary(team), kDefaultPolylineAreaWidthCm)
        };
        return shapes;
    }

    inline bool IsPointInsideCentralLeftLineArea(
        const UnitTeam team,
        const int x,
        const int y) {
        if (team != UnitTeam::Red && team != UnitTeam::Blue) {
            return false;
        }
        return IsPointInsideAreaShapes(CentralLeftLineShapes(team), x, y);
    }

    inline const std::vector<Point<int>>& SettleAreaBoundary(const UnitTeam team) {
        return PointLookupTeam(team) == UnitTeam::Blue
            ? BlueSettleAreaPoints
            : RedSettleAreaPoints;
    }

    inline std::vector<AreaShapeView> SettleAreaShapes(const UnitTeam team) {
        std::vector<AreaShapeView> shapes{PolygonShape(SettleAreaBoundary(team))};
        return shapes;
    }

    inline bool IsPointInsideSettleArea(
        const UnitTeam team,
        const int x,
        const int y) {
        if (team != UnitTeam::Red && team != UnitTeam::Blue) {
            return false;
        }
        return IsPointInsideAreaShapes(SettleAreaShapes(team), x, y);
    }

    // 特殊点 {Red, Blue}
    static const Location<std::uint16_t> Home{ {393, 810}, {2408, 683} };
    static const Location<std::uint16_t> Base{ {401, 691}, {2400, 811} };
    static const Location<std::uint16_t> Recovery{ {183, 245}, {2619, 1249} };
    static const Location<std::uint16_t> BuffShoot{ {854, 1382}, {1946, 118} };
    static const Location<std::uint16_t> LeftHighLand{ {406, 1332}, {2392, 187} };
    static const Location<std::uint16_t> CastleLeft1{ {510, 964}, {2290, 536} };
    static const Location<std::uint16_t> CastleLeft2{ {831, 960}, {1969, 540} };
    static const Location<std::uint16_t> Castle{ {666, 749}, {2132, 749} };
    static const Location<std::uint16_t> CastleRight1{ {505, 497}, {2293, 1014} };
    static const Location<std::uint16_t> CastleRight2{ {831, 509}, {1971, 985} };
    static const Location<std::uint16_t> FlyRoad{ {868, 80} , {1929, 1402}};
    static const Location<std::uint16_t> OutpostArea{ {1138, 431}, {1635, 1079} };
    static const Location<std::uint16_t> MidShoot{ {1075, 898}, {1702, 609} };
    static const Location<std::uint16_t> LeftShoot{ {1165, 1063}, {1644, 476} };
    static const Location<std::uint16_t> OutpostShoot{ {1393, 1047}, {1434, 497} };
    static const Location<std::uint16_t> BuffAround1{ {1372, 952}, {1459, 530} };
    static const Location<std::uint16_t> BuffAround2{ {1203, 770}, {1591, 753} };
    static const Location<std::uint16_t> RightShoot{ {1240, 559}, {1549, 923} };
    static const Location<std::uint16_t> HoleRoad{ {1074, 1249}, {1677, 204} };
    // 联盟赛推荐走 /ly/navi/goal=OccupyArea，由导航侧解释为“占点区域”。
    // 这里保留一个兼容坐标，占位到中场附近，避免旧链路在 UseXY=true 时无定义。
    static const Location<std::uint16_t> OccupyArea{ {1075, 898}, {1702, 609} };
    static const Location<std::uint16_t> Highland{ {774, 1166}, {2021, 334} };
    static const Location<std::uint16_t> BaseToCentral{ {1125, 155}, {1675, 1345} };
    static const Location<std::uint16_t> CentralToBase{ {515, 100}, {2285, 1400} };
    static const Location<std::uint16_t> BuffOutpost{ {1220, 1350}, {1580, 150} };
    static const Location<std::uint16_t> OutpostGuard{ {969, 368}, {1831, 1132} };
    // Official C3/C4 positions used for own-outpost emergency defense.
    static const Location<std::uint16_t> ProtectOutpost{ {1011, 429}, {1789, 1071} };
    static const Location<std::uint16_t> PreRoadland{ {457, 72}, {2343, 1428} };
    static const Line<std::uint16_t> CentralLeft{
        Location<std::uint16_t>{
            {static_cast<std::uint16_t>(RedCentralLeftLinePoints[0].x),
             static_cast<std::uint16_t>(RedCentralLeftLinePoints[0].y)},
            {static_cast<std::uint16_t>(BlueCentralLeftLinePoints[0].x),
             static_cast<std::uint16_t>(BlueCentralLeftLinePoints[0].y)}
        },
        Location<std::uint16_t>{
            {static_cast<std::uint16_t>(RedCentralLeftLinePoints[1].x),
             static_cast<std::uint16_t>(RedCentralLeftLinePoints[1].y)},
            {static_cast<std::uint16_t>(BlueCentralLeftLinePoints[1].x),
             static_cast<std::uint16_t>(BlueCentralLeftLinePoints[1].y)}
        }
    };

    // 地图静态瞄准点，单位为 cm；z 是目标中心相对地图平面的高度。
    static const Location3<double> OutpostPose{ {1093.0, 366.0, 150.0}, {1707.0, 1134.0, 150.0} };
    static const Location3<double> BuffPose{ {1400.0, 750.0, 150.0}, {1400.0, 750.0, 150.0} };
    static const Location3<double> OutpostAimTarget{ {1093.0, 366.0, 100.0}, {1707.0, 1134.0, 100.0} };
    static const Location3<double> BuffAimTarget{ {1400.0, 750.0, 100.0}, {1400.0, 750.0, 100.0} };



} // namespace Area
} // namespace BehaviorTree

//似乎是場地的類似信息收集，一些特別的點位?
