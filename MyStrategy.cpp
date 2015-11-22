#include "MyStrategy.h"

#include <cmath>
#include <memory>
#include <cstdlib>
#include <cassert>
#include <algorithm>
#include <unordered_map>
#include <limits>
#include <map>

#include <iostream>  // DEBUG
#include <iomanip>  // DEBUG

using namespace std;


constexpr double pi = 3.14159265358979323846264338327950288;


inline double frand()
{
    return rand() * (2.0 / RAND_MAX) - 1;
}


inline constexpr double sqr(double x)
{
    return x * x;
}

inline constexpr double rem(double x, double y)
{
    return y * (x / y - floor(x / y));
}

inline constexpr double relAngle(double angle)
{
    return rem(angle + pi, 2 * pi) - pi;
}

inline double limit(double val, double lim)
{
    return max(-lim, min(lim, val));
}

template<unsigned n> constexpr double pow(double val)
{
    return n & 1 ? sqr(pow<(n >> 1)>(val)) * val : sqr(pow<(n >> 1)>(val));
}
template<> constexpr double pow<1>(double val)
{
    return val;
}



struct Vec2D
{
    double x, y;

    Vec2D() = default;
    constexpr Vec2D(const Vec2D &v) = default;
    Vec2D &operator = (const Vec2D &v) = default;

    constexpr Vec2D(double x_, double y_) : x(x_), y(y_)
    {
    }

    constexpr Vec2D operator + (const Vec2D &v) const
    {
        return Vec2D(x + v.x, y + v.y);
    }

    Vec2D &operator += (const Vec2D &v)
    {
        x += v.x;  y += v.y;  return *this;
    }

    constexpr Vec2D operator - (const Vec2D &v) const
    {
        return Vec2D(x - v.x, y - v.y);
    }

    Vec2D &operator -= (const Vec2D &v)
    {
        x -= v.x;  y -= v.y;  return *this;
    }

    constexpr Vec2D operator - () const
    {
        return Vec2D(-x, -y);
    }

    constexpr Vec2D operator * (double a) const
    {
        return Vec2D(a * x, a * y);
    }

    Vec2D &operator *= (double a)
    {
        x *= a;  y *= a;  return *this;
    }

    constexpr double operator * (const Vec2D &v) const
    {
        return x * v.x + y * v.y;
    }

    constexpr Vec2D operator / (double a) const
    {
        return (*this) * (1 / a);
    }

    Vec2D operator /= (double a)
    {
        return (*this) *= (1 / a);
    }

    constexpr Vec2D operator ~ () const
    {
        return Vec2D(y, -x);
    }

    constexpr double operator % (const Vec2D &v) const
    {
        return *this * ~v;
    }

    constexpr double sqr() const
    {
        return x * x + y * y;
    }

    constexpr double len() const
    {
        return std::sqrt(x * x + y * y);
    }
};

inline constexpr Vec2D operator * (double a, const Vec2D &v)
{
    return v * a;
}

inline constexpr Vec2D normalize(const Vec2D &v)
{
    return v / v.len();
}

inline constexpr Vec2D sincos_slow(double angle)
{
    return Vec2D(cos(angle), sin(angle));
}

inline constexpr Vec2D sincos(float angle)
{
    return Vec2D(cos(angle), sin(angle));
}

inline constexpr Vec2D rotate(const Vec2D &v1, const Vec2D &v2)
{
    return Vec2D(v1.x * v2.x - v1.y * v2.y, v1.x * v2.y + v1.y * v2.x);
}

inline constexpr Vec2D conj(const Vec2D &v)
{
    return Vec2D(v.x, -v.y);
}


struct RectInfo
{
    double invMass, invAngMass;

    void set(double mass, double hw, double hh)
    {
        invMass = 1 / mass;  invAngMass = 3 * invMass / (hw * hw + hh * hh);
    }
};

struct CarInfo : public RectInfo
{
    double carAccel, carReverse;

    void set(double mass, double power, double rear);
};


constexpr double maxDist = 16;
constexpr double timeEps = 0.001;
constexpr double spdEps2 = 1e-12;

constexpr double tileToggle = 0.49;
constexpr double tileScore = 1000;

constexpr double pickupScore = 5;
constexpr double nitroCost = 50;
constexpr double scoreBonus = 500;
constexpr int repairPower = 4;
constexpr double repairScore = 500;
constexpr double slickPenalty = 1000;

constexpr int distPower = 4;
constexpr double distPenalty = 100;
constexpr double reversePenalty = 10;
constexpr double maxBlowSpeed = 3;
constexpr double pickupDepth = 5;
constexpr double pickupSpeed = 0;

constexpr double largeSpeed = 30;

constexpr int optTileDist = 8, optLookahead = 600;
constexpr int infTime = numeric_limits<int>::max();


constexpr int physIter = 1;
constexpr double physDt = 1.0 / physIter;
constexpr double carBounce = 1.25, carFrict = sqrt(0.125);
constexpr double bonusBounce = 1.5, bonusFrict = sqrt(0.5);
constexpr RectInfo borderInfo = {0, 0};

double tileSize, invTileSize, tileMargin;
int mapWidth, mapHeight, mapLine;

int nitroDuration, nitroCooldown;
double nitroPower;

double carHalfWidth, carHalfHeight, carRadius;  CarInfo carInfo[2];
double frictMul, longFrict, crossFrict, rotFrictMul, angFrict, carRotFactor;
double powerChange, invPowerChange, turnChange, invTurnChange;

double bonusHalfSize;  RectInfo bonusInfo;
double slickRadius;  int slidingTime;

int globalTick = -1;

void CarInfo::set(double mass, double power, double rear)
{
    RectInfo::set(mass, carHalfWidth, carHalfHeight);
    carAccel   = power * invMass * physDt;
    carReverse = rear  * invMass * physDt;
}

void initConsts(const model::Game &game, const model::World &world)
{
    tileSize = game.getTrackTileSize();  invTileSize = 1 / tileSize;
    tileMargin = game.getTrackTileMargin();

    mapWidth = world.getWidth();  mapHeight = world.getHeight();  mapLine = mapWidth + 1;

    nitroDuration = game.getNitroDurationTicks();
    nitroCooldown = game.getUseNitroCooldownTicks() - nitroDuration;
    nitroPower = game.getNitroEnginePowerFactor();

    carHalfWidth = game.getCarWidth() / 2;
    carHalfHeight = game.getCarHeight() / 2;
    carRadius = Vec2D(carHalfWidth, carHalfHeight).len();
    carInfo[model::BUGGY].set(game.getBuggyMass(),
        game.getBuggyEngineForwardPower(), game.getBuggyEngineRearPower());
    carInfo[model::JEEP].set(game.getJeepMass(),
        game.getJeepEngineForwardPower(), game.getJeepEngineRearPower());

    frictMul = pow(1 - game.getCarMovementAirFrictionFactor(), physDt);
    longFrict = game.getCarLengthwiseMovementFrictionFactor() * physDt;
    crossFrict = game.getCarCrosswiseMovementFrictionFactor() * physDt;
    rotFrictMul = pow(1 - game.getCarRotationAirFrictionFactor(), physDt);
    angFrict = game.getCarRotationFrictionFactor() * physDt;
    carRotFactor = game.getCarAngularSpeedFactor() * physDt;

    powerChange = game.getCarEnginePowerChangePerTick();  invPowerChange = 1 / powerChange;
    turnChange = game.getCarWheelTurnChangePerTick();  invTurnChange = 1 / turnChange;

    bonusHalfSize = game.getBonusSize() / 2 - pickupDepth;
    bonusInfo.set(game.getBonusMass(), bonusHalfSize, bonusHalfSize);

    slickRadius = game.getOilSlickRadius();
    slidingTime = game.getMaxOiledStateDurationTicks();
}


struct Quad
{
    Vec2D pos, dir;
    double hw, hh;

    Quad() = default;

    Quad(const Vec2D &pos_, const Vec2D &dir_, double hw_, double hh_) :
        pos(pos_), dir(dir_), hw(hw_), hh(hh_)
    {
    }

    Quad(const model::RectangularUnit &unit) : pos(unit.getX(), unit.getY()),
        dir(sincos(unit.getAngle())), hw(unit.getWidth() / 2), hh(unit.getHeight() / 2)
    {
    }

    Quad(const Vec2D &start, const Vec2D &delta, double size) : hh(size / 2)
    {
        dir = delta / 2;  hw = dir.len();  pos = start + dir;  dir /= hw;
    }

    Quad move(const Vec2D &pt) const
    {
        return Quad(pt, dir, hw, hh);
    }

    bool checkPoint(const Vec2D &pt, double rad = 0) const
    {
        Vec2D dr = pt - pos;
        return abs(dr * dir) < hw + rad && abs(dr % dir) < hh + rad;
    }

    void collideSide(double proj, double offs, double dot, double crs,
        const Vec2D &hitDir, double &minDepth, Vec2D &norm, Vec2D &pt) const
    {
        double depth = offs + hw * abs(dot) + hh * abs(crs) - abs(proj);
        if(!(minDepth > depth))return;

        minDepth = depth;
        norm = signbit(proj) ? -hitDir : hitDir;  pt = pos;
        pt += (signbit(dot) == signbit(proj) ? -hw : hw) *  dir;
        pt += (signbit(crs) == signbit(proj) ? -hh : hh) * ~dir;
    }

    double collide(const Quad &q, Vec2D &norm, Vec2D &pt) const
    {
        Vec2D dr = q.pos - pos;
        double dot = dir * q.dir, crs = dir % q.dir;
        double minDepth = numeric_limits<double>::infinity();
        q.collideSide(dr * dir,   hw,  dot,  crs,    dir, minDepth, norm, pt);
        q.collideSide(dr % dir,   hh,  crs, -dot,   ~dir, minDepth, norm, pt);
        collideSide(dr * q.dir, q.hw, -dot,  crs,  q.dir, minDepth, norm, pt);
        collideSide(dr % q.dir, q.hh, -crs, -dot, ~q.dir, minDepth, norm, pt);
        return minDepth;
    }
};


struct Bonus : public Quad
{
    unsigned flag;  model::BonusType type;

    Bonus() = default;

    Bonus(int index, const model::Bonus &bonus) : Quad(bonus), flag(1 << (index & 31)), type(bonus.getType())
    {
    }
};

struct Slick
{
    Vec2D pos;  int endTime;

    Slick() = default;

    Slick(const model::OilSlick &slick) : pos(slick.getX(), slick.getY()), endTime(slick.getRemainingLifetime())
    {
    }
};

struct TileMap
{
    vector<bool> borders;
    vector<int> waypoints;
    vector<vector<unsigned>> distMap;
    vector<vector<Bonus>> bonuses;
    vector<vector<Slick>> slicks;

    void init(const model::World &world)
    {
        const int line = 2 * mapLine;
        borders.resize(line * (mapHeight + 2), false);
        for(int x = 0, k1 = 2, k2 = line * mapHeight + 2; x < mapWidth; x++, k1 += 2, k2 += 2)
            borders[k1] = borders[k2] = true;
        for(int y = 0, k1 = line + 1, k2 = 2 * line - 1; y < mapHeight; y++, k1 += line, k2 += line)
            borders[k1] = borders[k2] = true;

        enum TileFlags
        {
            f_l = 1, f_r = 2, f_u = 4, f_d = 8,
            f_all = f_l | f_r | f_u | f_d
        };

        constexpr int tile[] =
        {
            f_all,      // EMPTY
            f_l | f_r,  // VERTICAL
            f_u | f_d,  // HORIZONTAL
            f_l | f_u,  // LEFT_TOP_CORNER
            f_r | f_u,  // RIGHT_TOP_CORNER
            f_l | f_d,  // LEFT_BOTTOM_CORNER
            f_r | f_d,  // RIGHT_BOTTOM_CORNER
            f_r,        // LEFT_HEADED_T
            f_l,        // RIGHT_HEADED_T
            f_d,        // TOP_HEADED_T
            f_u,        // BOTTOM_HEADED_T
            0,          // CROSSROADS
            0,          // UNKNOWN
        };

        const vector<vector<model::TileType>> &map = world.getTilesXY();
        for(int x = 0; x < mapWidth; x++)for(int y = 0; y < mapHeight; y++)
        {
            int flags = tile[map[x][y]];
            int pos = (y + 1) * line + 2 * (x + 1);
            if(flags & f_l)borders[pos -    1] = true;
            if(flags & f_r)borders[pos +    1] = true;
            if(flags & f_u)borders[pos - line] = true;
            if(flags & f_d)borders[pos +    0] = true;
        }

        /*
        for(int x = 0; x < mapWidth; x++)for(int y = 0; y < mapHeight; y++)
        {
            int flags = 0;
            int pos = (y + 1) * line + 2 * (x + 1);
            if(borders[pos -    1])flags |= f_l;
            if(borders[pos +    1])flags |= f_r;
            if(borders[pos - line])flags |= f_u;
            if(borders[pos +    0])flags |= f_d;
            assert(flags == tile[map[x][y]]);
        }
        */

        const vector<vector<int>> &wpts = world.getWaypoints();
        waypoints.reserve(wpts.size());  distMap.resize(wpts.size());
        for(auto &&pt : wpts)waypoints.push_back(pt[1] * mapLine + pt[0]);

        bonuses.resize(mapHeight * mapLine);
        slicks.resize(mapHeight * mapLine);
    }

    void reset(const model::World &world)
    {
        for(auto &list : bonuses)list.clear();  int index = 0;
        double shift = (carRadius + bonusHalfSize) * invTileSize;
        for(auto &bonus : world.getBonuses())
        {
            Bonus next(index++, bonus);  Vec2D offs = next.pos * invTileSize;
            int x = int(offs.x), y = int(offs.y), k = y * mapLine + x;  offs -= Vec2D(x, y);

            bonuses[k].push_back(next);
            if(x >             0 && offs.x <     shift)bonuses[k -       1].push_back(next);
            if(x < mapWidth  - 1 && offs.x > 1 - shift)bonuses[k +       1].push_back(next);
            if(y >             0 && offs.y <     shift)bonuses[k - mapLine].push_back(next);
            if(y < mapHeight - 1 && offs.y > 1 - shift)bonuses[k + mapLine].push_back(next);
        }

        for(auto &list : slicks)list.clear();
        shift = (carRadius + slickRadius) * invTileSize;
        for(auto &slick : world.getOilSlicks())
        {
            Slick next(slick);  Vec2D offs = next.pos * invTileSize;
            int x = int(offs.x), y = int(offs.y), k = y * mapLine + x;  offs -= Vec2D(x, y);

            slicks[k].push_back(next);
            if(x >             0 && offs.x <     shift)slicks[k -       1].push_back(next);
            if(x < mapWidth  - 1 && offs.x > 1 - shift)slicks[k +       1].push_back(next);
            if(y >             0 && offs.y <     shift)slicks[k - mapLine].push_back(next);
            if(y < mapHeight - 1 && offs.y > 1 - shift)slicks[k + mapLine].push_back(next);
        }
    }

    static void pathfinderUpdate(vector<unsigned> &map, vector<int> &queue, int pos, unsigned dist)
    {
        if(map[pos] <= dist)return;  map[pos] = dist;  queue.push_back(pos);
    }

    const vector<unsigned> &waypointDistMap(int index)
    {
        vector<unsigned> &map = distMap[index];  if(map.size())return map;

        const int line = 2 * mapLine;
        vector<int> queue, next;  queue.push_back(waypoints[index]);
        map.resize(mapHeight * mapLine, -1);  map[waypoints[index]] = 0;
        for(unsigned dist = 1; queue.size(); dist++)
        {
            for(int k : queue)
            {
                int p = 2 * k + line + 2;
                if(!borders[p -    1])pathfinderUpdate(map, next, k -       1, dist);
                if(!borders[p +    1])pathfinderUpdate(map, next, k +       1, dist);
                if(!borders[p - line])pathfinderUpdate(map, next, k - mapLine, dist);
                if(!borders[p +    0])pathfinderUpdate(map, next, k + mapLine, dist);
            }
            queue.clear();  swap(queue, next);
        }
        return map;
    }

    double collideBorder(const Quad &rect, Vec2D &norm, Vec2D &pt) const
    {
        Vec2D pos = rect.pos, dir = rect.dir;
        double hw = rect.hw, hh = rect.hh;

        Vec2D offs = pos * invTileSize + Vec2D(0.5, 0.5);
        int x = int(offs.x), y = int(offs.y);

        int dx = 2, dy = 2 * mapLine;
        int px = x * dx + y * dy, py = px + 1;

        enum SymmetryFlags
        {
            flip_x = 1, flip_y = 2, swap_xy = 4
        };

        int flags = 0;
        pos -= Vec2D(x, y) * tileSize;
        if(signbit(pos.x))
        {
            flags |= flip_x;  px += dx;  dx = -dx;
            pos.x = -pos.x;  dir.x = -dir.x;
        }
        if(signbit(pos.y))
        {
            flags |= flip_y;  py += dy;  dy = -dy;
            pos.y = -pos.y;  dir.y = -dir.y;
        }
        if(pos.x < pos.y)
        {
            flags |= swap_xy;  swap(px, py);  swap(dx, dy);
            swap(pos.x, pos.y);  swap(dir.x, dir.y);
        }

        enum CheckFlags
        {
            f_h = 1, f_v1 = 2, f_v2 = 4, f_c = 8, f_r = 16
        };

        constexpr int work[] =
        {
            /*  .  */ f_r,
            /* _.  */ f_v1 | f_r,
            /*  |  */ f_h,
            /* _|  */ f_h,
            /*  ._ */ f_v2 | f_r,
            /* _._ */ f_v1 | f_v2,
            /*  |_ */ f_c,
            /* _|_ */ f_c,
        };

        int check = work[(borders[py] ? 1 : 0) | (borders[px + dx] ? 2 : 0) | (borders[py + dy] ? 4 : 0)];

        Vec2D rw = hw * dir, rh = hh * ~dir;
        Vec2D minX = pos + (signbit(rw.x) ? rw : -rw) + (signbit(rh.x) ? rh : -rh);
        Vec2D minY = pos + (signbit(rw.y) ? rw : -rw) + (signbit(rh.y) ? rh : -rh);
        double maxDepth = -numeric_limits<double>::infinity();
        if(check & f_h)
        {
            double depth = tileMargin - minY.y;
            if(maxDepth < depth)
            {
                maxDepth = depth;  norm = {0, 1};  pt = minY;
            }
        }
        if(check & (signbit(minX.y) ? f_v1 : f_v2))
        {
            double depth = tileMargin - minX.x;
            if(maxDepth < depth)
            {
                maxDepth = depth;  norm = {1, 0};  pt = minX;
            }
        }
        if(check & f_c)
        {
            Vec2D d1(max(0.0, 2 * tileMargin - minX.x), max(0.0, 2 * tileMargin - minX.y));
            Vec2D d2(max(0.0, 2 * tileMargin - minY.x), max(0.0, 2 * tileMargin - minY.y));
            double dd1 = d1 * d1, dd2 = d2 * d2, depth = sqrt(max(dd1, dd2)) - tileMargin;
            if(maxDepth < depth)
            {
                maxDepth = depth;
                if(dd1 > dd2)
                {
                    norm = normalize(d1);  pt = minX;
                }
                else
                {
                    norm = normalize(d2);  pt = minY;
                }
            }
        }
        if(check & f_r)
        {
            double dot = pos * dir, dotLim = limit(dot, hw);
            double crs = pos % dir, crsLim = limit(crs, hh);
            Vec2D d(dot - dotLim, crs - crsLim);
            double len = d.len(), depth = tileMargin - len;
            if(maxDepth < depth)
            {
                maxDepth = depth;
                d /= len;  norm = d.x * dir + d.y * ~dir;
                pt = pos - dotLim * dir - crsLim * ~dir;
            }
        }
        if(maxDepth < -maxDist)return -maxDist;
        if(flags & swap_xy)
        {
            swap(norm.x, norm.y);  swap(pt.x, pt.y);
        }
        if(flags & flip_x)
        {
            norm.x = -norm.x;  pt.x = -pt.x;
        }
        if(flags & flip_y)
        {
            norm.y = -norm.y;  pt.y = -pt.y;
        }
        pt += Vec2D(x, y) * tileSize;
        if(!rect.checkPoint(pt, 1))cout << "NOT IN RECT!!!" << endl;
        return maxDepth;
    }
};

TileMap tileMap;


void solveImpulse(const RectInfo &info, const Vec2D &pos, Vec2D &spd, double &angSpd,
    const RectInfo &info1, const Vec2D &pos1, const Vec2D &norm, const Vec2D &pt, double coeff)
{
    double w = (pt - pos) % norm, w1 = (pt - pos1) % norm;
    double invEffMass = info.invMass + info1.invMass + w * w * info.invAngMass + w1 * w1 * info1.invAngMass;
    double impulse = coeff / invEffMass;  spd += impulse * info.invMass * norm;  angSpd += impulse * info.invAngMass * w;
}

struct CarState
{
    Vec2D pos, spd, dir;
    double angle, angSpd;

    int breakEnd, slidingEnd, nitroEnd;
    int nitroCount, ammoCount, oilCount;
    double durability, power, turn;

    int waypoint;
    unsigned base, dist;
    unsigned consumed;
    double score;

    CarState() = default;

    CarState(const model::Car &car) : pos(car.getX(), car.getY()), spd(car.getSpeedX(), car.getSpeedY()),
        dir(sincos(car.getAngle())), angle(car.getAngle()), angSpd(0)  // TODO: car.getAngularSpeed()
    {
        breakEnd = 0;  // TODO
        slidingEnd = car.getRemainingOiledTicks();
        nitroEnd = car.getRemainingNitroCooldownTicks() - nitroCooldown;
        nitroCount = car.getNitroChargeCount();
        ammoCount = car.getProjectileCount();
        oilCount = car.getOilCanisterCount();
        durability = car.getDurability();
        power = car.getEnginePower();
        turn = car.getWheelTurn();

        Vec2D offs = pos * invTileSize;  int k = int(offs.y) * mapLine + int(offs.x);
        base = tileMap.waypointDistMap(waypoint = car.getNextWaypointIndex())[k];
        dist = 0;  consumed = 0;  score = 0;
    }

    bool update(const CarInfo &info, int time, double power, double turn, double frict)
    {
        Vec2D accel = (signbit(power) ? info.carReverse : info.carAccel) * power * dir;
        double rot = carRotFactor * turn * (spd * dir), borderDist = maxDist;
        for(int i = 0; i < physIter; i++)
        {
            pos += spd * physDt;  spd += accel;  spd *= frictMul;
            spd -= limit(spd * dir, frict) * dir + limit(spd % dir, crossFrict) * ~dir;
            dir = sincos(angle += rot + angSpd * physDt);  angSpd *= rotFrictMul;
            angSpd -= limit(angSpd, angFrict);

            Quad rect(pos, dir, carHalfWidth, carHalfHeight);  Vec2D norm, pt;
            double depth = tileMap.collideBorder(rect, norm, pt);
            borderDist = min(borderDist, -depth);
            if(depth > 0)
            {
                Vec2D relSpd = spd + (rot * physIter + angSpd) * ~(pt - pos);
                double normSpd = relSpd * norm;
                if(normSpd < 0)
                {
                    if(normSpd < -maxBlowSpeed)return false;
                    double frictCoeff = carFrict * normSpd / sqrt(relSpd.sqr() + spdEps2);
                    solveImpulse(info, pos, spd, angSpd, borderInfo, pt,  norm, pt, -carBounce * normSpd);
                    solveImpulse(info, pos, spd, angSpd, borderInfo, pt, ~norm, pt, frictCoeff * (relSpd % norm));

                    // TODO: reduce score
                }
                pos += depth * norm;
            }

            Vec2D offs = pos * invTileSize;  int k = int(offs.y) * mapLine + int(offs.x);
            for(auto &bonus : tileMap.bonuses[k])if((consumed & bonus.flag) != bonus.flag)
            {
                double depth = bonus.collide(rect, norm, pt);
                if(depth < 0)continue;  consumed |= bonus.flag;

                Vec2D relSpd = spd + (rot * physIter + angSpd) * ~(pt - pos);
                double normSpd = relSpd * norm;
                if(normSpd < 0)
                {
                    double frictCoeff = bonusFrict * normSpd / sqrt(relSpd.sqr() + spdEps2);
                    solveImpulse(info, pos, spd, angSpd, bonusInfo, bonus.pos,  norm, pt, -bonusBounce * normSpd);
                    solveImpulse(info, pos, spd, angSpd, bonusInfo, bonus.pos, ~norm, pt, frictCoeff * (relSpd % norm));
                }
                pos += 0.5 * depth * norm;  if(normSpd > -pickupSpeed)continue;

                switch(bonus.type)
                {
                case model::REPAIR_KIT:    score += repairScore * pow<repairPower>(1 - durability);  durability = 1;  break;
                case model::AMMO_CRATE:    ammoCount++;   break;
                case model::NITRO_BOOST:   score += nitroCost;  nitroCount++;  break;
                case model::OIL_CANISTER:  oilCount++;    break;
                case model::PURE_SCORE:    score += scoreBonus;  break;
                default:  break;
                }
                score += pickupScore;
            }
            if(time >= slidingEnd)for(auto &slick : tileMap.slicks[k])
            {
                if(!rect.checkPoint(slick.pos, slickRadius))continue;
                slidingEnd = min(time + slidingTime, slick.endTime);
                score -= slickPenalty;
            }
        }
        score -= distPenalty * pow<distPower>(1 - max(0.0, borderDist) / maxDist);
        score--;  return true;
    }

    bool activateNitro(int time)
    {
        if(!nitroCount || time < nitroEnd + nitroCooldown)return false;
        power = 1;  nitroEnd = time + nitroDuration;  nitroCount--;
        score -= nitroCost;  return true;
    }

    bool nextStep(const CarInfo &info, int time, int powerTarget, int turnTarget, bool brake)
    {
        if(time >= optLookahead)return false;

        // TODO: broken & sliding

        power += limit(powerTarget - power, powerChange);
        turn += limit(turnTarget - turn, turnChange);
        double curPower = power, frict = longFrict;
        if(time < nitroEnd)
        {
            power = 1;  curPower = nitroPower;
        }
        if(brake)
        {
            curPower = 0;  frict = crossFrict;
        }
        if(!update(info, time, curPower, turn, frict))return false;
        if(powerTarget < 1)score -= reversePenalty;

        Vec2D offs = pos * invTileSize;
        int x = int(offs.x), y = int(offs.y), k = y * mapLine + x;
        if(abs(offs.x - x - 0.5) > tileToggle || abs(offs.y - y - 0.5) > tileToggle)return true;
        unsigned cur = tileMap.distMap[waypoint][k];  if(cur + dist >= base)return true;
        dist = base - cur;  score += tileScore;  if(cur)return true;

        if(size_t(++waypoint) >= tileMap.waypoints.size())  // TODO: detect finish
        {
            nitroCount++;  ammoCount++;  oilCount++;  waypoint = 0;
        }
        base += tileMap.waypointDistMap(waypoint)[k];  return true;
    }

    static int classify(Vec2D vec)
    {
        int flags = (signbit(vec.x) ? 1 : 0) | (signbit(vec.y) ? 2 : 0);
        vec.x = abs(vec.x);  vec.y = abs(vec.y);
        if(vec.x < vec.y)
        {
            flags |= 4;  swap(vec.x, vec.y);
        }
        return vec.y < 0.4 * vec.x ? flags : flags | 8;
    }

    int classify() const
    {
        Vec2D offs = pos * invTileSize;  int x = int(offs.x), y = int(offs.y);
        return (y * mapLine + x) << 9 | (spd.sqr() > sqr(largeSpeed) ? 1 << 8 : 0) |
            classify(offs - Vec2D(x + 0.5, y + 0.5)) << 4 | classify(dir);
    }
};


enum EventType
{
    e_accel, e_reverse, e_brake, e_unbrake, e_left, e_center, e_right,
    e_nitro, e_end
};

struct Event
{
    int time;  EventType type;

    Event() = default;

    Event(int time_, EventType type_) : time(time_), type(type_)
    {
    }
};

enum CommandType
{
    c_stop,   // end program
    c_jump,   // relative jump to arg
    c_fork,   // fork to arg
    c_twait,  // wait arg ticks
    c_rwait,  // random wait of 1 to arg ticks
    c_cwait,  // wait for turn = 0, but at least arg ticks
    c_exec,   // execute event type arg
};

struct Command
{
    CommandType cmd;  int arg;
};

struct Move
{
    enum ToggleFlags
    {
        f_nitro = 1
    };

    int power, turn, flags;
    bool brake;

    Move() : power(1), turn(0), flags(0), brake(false)
    {
    }

    void update(EventType type)
    {
        switch(type)
        {
        case e_accel:    power = +1;        return;
        case e_reverse:  power = -1;        return;
        case e_brake:    brake = true;      return;
        case e_unbrake:  brake = false;     return;
        case e_left:     turn = -1;         return;
        case e_center:   turn =  0;         return;
        case e_right:    turn = +1;         return;
        case e_nitro:    flags |= f_nitro;  return;
        default:         assert(false);     return;
        }
    }

    bool nextStep(const CarInfo &info, CarState &state, int time)
    {
        if((flags & f_nitro) && !state.activateNitro(time))return false;
        flags = 0;  return state.nextStep(info, time, power, turn, brake);
    }

    template<typename T> bool nextStep(T &handler,
        const CarInfo &info, CarState &state, const vector<Event> &events, int time)
    {
        unsigned old = state.dist;  if(!nextStep(info, state, time))return false;
        return old == state.dist || handler.evaluate(info, state, events, time + 1);
    }

    static void execute(vector<Event> &events, model::Move &move)
    {
        Move cur;  int pos = 0;
        vector<Event> base;  base.reserve(events.size());  swap(base, events);
        for(; base[pos].time <= 0; pos++)cur.update(base[pos].type);
        move.setEnginePower(cur.power);  move.setWheelTurn(cur.turn);
        move.setBrake(cur.brake);  move.setUseNitro(cur.flags & f_nitro);

        for(cur.flags = 0; base[pos].time <= 1; pos++)cur.update(base[pos].type);
        events.emplace_back(0, EventType(e_center + cur.turn));
        if(cur.flags & f_nitro)events.emplace_back(0, e_nitro);
        else if(cur.power < 0)events.emplace_back(0, e_reverse);
        if(cur.brake)events.emplace_back(0, e_brake);

        for(; base[pos].time < infTime; pos++)events.emplace_back(base[pos].time - 1, base[pos].type);
        events.emplace_back(infTime, e_end);
    }
};

struct ProgramState : public Move
{
    vector<Event> events;
    int prevPower, prevTurn, turnEvent, turnEnd;
    bool prevBrake;

    ProgramState(const CarInfo &info, const CarState &state, vector<Event> &&base, int time) : events(base), turnEvent(-1)
    {
        int pos = 0;
        for(; events[pos].time < time; pos++)
        {
            update(events[pos].type);
            if(events[pos].type >= e_left && events[pos].type <= e_right)turnEvent = pos;
        }
        events.resize(pos);  flags = 0;

        prevPower = power;  prevBrake = brake;
        if(turn)
        {
            prevTurn = turn;  turnEnd = infTime;
        }
        else if(turnEvent < 0)
        {
            prevTurn = 0;  turnEvent = pos;
            events.emplace_back(turnEnd = time, e_center);
        }
        else
        {
            prevTurn = signbit(state.turn) ? 1 : -1;
            turnEnd = time + int(abs(state.turn) * invTurnChange + timeEps);
        }
    }

    void dumpEvents(CarState &state, int time)
    {
        if(prevPower != power)
        {
            prevPower = power;  events.emplace_back(time, power < 0 ? e_reverse : e_accel);
        }
        if(prevBrake != brake)
        {
            prevBrake = brake;  events.emplace_back(time, brake ? e_brake : e_unbrake);
        }
        if(turn)
        {
            if(prevTurn == -turn || time > turnEnd)
            {
                turnEvent = events.size();
                events.emplace_back(time, turn < 0 ? e_left : e_right);
            }
            else events[turnEvent].type = turn < 0 ? e_left : e_right;
            prevTurn = turn;  turnEnd = infTime;
        }
        else if(turnEnd == infTime)
        {
            if((prevTurn < 0 ? -state.turn : state.turn) > turnChange * timeEps)
            {
                prevTurn = -prevTurn;  turnEvent = events.size();
                events.emplace_back(time, e_center);
            }
            else events[turnEvent].type = e_center;
            turnEnd = time + int(abs(state.turn) * invTurnChange + timeEps);
        }
        if(flags & f_nitro)events.emplace_back(time, e_nitro);
    }

    template<typename T> bool process(T &handler, const CarInfo &info, CarState &state, int &time, int endTime)
    {
        if(time >= endTime)return true;  dumpEvents(state, time);
        while(time < endTime)if(!nextStep(handler, info, state, events, time++))return false;
        return true;
    }
};

template<typename T> void executePlan(T &handler,
    const CarInfo &info, CarState state, const vector<Event> &events, int endTime)
{
    Move cur;  int time = 0, pos = 0;
    while(time < endTime)
    {
        for(; events[pos].time <= time; pos++)cur.update(events[pos].type);
        if(cur.nextStep(handler, info, state, events, time++))continue;
        handler.finalize(events, 0, time, false);  return;
    }
    handler.finalize(events, 0, time, true);
}

template<typename T> void executeProgram(T &handler, const Command *program,
    const CarInfo &info, CarState state, ProgramState cur, int startTime, int endTime)
{
    int time = startTime;  bool completed = false;
    for(;;)
    {
        if(time >= endTime)
        {
            completed = true;  break;
        }

        switch(program->cmd)
        {
        case c_stop:
            if(!cur.process(handler, info, state, time, endTime))break;  continue;

        case c_jump:
            assert(program->arg);  program += program->arg;  continue;

        case c_fork:
            assert(program->arg);
            executeProgram(handler, program + program->arg, info, state, cur, time, endTime);
            program++;  continue;

        case c_exec:
            cur.update(EventType(program->arg));
            program++;  continue;

        case c_twait:
            assert(program->arg > 0);
            if(!cur.process(handler, info, state, time, min(endTime, time + program->arg)))break;
            program++;  continue;

        case c_rwait:
            assert(program->arg > 0);
            if(!cur.process(handler, info, state, time, min(endTime, time + rand() % program->arg + 1)))break;
            program++;  continue;

        case c_cwait:
            if(!cur.process(handler, info, state, time,
                min(endTime, time + max(program->arg, int(abs(state.turn) * invTurnChange + timeEps)))))break;
            program++;  continue;

        default:
            assert(false);
        }
        break;
    }
    handler.finalize(cur.events, startTime, time, completed);
}

template<typename T> void executeProgram(T &handler, const Command *program, int duration,
    const CarInfo &info, const CarState &state, vector<Event> &&events, int start)
{
    ProgramState cur(info, state, move(events), start);
    executeProgram(handler, program, info, state, cur, start, start + duration);
}


namespace Program
{
    struct Branch
    {
        CommandType cmd;
        const char *label;
    };

    struct Listing
    {
        vector<Command> data;
        map<string, int> labels;
        vector<int> targets;

        void append(const Command &cmd)
        {
            data.push_back(cmd);
        }

        void append(const Branch &cmd)
        {
            auto res = labels.insert(pair<string, int>(cmd.label, targets.size()));
            if(res.second)targets.push_back(-1);  int pos = res.first->second;
            if(cmd.cmd == c_stop)
            {
                assert(targets[pos] < 0);  targets[pos] = data.size();
            }
            else data.push_back({cmd.cmd, pos});
        }
    };

    Listing &&operator + (Listing &&program, const Command &cmd)
    {
        program.append(cmd);  return move(program);
    }

    Listing &&operator + (Listing &&program, const Branch &cmd)
    {
        program.append(cmd);  return move(program);
    }

    struct Bytecode
    {
        vector<Command> data;
        map<string, int> labels;

        Bytecode(Listing &&program) : data(program.data), labels(program.labels)
        {
            for(auto &label : labels)
            {
                label.second = program.targets[label.second];  assert(label.second >= 0);
            }
            for(size_t i = 0; i < data.size(); i++)
                if(data[i].cmd == c_jump || data[i].cmd == c_fork)
                {
                    data[i].arg = program.targets[data[i].arg] - i;
                }
        }

        const Command *operator [] (const char *label) const
        {
            auto res = labels.find(label);  assert(res != labels.end());  return &data[res->second];
        }
    };

    Listing start()
    {
        return Listing();
    }

    constexpr Command stop()
    {
        return {c_stop, 0};
    }

    constexpr Branch jump(const char *label)
    {
        return {c_jump, label};
    }

    constexpr Branch fork(const char *label)
    {
        return {c_fork, label};
    }

    constexpr Command twait(int duration)
    {
        return {c_twait, duration};
    }

    constexpr Command rwait(int duration)
    {
        return {c_rwait, duration};
    }

    constexpr Command cwait(int duration)
    {
        return {c_cwait, duration};
    }

    constexpr Command exec(EventType event)
    {
        return {c_exec, event};
    }

    Branch label(const char *label)
    {
        return {c_stop, label};
    }


    constexpr int optStep = 20, brakeTime = 20;

    const Bytecode bytecode = start() +

        label("> left-center") + cwait(1) + fork("right") + jump("> center") +
        label("> right-center") + cwait(1) + fork("left") +
        label("> center") + rwait(optStep) +
        label("loop_c") + fork("nitro") + fork("left") + fork("right") + twait(optStep) + jump("loop_c") +
        label("nitro") + exec(e_nitro) + stop() +

        label("left") + exec(e_left) + label("> left") + rwait(optStep) +
        label("loop_l") + fork("left-center") + fork("brake") + twait(optStep) + jump("loop_l") +
        label("left-center") + exec(e_center) + cwait(0) + fork("left-right") + stop() +
        label("left-right") + exec(e_right) + stop() +

        label("right") + exec(e_right) + label("> right") + rwait(optStep) +
        label("loop_r") + fork("right-center") + fork("brake") + twait(optStep) + jump("loop_r") +
        label("right-center") + exec(e_right) + cwait(0) + fork("right-left") + stop() +
        label("right-left") + exec(e_left) + stop() +

        label("brake") + exec(e_brake) + twait(brakeTime) + exec(e_unbrake) + stop() +

        label("> back") + rwait(optStep) +
        label("loop_b") + fork("back-left") + fork("back-right") + twait(optStep) + jump("loop_b") +
        label("back-left") + exec(e_left) + rwait(optStep) +
        label("loop_blr") + fork("back-left-right") + twait(optStep) + jump("loop_blr") +
        label("back-left-right") + exec(e_accel) + twait(30) + exec(e_right) + cwait(0) + fork("back-center") + stop() +
        label("back-right") + exec(e_right) + rwait(optStep) +
        label("loop_brl") + fork("back-right-left") + twait(optStep) + jump("loop_brl") +
        label("back-right-left") + exec(e_accel) + twait(30) + exec(e_left) + cwait(0) + fork("back-center") + stop() +
        label("back-center") + exec(e_accel) + exec(e_center) + stop();

}  // namespace Program

using Program::bytecode;

enum ProgramType
{
    p_center, p_left_center, p_right_center, p_left, p_right, p_back, p_count
};

const Command *const program[] =
{
    bytecode["> center"], bytecode["> left-center"], bytecode["> right-center"],
    bytecode["> left"], bytecode["> right"], bytecode["> back"],
};

const int programDuration[] = {100, 100, 100, 100, 100, 300};

ProgramType classifyState(const CarState &state, const ProgramState &cur, int time)
{
    if(cur.power < 1)return p_back;
    if(cur.turn)return cur.turn < 0 ? p_left : p_right;
    if(time > cur.turnEnd)return p_center;
    return cur.prevTurn < 0 ? p_left_center : p_right_center;
}


struct Position
{
    int time, leafDist;
    shared_ptr<Position> prev;
    CarState state;
    int eventCount;

    Position(shared_ptr<Position> &&prev_, int time_, const CarState &state_, int eventCount_) :
        time(time_), leafDist(0), prev(prev_), state(state_), eventCount(eventCount_)
    {
    }
};

struct Plan
{
    int time, tileDist, leafDist;
    shared_ptr<Position> last;
    vector<Event> events;
    double score;

    Plan() : time(0), tileDist(0), leafDist(0), score(-numeric_limits<double>::infinity())
    {
        events.emplace_back(0, e_center);
        events.emplace_back(infTime, e_end);
    }

    void set(const vector<Event> &base, const shared_ptr<Position> &pos)
    {
        last = pos;  time = pos->time;
        tileDist = pos->state.dist;  leafDist = pos->leafDist;
        score = pos->state.score;

        events.clear();  events.reserve(pos->eventCount + 1);
        events.insert(events.begin(), base.begin(), base.begin() + pos->eventCount);
        events.emplace_back(infTime, e_end);
    }

    void print() const
    {
        cout << "Path: ";
        constexpr char flags[] = "adbulcrn";
        for(const auto &evt : events)
            if(evt.type < e_end)cout << evt.time << flags[evt.type] << ' ';
            else break;
        cout << "| " << score << endl;
    }

    void execute(model::Move &move)
    {
        Move::execute(events, move);  last.reset();
    }
};

struct Optimizer
{
    Plan old;  int lastGood;
    unordered_map<int, Plan> best;
    shared_ptr<Position> last;
    int startLeafDist;
    vector<Plan> buf;

    Optimizer() : lastGood(numeric_limits<int>::min())
    {
    }

    bool evaluate(const CarInfo &info, const CarState &state, const vector<Event> &events, int time)
    {
        assert(state.dist <= optTileDist);
        last = make_shared<Position>(move(last), time, state, events.size());
        return state.dist < optTileDist;
    }

    void finalize(const vector<Event> &events, int startTime, int endTime, bool completed)
    {
        auto pos = &last;  int leafDist = startLeafDist;
        for(; (*pos)->time > startTime; pos = &(*pos)->prev, leafDist++)
        {
            (*pos)->leafDist = max(leafDist, (*pos)->leafDist);

            Plan &track = best[(*pos)->state.classify()];
            if((*pos)->state.score > track.score)track.set(events, *pos);
        }
        last = *pos;  pos = &last;
        for(; (*pos); pos = &(*pos)->prev, leafDist++)
            (*pos)->leafDist = max(leafDist, (*pos)->leafDist);
    }

    void executeProgram(const CarInfo &info, ProgramState &cur, ProgramType type)
    {
        assert(last && !last->time);
        ::executeProgram(*this, program[type], info, last->state, cur, 0, programDuration[type]);
    }

    void process(const model::Car &car)
    {
        const CarInfo &info = carInfo[car.getType()];

        best.clear();  startLeafDist = -optTileDist;
        assert(!last);  last = make_shared<Position>(move(last), 0, car, 0);
        executePlan(*this, info, last->state, old.events, old.time);

        startLeafDist = 0;
        ProgramState cur(info, last->state, {Event(infTime, e_end)}, 0);
        if(!cur.turnEnd)
        {
            cur.update(e_center);  executeProgram(info, cur, p_center);
            cur.update(e_left);    executeProgram(info, cur, p_left);
            cur.update(e_right);   executeProgram(info, cur, p_right);
        }
        else if(cur.prevTurn < 0)
        {
            cur.update(e_center);  executeProgram(info, cur, p_left_center);
            cur.update(e_left);    executeProgram(info, cur, p_left);
        }
        else
        {
            cur.update(e_center);  executeProgram(info, cur, p_right_center);
            cur.update(e_right);   executeProgram(info, cur, p_right);
        }
        cur.update(e_center);  cur.update(e_reverse);
        executeProgram(info, cur, p_back);

        for(;;)
        {
            buf.clear();
            for(auto &track : best)if(track.second.tileDist < optTileDist && track.second.leafDist < 2)
            {
                buf.push_back(track.second);  track.second.leafDist = numeric_limits<int>::max();
            }
            if(buf.empty())break;
            for(auto &track : buf)
            {
                last = track.last;
                ProgramState cur(info, last->state, move(track.events), last->time);
                ProgramType type = classifyState(last->state, cur, last->time);
                ::executeProgram(*this, program[type], info,
                    last->state, cur, last->time, last->time + programDuration[type]);
            }
        }
        last.reset();
    }

    void executeFallback(const model::Car &car, model::Move &move)
    {
        Vec2D pos(car.getX(), car.getY()), dir = sincos(car.getAngle());
        Vec2D spd(car.getSpeedX(), car.getSpeedY());

        Vec2D offs = pos * invTileSize;
        int x = int(offs.x), y = int(offs.y), k = y * mapLine + x;
        const vector<unsigned> &map = tileMap.distMap[car.getNextWaypointIndex()];

        unsigned dist = map[k];  Vec2D target;
        const int line = 2 * mapLine;  int p = 2 * k + line + 2;
        if(!tileMap.borders[p -    1] && map[k -       1] < dist)target = {-1, 0};
        if(!tileMap.borders[p +    1] && map[k +       1] < dist)target = {+1, 0};
        if(!tileMap.borders[p - line] && map[k - mapLine] < dist)target = {0, -1};
        if(!tileMap.borders[p +    0] && map[k + mapLine] < dist)target = {0, +1};

        offs -= Vec2D(x + 0.5, y + 0.5);  double dot = dir * target;
        constexpr double forward = sqrt(0.75), turnCoeff = sqrt(0.25);
        if(dot > forward)
        {
            move.setEnginePower(1);
            move.setWheelTurn(turnCoeff * ((dir + offs) % target));
            move.setBrake(spd * target < 0);  return;
        }
        else if(dot < -forward)
        {
            move.setEnginePower(-1);
            move.setWheelTurn(signbit(offs % target) ? 1 : -1);  return;
        }

        double shift = offs % target, spdProj = spd % target;
        bool flag = signbit(abs(shift) < 0.15 ? spdProj : -shift);
        move.setEnginePower(flag == signbit(dir % target) ? 1 : -1);
        move.setBrake((flag ? spdProj : -spdProj) > 0.1);
        move.setWheelTurn(flag ? -1 : 1);
    }

    void execute(const model::Car &car, model::Move &move)
    {
        Plan *sel = nullptr;
        double score = -numeric_limits<double>::infinity();
        for(auto &track : best)if(score < track.second.score)
        {
            score = track.second.score;  sel = &track.second;
        }
        if(sel)
        {
#ifdef PRINT_LOG
            sel->print();
#endif
            sel->execute(move);  swap(*sel, old);  lastGood = globalTick;
        }
        else
        {
#ifdef PRINT_LOG
            cout << "Path: NOT FOUND!!!" << endl;
#endif
            if(old.events.size() < 3 || globalTick > lastGood + 10)
            {
                executeFallback(car, move);  old = Plan();
            }
            else old.execute(move);
        }
    }
};

Optimizer optimizer;



void MyStrategy::move(const model::Car &self, const model::World &world, const model::Game &game, model::Move &move)
{
    if(globalTick != world.getTick())
    {
        if(globalTick < 0)
        {
            initConsts(game, world);  srand(game.getRandomSeed());
            tileMap.init(world);
        }
        globalTick = world.getTick();  tileMap.reset(world);
    }

    if(globalTick < game.getInitialFreezeDurationTicks())
    {
        move.setEnginePower(1);  return;
    }
    else if(self.isFinishedTrack())return;

    optimizer.process(self);
    optimizer.execute(self, move);

    /*
    move.setEnginePower(1);
    move.setWheelTurn(globalTick < 570 ? 0 : 1);
    move.setBrake(globalTick >= 600 && globalTick < 620);

    move.setEnginePower(globalTick < 220 ? 1 : -1);
    move.setWheelTurn(globalTick < 300 || globalTick >= 410 ? 0 : 0.1);
    move.setBrake(globalTick >= 410);

    move.setEnginePower(globalTick < 300 ? 1 : 0);
    move.setWheelTurn(globalTick >= 280 && globalTick < 300 ? 0.8827958 : 0);

    static double baseAngSpd;
    static double predAngle, predAngSpd;
    static Vec2D predPos, predSpd;

    Vec2D pos(self.getX(), self.getY());
    Vec2D spd(self.getSpeedX(), self.getSpeedY());
    double angle = self.getAngle();
    double angSpd = self.getAngularSpeed() - baseAngSpd;

    if(globalTick > game.getInitialFreezeDurationTicks())
    {
        Vec2D errPos = predPos - pos, errSpd = predSpd - spd;

        cout << globalTick << ' ';
        //cout << pos.x << ' ' << pos.y << ' ';
        //cout << spd.x << ' ' << spd.y << ' ';
        //cout << errPos.x << ' ' << errPos.y << ' ';
        cout << errSpd.x << ' ' << errSpd.y << ' ';
        cout << errPos.len() << ' ' << errSpd.len() << ' ';
        cout << (predAngle - angle) << ' ' << (predAngSpd - angSpd) << endl;
    }

    if(self.getDurability() < 0.999)exit(0);

    const auto &info = carInfo[self.getType()];
    double power = self.getEnginePower(), turn = self.getWheelTurn();
    power += limit(move.getEnginePower() - power, powerChange);
    turn += limit(move.getWheelTurn() - turn, turnChange);
    power *= power < 0 ? info.carReverse : info.carAccel;
    if(move.isBrake())power = 0;  unsigned consumed = 0;

    Vec2D dir = sincos(angle), accel = power * dir;
    double frict = move.isBrake() ? crossFrict : longFrict;
    double rot = carRotFactor * turn * (spd * dir);
    for(int i = 0; i < physIter; i++)
    {
        pos += spd * physDt;  spd += accel;  spd *= frictMul;
        spd -= limit(spd * dir, frict) * dir + limit(spd % dir, crossFrict) * ~dir;
        dir = sincos(angle += rot + angSpd * physDt);  angSpd *= rotFrictMul;
        angSpd -= limit(angSpd, angFrict);

        Quad rect(pos, dir, carHalfWidth, carHalfHeight);  Vec2D norm, pt;
        double depth = tileMap.collideBorder(rect, norm, pt);
        if(depth > 0)
        {
            cout << "Border collision: " << depth << ' ';
            cout << norm.x << ' ' << norm.y << ' ';
            cout << pt.x << ' ' << pt.y << endl;

            Vec2D relSpd = spd + (rot * physIter + angSpd) * ~(pt - pos);
            double normSpd = relSpd * norm;
            if(normSpd < 0)
            {
                double frictCoeff = carFrict * normSpd / sqrt(relSpd.sqr() + spdEps2);
                solveImpulse(info, pos, spd, angSpd, borderInfo, pt,  norm, pt, -carBounce * normSpd);
                solveImpulse(info, pos, spd, angSpd, borderInfo, pt, ~norm, pt, frictCoeff * (relSpd % norm));
            }
            pos += depth * norm;
        }

        Vec2D offs = pos * invTileSize;
        int k = int(offs.y) * mapLine + int(offs.x);
        for(auto &bonus : tileMap.bonuses[k])if((consumed & bonus.flag) != bonus.flag)
        {
            double depth = bonus.collide(rect, norm, pt);
            if(depth < 0)continue;  consumed |= bonus.flag;

            cout << "Bonus collision: " << depth << ' ';
            cout << norm.x << ' ' << norm.y << ' ';
            cout << pt.x << ' ' << pt.y << endl;

            Vec2D relSpd = spd + (rot * physIter + angSpd) * ~(pt - pos);
            double normSpd = relSpd * norm;
            if(normSpd < 0)
            {
                double frictCoeff = bonusFrict * normSpd / sqrt(relSpd.sqr() + spdEps2);
                solveImpulse(info, pos, spd, angSpd, bonusInfo, bonus.pos,  norm, pt, -bonusBounce * normSpd);
                solveImpulse(info, pos, spd, angSpd, bonusInfo, bonus.pos, ~norm, pt, frictCoeff * (relSpd % norm));
            }
            pos += 0.5 * depth * norm;
        }

        if(!consumed)for(auto &car : world.getCars())if(!car.isTeammate())
        {
            Quad rect1(car);
            double depth = rect1.collide(rect, norm, pt);
            if(depth < 0)continue;  consumed = -1;

            cout << "Car collision: " << depth << ' ';
            cout << norm.x << ' ' << norm.y << ' ';
            cout << pt.x << ' ' << pt.y << endl;

            Vec2D relSpd = spd + (rot * physIter + angSpd) * ~(pt - pos);
            double normSpd = relSpd * norm;
            if(normSpd < 0)
            {
                double frictCoeff = carFrict * normSpd / sqrt(relSpd.sqr() + spdEps2);
                solveImpulse(info, pos, spd, angSpd, carInfo[car.getType()], rect1.pos,  norm, pt, -carBounce * normSpd);
                solveImpulse(info, pos, spd, angSpd, carInfo[car.getType()], rect1.pos, ~norm, pt, frictCoeff * (relSpd % norm));
            }
            pos += 0.5 * depth * norm;
        }
    }
    baseAngSpd = rot * physIter;
    predAngle = angle;  predAngSpd = angSpd;
    predPos = pos;  predSpd = spd;
    */
}

MyStrategy::MyStrategy()
{
    cout << fixed << setprecision(5);
    //cout << scientific << setprecision(8);
}
