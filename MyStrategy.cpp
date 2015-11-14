#include "MyStrategy.h"

#include <cmath>
#include <cstdlib>
#include <cassert>
#include <algorithm>
#include <unordered_map>
#include <limits>
#include <set>

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


constexpr int distPower = 4;
constexpr double distPenalty = 10;
constexpr double maxDist = 32, epsDist = 1;
constexpr double timeEps = 0.001;

constexpr double tileBonus = 1000;
constexpr double pickupBonus = 5;
constexpr double scoreBonus = 500;
constexpr int repairPower = 4;
constexpr double repairBonus = 500;
constexpr double slickPenalty = 500;

constexpr int optStep = 16, brakeTime = 20;
constexpr int mnvDuration = 64, mnvTail = 64, stageCount = 3;
constexpr int infTime = numeric_limits<int>::max();


constexpr int physIter = 10;
constexpr double physDt = 1.0 / physIter;

double tileSize, invTileSize, tileMargin;
int mapWidth, mapHeight, mapLine;

int nitroDuration, nitroCooldown;
double nitroPower;

double carHalfWidth, carHalfHeight, carRadius;  CarInfo carInfo[2];
double frictMul, longFrict, crossFrict, carRotFactor, rotFrictMul;
double powerChange, invPowerChange, turnChange, invTurnChange;

double bonusHalfSize;  RectInfo bonusInfo;
double slickRadius;  int slidingTime;

int globalTick = -1, nextWaypoint = 0;

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
    nitroCooldown = game.getUseNitroCooldownTicks();
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
    carRotFactor = game.getCarAngularSpeedFactor() * physDt;
    rotFrictMul = pow(1 - game.getCarRotationAirFrictionFactor(), physDt);

    powerChange = game.getCarEnginePowerChangePerTick();  invPowerChange = 1 / powerChange;
    turnChange = game.getCarWheelTurnChangePerTick();  invTurnChange = 1 / turnChange;

    bonusHalfSize = game.getBonusSize() / 2;
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
        /*
        if(minDepth > 0)
        {
            if(!checkPoint(pt, 1))cout << "-------------------------------------------------> NOT IN FIRST" << endl;
            if(!q.checkPoint(pt, 1))cout << "-------------------------------------------------> NOT IN SECOND" << endl;
        }
        */
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
        };

        const vector<vector<model::TileType>> &map = world.getTilesXY();
        for(int x = 0; x < mapWidth; x++)for(int y = 0; y < mapWidth; y++)
        {
            int flags = tile[map[x][y]];
            int pos = (y + 1) * line + 2 * (x + 1);
            if(flags & f_l)borders[pos -    1] = true;
            if(flags & f_r)borders[pos +    1] = true;
            if(flags & f_u)borders[pos - line] = true;
            if(flags & f_d)borders[pos +    0] = true;
        }

        /*
        for(int x = 0; x < mapWidth; x++)for(int y = 0; y < mapWidth; y++)
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

    double borderDist(Vec2D pos, Vec2D dir, double hw, double hh) const
    {
        Vec2D offs = pos * invTileSize + Vec2D(0.5, 0.5);
        int x = int(offs.x), y = int(offs.y);

        int dx = 2, dy = 2 * mapLine;
        int px = x * dx + y * dy, py = px + 1;

        pos -= Vec2D(x, y) * tileSize;
        if(signbit(pos.x))
        {
            px += dx;  dx = -dx;
            pos.x = -pos.x;  dir.x = -dir.x;
        }
        if(signbit(pos.y))
        {
            py += dy;  dy = -dy;
            pos.y = -pos.y;  dir.y = -dir.y;
        }
        if(pos.x < pos.y)
        {
            swap(px, py);  swap(dx, dy);
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

        int flags = work[(borders[py] ? 1 : 0) | (borders[px + dx] ? 2 : 0) | (borders[py + dy] ? 4 : 0)];

        double dist = maxDist;
        Vec2D rw = hw * dir, rh = hh * ~dir;
        Vec2D minX = pos + (signbit(rw.x) ? rw : -rw) + (signbit(rh.x) ? rh : -rh);
        Vec2D minY = pos + (signbit(rw.y) ? rw : -rw) + (signbit(rh.y) ? rh : -rh);
        if(flags & f_h)
        {
            dist = min(dist, minY.y - tileMargin);
        }
        if(flags & (signbit(minX.y) ? f_v1 : f_v2))
        {
            dist = min(dist, minX.x - tileMargin);
        }
        if(flags & f_c)
        {
            Vec2D d1(max(0.0, tileMargin - minX.x), max(0.0, tileMargin - minX.y));
            Vec2D d2(max(0.0, tileMargin - minY.x), max(0.0, tileMargin - minY.y));
            dist = min(dist, tileMargin - sqrt(max(d1 * d1, d2 * d2)));
        }
        if(flags & f_r)
        {
            double dot = abs(pos * dir), crs = abs(pos % dir);
            Vec2D d(max(0.0, dot - hw), max(0.0, crs - hh));
            dist = min(dist, d.len() - tileMargin);
        }
        return dist;
    }
};

TileMap tileMap;


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

        int &next = nextWaypoint;  // TODO
        int kk = car.getNextWaypointY() * mapLine + car.getNextWaypointX();
        while(tileMap.waypoints[next] != kk)
            if(size_t(++next) >= tileMap.waypoints.size())next = 0;
        waypoint = next;

        Vec2D offs = pos * invTileSize;
        int k = int(offs.y) * mapLine + int(offs.x);
        base = dist = tileMap.waypointDistMap(waypoint)[k];
        consumed = 0;  score = 0;
    }

    double update(const CarInfo &info, int time, double power, double turn, double frict)
    {
        Vec2D accel = (signbit(power) ? info.carReverse : info.carAccel) * power * dir;
        double rot = carRotFactor * turn * (spd * dir), borderDist = maxDist;
        for(int i = 0; i < physIter; i++)
        {
            pos += spd * physDt;  spd += accel;  spd *= frictMul;
            spd -= limit(spd * dir, frict) * dir + limit(spd % dir, crossFrict) * ~dir;
            dir = sincos(angle += rot + angSpd * physDt);  angSpd *= rotFrictMul;

            Quad rect(pos, dir, carHalfWidth, carHalfHeight);
            borderDist = min(borderDist, tileMap.borderDist(pos, dir, rect.hw, rect.hh));

            Vec2D offs = pos * invTileSize;  int k = int(offs.y) * mapLine + int(offs.x);
            for(auto &bonus : tileMap.bonuses[k])if((consumed & bonus.flag) != bonus.flag)
            {
                Vec2D norm, pt;
                double depth = bonus.collide(rect, norm, pt);
                if(depth < 0)continue;  consumed |= bonus.flag;

                // TODO: resolve collision

                switch(bonus.type)
                {
                case model::REPAIR_KIT:    score += repairBonus * pow(1 - durability, repairPower);  break;
                case model::AMMO_CRATE:    ammoCount++;   break;
                case model::NITRO_BOOST:   nitroCount++;  break;
                case model::OIL_CANISTER:  oilCount++;    break;
                case model::PURE_SCORE:    score += scoreBonus;  break;
                default:  break;
                }
                score += pickupBonus;
            }
            if(time >= slidingEnd)for(auto &slick : tileMap.slicks[k])
            {
                if(!rect.checkPoint(slick.pos, slickRadius))continue;
                slidingEnd = min(time + slidingTime, slick.endTime);
                score -= slickPenalty;
            }
        }
        score -= distPenalty * pow(1 - borderDist / maxDist, distPower);  return borderDist;
    }

    bool activateNitro(int time)
    {
        if(!nitroCount || time < nitroEnd + nitroCooldown)return false;
        power = 1;  nitroEnd = time + nitroDuration;  nitroCount--;  return true;
    }

    bool nextStep(const CarInfo &info, int time, int powerTarget, int turnTarget, bool brake)
    {
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
        double brd = update(info, time, curPower, turn, frict);
        if(brd < (time < 20 ? -epsDist : epsDist))return false;  // TODO: ~~~

        Vec2D offs = pos * invTileSize;
        int k = int(offs.y) * mapLine + int(offs.x);
        unsigned cur = tileMap.distMap[waypoint][k];
        if(dist > cur)
        {
            dist = cur;  score += tileBonus;
            if(!dist)
            {
                if(size_t(++waypoint) >= tileMap.waypoints.size())waypoint = 0;
                int delta = tileMap.waypointDistMap(waypoint)[k];  // TODO: detect finish
                base += delta;  dist += delta;
            }
        }
        return true;
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
        return (y * mapLine + x) << 8 | classify(offs - Vec2D(x + 0.5, y + 0.5)) << 4 | classify(dir);
    }

    int distance() const
    {
        return base - dist;
    }
};


struct Maneuver
{
    int start, time1, time2, turn1, turn2;
};

template<typename T> void evaluatePath(const CarInfo &info, CarState &state, T &handler, const Maneuver &mnv, int time)
{
    int lim = mnv.start + mnvDuration;
    while(time < lim)if(!state.nextStep(info, time++, 1, mnv.turn2, false))return;
    CarState last = state;

    int dist = state.distance();  lim += mnvTail;
    do if(time >= lim || !state.nextStep(info, time++, 1, mnv.turn2, false))return;
    while(dist >= state.distance());
    handler.evaluate(mnv, last, time, state, dist = state.distance(), true);

    do if(time >= lim || !state.nextStep(info, time++, 1, mnv.turn2, false))return;
    while(dist >= state.distance());
    handler.evaluate(mnv, last, time, state, dist = state.distance(), false);
}

template<typename T> void generatePathsTail(const CarInfo &info, CarState state, T &handler, Maneuver &mnv)
{
    int brakeEnd = mnv.time2 + brakeTime;
    if(brakeEnd >= mnv.start + mnvDuration)
    {
        CarState state1 = state;  int time = mnv.time2;  mnv.turn2 = mnv.turn1;
        while(time < brakeEnd)if(!state1.nextStep(info, time++, 1, mnv.turn1, true))return;
        evaluatePath(info, state1, handler, mnv, brakeEnd);
    }
    int turnEnd = mnv.time2 + int(abs(state.turn) * invTurnChange + timeEps);  mnv.turn2 = 0;
    if(turnEnd >= mnv.start + mnvDuration)
    {
        evaluatePath(info, state, handler, mnv, mnv.time2);  return;
    }
    int time = mnv.time2;
    while(time < turnEnd)if(!state.nextStep(info, time++, 1, 0, false))return;
    CarState state1 = state;  evaluatePath(info, state,  handler, mnv, turnEnd);
    mnv.turn2 = -mnv.turn1;   evaluatePath(info, state1, handler, mnv, turnEnd);
}

template<typename T> void generatePathsTurn(const CarInfo &info, CarState state, T &handler, Maneuver &mnv)
{
    int mnvEnd = mnv.start + mnvDuration, time = mnv.time1;
    while(time < mnvEnd)
    {
        if(!state.nextStep(info, time++, 1, mnv.turn1, false))return;
        if((time + 2 * globalTick) % optStep)continue;  mnv.time2 = time;
        generatePathsTail(info, state, handler, mnv);
    }
    mnv.time2 = infTime;  mnv.turn2 = mnv.turn1;
    evaluatePath(info, state, handler, mnv, mnvEnd);
}

template<typename T> void generatePaths(const CarInfo &info, CarState state, T &handler, int time)
{
    Maneuver mnv;  mnv.start = mnv.time1 = time;
    int turnEnd = time + int(abs(state.turn) * invTurnChange + timeEps);
    mnv.turn1 = signbit(state.turn) ? -1 : 1;  generatePathsTurn(info, state, handler, mnv);
    while(time < turnEnd)if(!state.nextStep(info, time++, 1, 0, false))return;

    mnv.time1 = turnEnd;
    if(turnEnd > mnv.start + optStep / 2)generatePathsTurn(info, state, handler, mnv);
    mnv.turn1 = -mnv.turn1;  generatePathsTurn(info, state, handler, mnv);

    int mnvEnd = mnv.start + mnvDuration;
    while(time < mnvEnd)
    {
        if(!state.nextStep(info, time++, 1, 0, false))return;
        if(time % optStep)continue;  mnv.time1 = time;
        mnv.turn1 = -1;  generatePathsTurn(info, state, handler, mnv);
        mnv.turn1 = +1;  generatePathsTurn(info, state, handler, mnv);
    }
    mnv.time1 = mnv.time2 = infTime;  mnv.turn1 = mnv.turn2 = 0;
    evaluatePath(info, state, handler, mnv, mnvEnd);
}


struct Plan
{
    enum EventType
    {
        e_accel, e_reverse, e_brake, e_unbrake, e_left, e_center, e_right,
        e_nitro, e_end
    };

    enum ToggleFlags
    {
        f_nitro = 1
    };

    struct Event
    {
        int time;  EventType type;

        Event() = default;

        Event(int time_, EventType type_) : time(time_), type(type_)
        {
        }
    };

    struct Position
    {
        int event, power, turn;  bool brake;

        Position() : event(0), power(1), turn(0), brake(false)
        {
        }

        int update(EventType type)
        {
            switch(type)
            {
            case e_accel:    power = +1;     return 0;
            case e_reverse:  power = -1;     return 0;
            case e_brake:    brake = true;   return 0;
            case e_unbrake:  brake = false;  return 0;
            case e_left:     turn = -1;      return 0;
            case e_center:   turn =  0;      return 0;
            case e_right:    turn = +1;      return 0;

            case e_nitro:    return f_nitro;
            default:         return 0;
            }
        }
    };

    struct Manager
    {
        vector<Event> &events;
        Position prev, next;
        int time, flags;

        Manager(vector<Event> &events_) : events(events_), time(0), flags(0)
        {
            events.clear();
        }

        void execute(model::Move &move)
        {
            move.setEnginePower(next.power);  move.setWheelTurn(next.turn);
            move.setBrake(next.brake);  move.setUseNitro(flags & f_nitro);
            prev = Position();  flags = 0;
        }

        void dumpEvents(int nextTime)
        {
            if(time >= nextTime)return;

            constexpr EventType turns[] = {e_left, e_center, e_right};
            if(prev.power != next.power)events.emplace_back(time, next.power < 0 ? e_reverse : e_accel);
            if(prev.brake != next.brake)events.emplace_back(time, next.brake ? e_brake : e_unbrake);
            if(prev.turn != next.turn)events.emplace_back(time, turns[next.turn + 1]);
            if(flags & f_nitro)events.emplace_back(time, e_nitro);

            prev = next;  time = nextTime;  flags = 0;
        }

        void append(int eventTime, EventType type)
        {
            dumpEvents(eventTime);  flags |= next.update(type);
        }

        void append(const Maneuver &mnv)
        {
            dumpEvents(mnv.start);  next = Position();
            if(mnv.time1 >= infTime)return;  dumpEvents(mnv.time1);  next.turn = mnv.turn1;
            if(mnv.time2 >= infTime)return;  dumpEvents(mnv.time2);
            if(mnv.turn2 == mnv.turn1)  // brake
            {
                next.brake = true;  dumpEvents(mnv.time2 + brakeTime);  next.brake = false;
            }
            else next.turn = mnv.turn2;
        }

        void finalize()
        {
            dumpEvents(infTime);  events.emplace_back(infTime, e_end);
        }
    };

    vector<Event> events;
    CarState last;
    double score;

    Plan() : score(-numeric_limits<double>::infinity())
    {
        events.emplace_back(infTime, e_end);
    }

    void set(const Plan *prev, const Maneuver &mnv, const CarState &last_, double score_)
    {
        Manager mng(events);
        if(prev)for(const auto &evt : prev->events)
            if(evt.time < mnv.start)mng.append(evt.time, evt.type);
            else break;

        mng.append(mnv);  mng.finalize();
        last = last_;  score = score_;
    }

    bool nextStep(const CarInfo &info, CarState &state, Position &pos, int time) const
    {
        int flags = 0;
        for(; events[pos.event].time <= time; pos.event++)
            flags |= pos.update(events[pos.event].type);
        if(flags & f_nitro)state.activateNitro(time);
        return state.nextStep(info, time, pos.power, pos.turn, pos.brake);
    }

    void print() const
    {
        cout << "Path: ";
        constexpr char flags[] = "arbulcrn";
        for(const auto &evt : events)
            if(evt.type < e_end)cout << evt.time << flags[evt.type] << ' ';
            else break;
        cout << "| " << score << endl;
    }

    void execute(model::Move &move)
    {
        vector<Event> old;  swap(old, events);

        Manager mng(events);  int pos = 0;
        for(; old[pos].time <= 0; pos++)
            mng.append(0, old[pos].type);
        mng.execute(move);

        for(; old[pos].time < infTime; pos++)
            mng.append(old[pos].time - 1, old[pos].type);
        mng.finalize();
    }
};

struct Optimizer
{
    Plan old, *current;  int bestDist;
    unordered_map<int, Plan> prev, best, next;

    void reset(const CarInfo &info, Plan::Position &pos, int time)
    {
        bestDist = 0;  int lim = time + mnvDuration;
        while(time < lim)if(!old.nextStep(info, old.last, pos, time++))return;
        CarState state = old.last;  current = &old;

        Maneuver mnv;
        mnv.start = mnv.time1 = lim;  mnv.turn1 = pos.turn;
        mnv.time2 = infTime;  mnv.turn2 = 0;

        int dist = state.distance();  lim += mnvTail;
        do if(time >= lim || !state.nextStep(info, time++, 1, mnv.turn2, false))return;
        while(dist >= state.distance());
        evaluate(mnv, old.last, time, state, dist = state.distance(), true);

        do if(time >= lim || !state.nextStep(info, time++, 1, mnv.turn2, false))return;
        while(dist >= state.distance());
        evaluate(mnv, old.last, time, state, dist = state.distance(), false);
    }

    void evaluate(const Maneuver &mnv, const CarState &last, int time, const CarState &state, int dist, bool main)
    {
        if(dist < bestDist)return;
        if(main && dist > bestDist)
        {
            best.clear();
            if(dist == bestDist + 1)swap(best, next);
            else next.clear();  bestDist = dist;
        }

        double score = state.score - time;
        unordered_map<int, Plan> &cur = dist > bestDist ? next : best;
        Plan &track = cur[state.classify() & ~0x80];  if(!(score > track.score))return;
        track.set(current, mnv, last, score);
    }

    void process(const model::Car &car)
    {
        assert(!prev.size() && !best.size() && !next.size());

        const CarInfo &info = carInfo[car.getType()];
        CarState state(car);  old.last = state;  Plan::Position pos;
        reset(info, pos, 0);  current = nullptr;
        generatePaths(info, state, *this, 0);

        for(int i = 1; i < stageCount; i++)
        {
            swap(prev, best);  next.clear();  reset(info, pos, i * mnvDuration);
            for(auto &track : prev)
            {
                current = &track.second;
                generatePaths(info, current->last, *this, i * mnvDuration);
            }
            if(!best.size())
            {
                swap(prev, best);  break;
            }
            prev.clear();
        }
    }

    void execute(model::Move &move)
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
            sel->execute(move);  swap(*sel, old);
        }
        else
        {
#ifdef PRINT_LOG
            cout << "Path: NOT FOUND!!!" << endl;
#endif
            move.setEnginePower(-1);  move.setWheelTurn(0);
            move.setUseNitro(false);  move.setBrake(false);
            old = Plan();
        }
        best.clear();  next.clear();
    }
};

Optimizer optimizer;



void solveCollision(const RectInfo &info, const Vec2D &pos, Vec2D &spd, double &angSpd,
    const RectInfo &info1, const Vec2D &pos1, const Vec2D &norm, const Vec2D &pt, double coeff)
{
    double w = (pt - pos) % norm, w1 = (pt - pos1) % norm;
    double invEffMass = info.invMass + info1.invMass + w * w * info.invAngMass + w1 * w1 * info1.invAngMass;
    double impulse = coeff / invEffMass;  spd += impulse * info.invMass * norm;  angSpd += impulse * info.invAngMass * w;
}

void MyStrategy::move(const model::Car &self, const model::World &world, const model::Game &game, model::Move &move)
{
    if(globalTick != world.getTick())
    {
        if(!(globalTick = world.getTick()))
        {
            initConsts(game, world);  srand(game.getRandomSeed());
            tileMap.init(world);
        }

        tileMap.reset(world);
    }

    if(globalTick < game.getInitialFreezeDurationTicks())
    {
        move.setEnginePower(1);  return;
    }
    else if(self.isFinishedTrack())return;

    optimizer.process(self);
    optimizer.execute(move);

    /*
    move.setEnginePower(1);
    move.setWheelTurn(globalTick < 570 ? 0 : 1);
    move.setBrake(globalTick >= 600 && globalTick < 620);
    */

    /*
    move.setEnginePower(globalTick < 220 ? 1 : -1);
    move.setWheelTurn(globalTick < 300 || globalTick >= 410 ? 0 : 0.1);
    move.setBrake(globalTick >= 410);

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

    double power = self.getEnginePower(), turn = self.getWheelTurn();
    power += limit(move.getEnginePower() - power, powerChange);
    turn += limit(move.getWheelTurn() - turn, turnChange);
    power *= power < 0 ? carInfo[self.getType()].carReverse : carInfo[self.getType()].carAccel;
    if(move.isBrake())power = 0;  set<int> consumed;

    Vec2D dir = sincos(angle), accel = power * dir;
    double frict = move.isBrake() ? crossFrict : longFrict;
    double rot = carRotFactor * turn * (spd * dir);
    for(int i = 0; i < physIter; i++)
    {
        pos += spd * physDt;  spd += accel;  spd *= frictMul;
        spd -= limit(spd * dir, frict) * dir + limit(spd % dir, crossFrict) * ~dir;
        dir = sincos(angle += rot + angSpd * physDt);  angSpd *= rotFrictMul;

        double dist = tileMap.borderDist(pos, dir, carHalfWidth, carHalfHeight);
        if(dist < 0)
        {
            cout << "Border collision: " << -dist << endl;
        }

        Vec2D offs = pos * invTileSize;
        int k = int(offs.y) * mapLine + int(offs.x);
        for(auto &bonus : tileMap.bonuses[k])if(consumed.find(bonus.index) == consumed.end())
        {
            Vec2D norm, pt;
            double depth = bonus.collide(Quad(pos, dir, carHalfWidth, carHalfHeight), norm, pt);
            if(depth < 0)continue;  consumed.insert(bonus.index);

            cout << "Bonus collision: " << depth << ' ';
            cout << norm.x << ' ' << norm.y << ' ';
            cout << pt.x << ' ' << pt.y << endl;

            Vec2D v = spd + (rot * physIter + angSpd) * ~(pt - pos);  if(v * norm > 0)continue;
            solveCollision(carInfo[self.getType()], pos, spd, angSpd,
                bonusInfo, bonus.pos, norm, pt, -1.5 * (v * norm));
            solveCollision(carInfo[self.getType()], pos, spd, angSpd,
                bonusInfo, bonus.pos, ~norm, pt, sqrt(2.0 * 0.25) * (v * norm) * (v % norm) / v.len());
        }

        if(!consumed.size())for(auto &car : world.getCars())if(!car.isTeammate())
        {
            Vec2D norm, pt;  Quad rect(car);
            double depth = rect.collide(Quad(pos, dir, carHalfWidth, carHalfHeight), norm, pt);
            if(depth < 0)continue;  consumed.insert(-1);

            cout << "Car collision: " << depth << ' ';
            cout << norm.x << ' ' << norm.y << ' ';
            cout << pt.x << ' ' << pt.y << endl;

            Vec2D v = spd + (rot * physIter + angSpd) * ~(pt - pos);  if(v * norm > 0)continue;
            solveCollision(carInfo[self.getType()], pos, spd, angSpd,
                carInfo[car.getType()], rect.pos, norm, pt, -1.25 * (v * norm));
            solveCollision(carInfo[self.getType()], pos, spd, angSpd,
                carInfo[car.getType()], rect.pos, ~norm, pt, sqrt(2.0) * 0.25 * (v * norm) * (v % norm) / v.len());
            cout << ">>>>>>>>> " << v % norm << endl;
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
