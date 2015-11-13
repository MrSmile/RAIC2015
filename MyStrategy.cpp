#include "MyStrategy.h"

#include <cmath>
#include <cstdlib>
#include <cassert>
#include <algorithm>
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

inline constexpr Vec2D sincos(double angle)
{
    return Vec2D(cos(angle), sin(angle));
}

inline constexpr Vec2D sincos_fast(float angle)
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



constexpr double maxDist = 80, epsDist = 1;
constexpr double timeEps = 0.001;

constexpr int tileLookahead = 8;
constexpr double tileBonus = 1000;
constexpr int optStep = 5, optDuration = 100, optLookahead = 400;


constexpr int physIter = 10;
constexpr double physDt = 1.0 / physIter;

double tileSize, invTileSize, tileMargin;
int mapWidth, mapHeight, mapLine;

int nitroDuration, nitroCooldown;
double nitroPower;

struct CarInfo
{
    double carAccel, carReverse;
};

double carHalfWidth, carHalfHeight;  CarInfo carInfo[2];
double frictMul, longFrict, crossFrict, carRotFactor, rotFrictMul;
double powerChange, invPowerChange, turnChange, invTurnChange;

int globalTick = -1, nextWaypoint = 0;

void initConsts(const model::Game &game, const model::World &world)
{
    tileSize = game.getTrackTileSize();  invTileSize = 1 / tileSize;
    tileMargin = game.getTrackTileMargin();

    mapWidth = world.getWidth();  mapHeight = world.getHeight();  mapLine = mapWidth + 1;

    nitroDuration = game.getNitroDurationTicks();
    nitroCooldown = game.getUseNitroCooldownTicks();
    nitroPower = game.getNitroEnginePowerFactor();

    carHalfWidth = game.getCarWidth() / 2;  carHalfHeight = game.getCarHeight() / 2;
    carInfo[model::BUGGY].carAccel = game.getBuggyEngineForwardPower() / game.getBuggyMass() * physDt;
    carInfo[model::JEEP].carAccel = game.getJeepEngineForwardPower() / game.getJeepMass() * physDt;
    carInfo[model::BUGGY].carReverse = game.getBuggyEngineRearPower() / game.getBuggyMass() * physDt;
    carInfo[model::JEEP].carReverse = game.getJeepEngineRearPower() / game.getJeepMass() * physDt;

    frictMul = pow(1 - game.getCarMovementAirFrictionFactor(), physDt);
    longFrict = game.getCarLengthwiseMovementFrictionFactor() * physDt;
    crossFrict = game.getCarCrosswiseMovementFrictionFactor() * physDt;
    carRotFactor = game.getCarAngularSpeedFactor() * physDt;
    rotFrictMul = pow(1 - game.getCarRotationAirFrictionFactor(), physDt);

    powerChange = game.getCarEnginePowerChangePerTick();  invPowerChange = 1 / powerChange;
    turnChange = game.getCarWheelTurnChangePerTick();  invTurnChange = 1 / turnChange;
}


struct Quad
{
    Vec2D pos, dir;
    double half_w, half_h;

    Quad() = default;

    Quad(const Vec2D &pos_, const Vec2D &dir_, double hw, double hh) :
        pos(pos_), dir(dir_), half_w(hw), half_h(hh)
    {
    }

    Quad(const model::RectangularUnit &unit) : pos(unit.getX(), unit.getY()),
        dir(sincos(unit.getAngle())), half_w(unit.getWidth() / 2), half_h(unit.getHeight() / 2)
    {
    }

    Quad(const Vec2D &start, const Vec2D &delta, double size) : half_h(size / 2)
    {
        dir = delta / 2;  half_w = dir.len();  pos = start + dir;  dir /= half_w;
    }

    Quad move(const Vec2D &pt) const
    {
        return Quad(pt, dir, half_w, half_h);
    }

    bool checkPoint(const Vec2D &pt, double rad = 0) const
    {
        Vec2D dr = pt - pos;
        return abs(dr * dir) < half_w + rad && abs(dr % dir) < half_h + rad;
    }

    bool cross(const Quad &q) const
    {
        Vec2D dr = q.pos - pos;
        //if(dr.sqr() > sqr(half_w + half_h + q.half_w + q.half_h))return false;

        double dot = abs(dir * q.dir), cross = abs(dir % q.dir);
        if(!(abs(dr * dir) < half_w + q.half_w * dot + q.half_h * cross))return false;
        if(!(abs(dr % dir) < half_h + q.half_h * dot + q.half_w * cross))return false;
        if(!(abs(dr * q.dir) < q.half_w + half_w * dot + half_h * cross))return false;
        if(!(abs(dr % q.dir) < q.half_h + half_h * dot + half_w * cross))return false;
        return true;
    }
};


struct TileMap
{
    vector<bool> borders;
    vector<int> waypoints;
    vector<vector<unsigned>> distMap;

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

    double borderDist(Vec2D pos, Vec2D dir, double half_w, double half_h) const
    {
        Vec2D offs = pos * invTileSize + Vec2D(0.5, 0.5);
        int x = int(offs.x), y = int(offs.y);

        int dx = 2, dy = 2 * mapLine;
        int px = x * dx + y * dy, py = px + 1;

        pos -= Vec2D(x, y) * tileSize;
        if(pos.x < 0)
        {
            px += dx;  dx = -dx;
            pos.x = -pos.x;  dir.x = -dir.x;
        }
        if(pos.y < 0)
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
        Vec2D rw = half_w * dir, rh = half_h * ~dir;
        Vec2D minX = pos + (rw.x < 0 ? rw : -rw) + (rh.x < 0 ? rh : -rh);
        Vec2D minY = pos + (rw.y < 0 ? rw : -rw) + (rh.y < 0 ? rh : -rh);
        if(flags & f_h)
        {
            dist = min(dist, minY.y - tileMargin);
        }
        if(flags & (minX.y < 0 ? f_v1 : f_v2))
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
            double dot = abs(pos * dir), cross = abs(pos % dir);
            Vec2D d(max(0.0, dot - half_w), max(0.0, cross - half_h));
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
    double power, turn;

    int waypoint;
    unsigned base, dist;
    double lastTime, score;

    CarState(const model::Car &car) : pos(car.getX(), car.getY()), spd(car.getSpeedX(), car.getSpeedY()),
        dir(sincos(car.getAngle())), angle(car.getAngle()), angSpd(0)  // TODO: car.getAngularSpeed()
    {
        breakEnd = 0;  // TODO
        slidingEnd = car.getRemainingOiledTicks();
        nitroEnd = car.getRemainingNitroCooldownTicks() - nitroCooldown;
        nitroCount = car.getNitroChargeCount();
        ammoCount = car.getProjectileCount();
        oilCount = car.getOilCanisterCount();
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
        lastTime = 0;  score = 0;
    }

    double update(const CarInfo &info, double power, double turn, double frict)
    {
        Vec2D accel = (power < 0 ? info.carReverse : info.carAccel) * power * dir;
        double rot = carRotFactor * turn * (spd * dir), borderDist = maxDist;
        for(int i = 0; i < physIter; i++)
        {
            pos += spd * physDt;  spd += accel;  spd *= frictMul;
            spd -= limit(spd * dir, frict) * dir + limit(spd % dir, crossFrict) * ~dir;
            dir = sincos(angle += rot + angSpd * physDt);  angSpd *= rotFrictMul;

            borderDist = min(borderDist, tileMap.borderDist(pos, dir, carHalfWidth, carHalfHeight));
        }
        return borderDist;
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
        double eps = time < 20 ? -maxDist : epsDist;  // TODO
        if(update(info, curPower, turn, frict) < eps)return false;

        Vec2D offs = pos * invTileSize;
        int k = int(offs.y) * mapLine + int(offs.x);
        unsigned cur = tileMap.distMap[waypoint][k];
        if(dist > cur)
        {
            dist = cur;  lastTime = time + 1;  score += tileBonus;
            if(base - dist >= tileLookahead)return false;
            if(!dist)
            {
                if(size_t(++waypoint) >= tileMap.waypoints.size())waypoint = 0;
                int delta = tileMap.waypointDistMap(waypoint)[k];  // TODO: detect finish
                base += delta;  dist += delta;
            }
        }
        return true;
    }

    double calcScore() const
    {
        return score - lastTime;
    }
};


template<typename T> void generateTracksTail(const CarInfo &info, CarState state, T &handler, int time1, int time2, int maxTime, int turn)
{
    int turnTime = int(abs(state.turn) * invTurnChange + timeEps), lim = maxTime - time2;
    for(int i = 0; i < turnTime; i++)
    {
        if(i < lim && state.nextStep(info, time2 + i, 1, 0, false))continue;
        handler.evaluate(state, time1, time2, turn, 0);  return;
    }
    CarState next = state;
    for(int i = turnTime;; i++)
    {
        if(i < lim && state.nextStep(info, time2 + i, 1, 0, false))continue;
        handler.evaluate(state, time1, time2, turn, 0);  break;
    }
    for(int i = turnTime;; i++)
    {
        if(i < lim && next.nextStep(info, time2 + i, 1, -turn, false))continue;
        handler.evaluate(next, time1, time2, turn, -turn);  break;
    }
}

template<typename T> void generateTracksTurn(const CarInfo &info, CarState state, T &handler, int time1, int optTime, int maxTime, int turn)
{
    int lim = maxTime - time1, opt = optTime - time1;
    for(int i = 0;; i++)
    {
        if(i < lim && state.nextStep(info, time1 + i, 1, turn, false))
        {
            if((i + 1) % optStep || i >= opt)continue;
            generateTracksTail(info, state, handler, time1, time1 + i + 1, maxTime, turn);
            continue;
        }
        handler.evaluate(state, time1, maxTime, turn, 0);  return;
    }
}

template<typename T> void generateTracks(const CarInfo &info, CarState state, T &handler, int time)
{
    int turnTime = int(abs(state.turn) * invTurnChange + timeEps);
    int turn = state.turn < 0 ? -1 : 1, optTime = time + optDuration, maxTime = time + optLookahead;
    generateTracksTurn(info, state, handler, time, optTime, maxTime, turn);  time += turnTime;
    generateTracksTurn(info, state, handler, time, optTime, maxTime, -turn);
    if(turnTime > optStep / 4)
        generateTracksTurn(info, state, handler, time, optTime, maxTime, turn);

    int lim = maxTime - time;
    for(int i = 0;; i++)
    {
        if(i < lim && state.nextStep(info, time + i, 1, 0, false))
        {
            if((i + 1) % optStep || i >= optDuration)continue;
            generateTracksTurn(info, state, handler, time + i + 1, optTime, maxTime, -1);
            generateTracksTurn(info, state, handler, time + i + 1, optTime, maxTime, +1);
            continue;
        }
        handler.evaluate(state, maxTime, maxTime, 0, 0);  return;
    }
}


struct Track
{
    enum EventType
    {
        e_accel, e_reverse, e_nitro, e_brake, e_unbrake, e_left, e_center, e_right, e_end
    };

    struct Event
    {
        int time;  EventType type;

        Event() = default;

        Event(int time_, EventType type_) : time(time_), type(type_)
        {
        }
    };

    vector<Event> events;
    double score;

    Track()
    {
        events.emplace_back(optLookahead, e_end);
    }

    int process(const CarInfo &info, CarState &state) const
    {
        int power = 1, turn = 0;  bool brake = false;
        for(int time = 0, pos = 0; time < optLookahead; time++)
        {
            for(; events[pos].time <= time; pos++)switch(events[pos].type)
            {
            case e_accel:    power = +1;                 break;
            case e_reverse:  power = -1;                 break;
            case e_nitro:    state.activateNitro(time);  break;
            case e_brake:    brake = true;               break;
            case e_unbrake:  brake = false;              break;
            case e_left:     turn = -1;                  break;
            case e_center:   turn =  0;                  break;
            case e_right:    turn = +1;                  break;
            default:         return time;
            }
            if(!state.nextStep(info, time, power, turn, brake))return time;
        }
        return optLookahead;
    }

    void print() const
    {
        cout << "Path: ";
        constexpr char flags[] = "arnbulcr";
        for(const auto &event : events)
            if(event.type < e_end)cout << event.time << flags[event.type] << ' ';
            else break;
        cout << " | " << score << endl;
    }

    void execute(model::Move &move)
    {
        print();  // DEBUG

        int power = 1, turn = 0, pos = 0;
        bool nitro = false, brake = false;
        for(; events[pos].time <= 0; pos++)
        {
            switch(events[pos].type)
            {
            case e_accel:    power = +1;     continue;
            case e_reverse:  power = -1;     continue;
            case e_nitro:    nitro = true;   continue;
            case e_brake:    brake = true;   continue;
            case e_unbrake:  brake = false;  continue;
            case e_left:     turn = -1;      continue;
            case e_center:   turn =  0;      continue;
            case e_right:    turn = +1;      continue;
            default:         break;
            }
            break;
        }
        move.setEnginePower(power);  move.setWheelTurn(turn);
        move.setUseNitro(nitro);  move.setBrake(brake);

        int dst = 0;
        for(; events[pos].type < e_end; pos++)
            events[dst++] = Event(events[pos].time - 1, events[pos].type);
        events[dst++] = Event(optLookahead, e_end);  events.resize(dst);
    }
};

struct Optimizer
{
    Track best;

    void reset(const CarInfo &info, CarState state)
    {
        best.process(info, state);  best.score = state.calcScore();
    }

    void evaluate(const CarState &state, int time1, int time2, int turn1, int turn2)
    {
        double score = state.calcScore();  if(!(best.score < score))return;

        best.score = score;  best.events.clear();
        constexpr Track::EventType turns[] = {Track::e_left, Track::e_center, Track::e_right};
        if(time1 < optLookahead)best.events.emplace_back(time1, turns[turn1 + 1]);
        if(time2 < optLookahead)best.events.emplace_back(time2, turns[turn2 + 1]);
        best.events.emplace_back(optLookahead, Track::e_end);
    }

    void process(const model::Car &car)
    {
        const CarInfo &info = carInfo[car.getType()];
        CarState state(car);  reset(info, state);
        generateTracks(info, state, *this, 0);
    }

    void execute(model::Move &move)
    {
        best.execute(move);
    }
};

Optimizer optimizer;



void MyStrategy::move(const model::Car &self, const model::World &world, const model::Game &game, model::Move &move)
{
    if(globalTick != world.getTick())
    {
        if(!(globalTick = world.getTick()))
        {
            initConsts(game, world);  srand(game.getRandomSeed());
            tileMap.init(world);
        }

        // TODO
    }

    if(globalTick < game.getInitialFreezeDurationTicks())
    {
        move.setEnginePower(1);  return;
    }

    optimizer.process(self);
    optimizer.execute(move);

    /*
    if(self.getDurability() < 0.999)exit(0);

    move.setEnginePower(1);
    move.setWheelTurn(globalTick < 380 || globalTick > 420 ? 0 : 1);

    static double baseAngSpd;
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
        cout << errPos.len() << ' ' << errSpd.len() << endl;
    }

    double power = self.getEnginePower(), turn = self.getWheelTurn();
    power += limit(move.getEnginePower() - power, powerChange);
    turn += limit(move.getWheelTurn() - turn, turnChange);
    if(move.isBrake())power = 0;

    Vec2D dir = sincos(angle);
    Vec2D accel = carInfo[self.getType()].carAccel * power * dir;
    double frict = move.isBrake() ? crossFrict : longFrict;
    double rot = carRotFactor * turn * (spd * dir);
    for(int i = 0; i < physIter; i++)
    {
        pos += spd * physDt;  spd += accel;  spd *= frictMul;
        spd -= limit(spd * dir, frict) * dir + limit(spd % dir, crossFrict) * ~dir;
        dir = sincos(angle += rot + angSpd * physDt);  angSpd *= rotFrictMul;
    }
    baseAngSpd = rot * physIter;  predPos = pos;  predSpd = spd;
    */
}

MyStrategy::MyStrategy()
{
    cout << fixed << setprecision(5);
    //cout << scientific << setprecision(8);
}
