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

    Vec2D()
    {
    }

    constexpr Vec2D(const Vec2D &v) : x(v.x), y(v.y)
    {
    }

    constexpr Vec2D(double x_, double y_) : x(x_), y(y_)
    {
    }

    Vec2D &operator = (const Vec2D &v)
    {
        x = v.x;  y = v.y;  return *this;
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



constexpr double maxDist = 80;

constexpr int physIter = 10;
constexpr double physDt = 1.0 / physIter;

double tileSize, invTileSize, tileMargin;
int mapWidth, mapHeight, mapLine;

double carAccel[2], carReverse[2], frictMul, longFrict, crossFrict;
double carRotFactor, rotFrictMul;
double powerChange, turnChange;

int globalTick = -1;

void initConsts(const model::Game &game, const model::World &world)
{
    tileSize = game.getTrackTileSize();  invTileSize = 1 / tileSize;
    tileMargin = game.getTrackTileMargin();

    mapWidth = world.getWidth();  mapHeight = world.getHeight();
    mapLine = 2 * mapWidth + 2;

    carAccel[model::BUGGY] = game.getBuggyEngineForwardPower() / game.getBuggyMass() * physDt;
    carAccel[model::JEEP] = game.getJeepEngineForwardPower() / game.getJeepMass() * physDt;
    carReverse[model::BUGGY] = game.getBuggyEngineRearPower() / game.getBuggyMass() * physDt;
    carReverse[model::JEEP] = game.getJeepEngineRearPower() / game.getJeepMass() * physDt;
    frictMul = pow(1 - game.getCarMovementAirFrictionFactor(), physDt);
    longFrict = game.getCarLengthwiseMovementFrictionFactor() * physDt;
    crossFrict = game.getCarCrosswiseMovementFrictionFactor() * physDt;

    carRotFactor = game.getCarAngularSpeedFactor() * physDt;
    rotFrictMul = pow(1 - game.getCarRotationAirFrictionFactor(), physDt);

    powerChange = game.getCarEnginePowerChangePerTick();
    turnChange = game.getCarWheelTurnChangePerTick();
}


struct Quad
{
    Vec2D pos, dir;
    double half_w, half_h;

    Quad()
    {
    }

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
    vector<bool> barrier;

    void init(const model::World &world)
    {
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

        barrier.resize(mapLine * (mapHeight + 2), false);
        const vector<vector<model::TileType>> &map = world.getTilesXY();
        for(int x = 0; x < mapWidth; x++)for(int y = 0; y < mapWidth; y++)
        {
            int flags = tile[map[x][y]];
            int pos = (y + 1) * mapLine + 2 * (x + 1);
            if(flags & f_l)barrier[pos - 1] = true;
            if(flags & f_r)barrier[pos + 1] = true;
            if(flags & f_u)barrier[pos - mapLine] = true;
            if(flags & f_d)barrier[pos] = true;
        }

        /*
        for(int x = 0; x < mapWidth; x++)for(int y = 0; y < mapWidth; y++)
        {
            int flags = 0;
            int pos = (y + 1) * mapLine + 2 * (x + 1);
            if(barrier[pos - 1])flags |= f_l;
            if(barrier[pos + 1])flags |= f_r;
            if(barrier[pos - mapLine])flags |= f_u;
            if(barrier[pos])flags |= f_d;
            assert(flags == tile[map[x][y]]);
        }
        */
    }

    double calcDist(const Quad &quad) const
    {
        Vec2D offs = quad.pos * invTileSize + Vec2D(0.5, 0.5);
        int x = int(offs.x), y = int(offs.y);
        int px = y * mapLine + 2 * x, py = px + 1;
        int dx = 2, dy = mapLine;

        Quad check = quad;
        check.pos -= Vec2D(x, y) * tileSize;
        if(check.pos.x < 0)
        {
            px += dx;  dx = -dx;
            check.pos.x = -check.pos.x;
            check.dir.x = -check.dir.x;
        }
        if(check.pos.y < 0)
        {
            py += dy;  dy = -dy;
            check.pos.y = -check.pos.y;
            check.dir.y = -check.dir.y;
        }
        if(check.pos.x < check.pos.y)
        {
            swap(px, py);  swap(dx, dy);
            swap(check.pos.x, check.pos.y);
            swap(check.dir.x, check.dir.y);
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

        int flags = work[(barrier[py] ? 1 : 0) | (barrier[px + dx] ? 2 : 0) | (barrier[py + dy] ? 4 : 0)];

        double dist = maxDist;
        Vec2D rw = check.half_w * check.dir, rh = check.half_h * ~check.dir;
        Vec2D minX = check.pos + (rw.x < 0 ? rw : -rw) + (rh.x < 0 ? rh : -rh);
        Vec2D minY = check.pos + (rw.y < 0 ? rw : -rw) + (rh.y < 0 ? rh : -rh);
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
            double dot = abs(check.pos * check.dir), cross = abs(check.pos % check.dir);
            Vec2D d(max(0.0, dot - check.half_w), max(0.0, cross - check.half_h));
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

    int leftBroken, leftSliding, leftNitro;
    double power, powerTarget;
    double turn, turnTarget;
    bool brake;

    CarState(const model::Car &car) : pos(car.getX(), car.getY()), spd(car.getSpeedX(), car.getSpeedY()),
        dir(sincos(car.getAngle())), angle(car.getAngle()), angSpd(0)  // TODO: car.getAngularSpeed()
    {
    }

    void nextStep(model::CarType type, double power, double turn, double frict)
    {
        Vec2D accel = (power < 0 ? carReverse : carAccel)[type] * power * dir;
        double rot = carRotFactor * turn * (spd * dir);
        for(int i = 0; i < physIter; i++)
        {
            pos += spd * physDt;  spd += accel;  spd *= frictMul;
            spd -= limit(spd * dir, frict) * dir + limit(spd % dir, crossFrict) * ~dir;
            dir = sincos(angle += rot + angSpd * physDt);  angSpd *= rotFrictMul;
        }
    }

    void activateNitro()
    {
        power = powerTarget = 1;  leftNitro = 120;  // TODO: const
    }

    void nextStep(model::CarType type)
    {
        // TODO: broken & sliding

        nextStep(type, brake ? 0 : (leftNitro ? 2 : power), turn, brake ? crossFrict : longFrict);
    }
};


struct Track
{
    enum EventType
    {
        e_accel, e_reverse, e_nitro, e_brake, e_unbrake, e_left, e_center, e_right
    };

    struct Event
    {
        int time;  EventType event;
    };

    vector<Event> events;
    double score;
};



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

    if(self.getDurability() < 0.999)exit(0);
    move.setEnginePower(globalTick < 250 ? 1 : 0);
    move.setWheelTurn(globalTick < 250 ? 0.5 : -0.5);
    move.setBrake(globalTick > 250);

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
    Vec2D accel = carAccel[self.getType()] * power * dir;
    double frict = move.isBrake() ? crossFrict : longFrict;
    double rot = carRotFactor * turn * (spd * dir);
    for(int i = 0; i < physIter; i++)
    {
        pos += spd * physDt;  spd += accel;  spd *= frictMul;
        spd -= limit(spd * dir, frict) * dir + limit(spd % dir, crossFrict) * ~dir;
        dir = sincos(angle += rot + angSpd * physDt);  angSpd *= rotFrictMul;
    }
    baseAngSpd = rot * physIter;  predPos = pos;  predSpd = spd;
}

MyStrategy::MyStrategy()
{
    //cout << fixed << setprecision(5);
    cout << scientific << setprecision(8);
}
