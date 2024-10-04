#pragma once 

// Structs
struct RawData
{
    int16_t ax, ay, az, gx, gy, gz, gt, mx, my, mz;
};

struct ProcessedData
{
    double ax, ay, az, gx, gy, gz, gt, mx, my, mz;
};

struct GyroCal
{
    double x, y, z;
};

struct Mag
{
    double magX, magY, magZ;
};

struct EulerAngles {
    double roll, pitch, yaw;

    EulerAngles(double r = 0.0, double p = 0.0, double y = 0.0) : roll(r), pitch(p), yaw(y) {}
};

struct Attitude : public EulerAngles {
    using EulerAngles::EulerAngles; // Inherit constructors
};

struct kalmanf : public EulerAngles {
    using EulerAngles::EulerAngles; // Inherit constructors
};

struct madgwickf : public EulerAngles {
    using EulerAngles::EulerAngles; // Inherit constructors
};

struct complementaryf : public EulerAngles {
    using EulerAngles::EulerAngles; // Inherit constructors
};

struct quaternionf : public EulerAngles {
    using EulerAngles::EulerAngles;
};
