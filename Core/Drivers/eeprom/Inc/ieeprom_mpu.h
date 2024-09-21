#pragma once

class IeepromMpu
{
public:
    // Offset getters
    virtual int get_x_gyro_offset() = 0;
    virtual int get_y_gyro_offset() = 0;
    virtual int get_z_gyro_offset() = 0;
    virtual int get_x_accel_offset() = 0;
    virtual int get_y_accel_offset() = 0;
    virtual int get_z_accel_offset() = 0;

    // Offset setters
    virtual int set_x_gyro_offset(int os) = 0;
    virtual int set_y_gyro_offset(int os) = 0;
    virtual int set_z_gyro_offset(int os) = 0;
    virtual int set_x_accel_offset(int os) = 0;
    virtual int set_y_accel_offset(int os) = 0;
    virtual int set_z_accel_offset(int os) = 0;

    // Calib flag
    virtual bool get_calib_flag();
    virtual void set_calib_flag();      // Sets it to true value
    virtual void reset_calib_flag();    // Sets it to false value
private:

};