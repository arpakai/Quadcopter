#pragma once

#include "stm32f4xx_hal.h"

/// @brief Interface is nothing more than but signatures.
class IMPU
{
public:
    virtual ~IMPU() = default;
    virtual void _init() = 0;
    virtual void _tune_acc_gyro_offsets() = 0;
    virtual void _get_processed_accel_data(double& x, double& y, double& z) = 0;
    virtual void _get_processed_gyro_data(double& x, double& y, double& z, long&t) = 0;
    virtual void _get_processed_mag_data(double& x, double& y, double& z) = 0;
    virtual void _get_processed_all_data(double& ax, double& ay, double& az, 
                                            double& gx, double& gy, double& gz, long& gt,
                                            double& mx, double& my, double& mz) = 0;
    virtual void _set_computed_average_rpy(uint8_t num_samples, double& r, double& p, double& y) = 0;
    virtual void _get_roll_pitch_yaw(double& r, double& p, double& y) = 0;
    virtual void _print_roll_pitch_yaw(double& r, double& p, double& y, UART_HandleTypeDef* uart_handle) = 0;
};