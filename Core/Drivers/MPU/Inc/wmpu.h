#pragma once

#include <vector>

#include "impu.h"
#include "MPUXX50.h"

class WMPU : public IMPU
{
public:
    WMPU(const std::vector<I2C_HandleTypeDef *>& pi2c, UART_HandleTypeDef *uart_handles = nullptr);
    virtual ~WMPU(); // Virtual destructor for proper cleanup

    // Inherited via IMPU
    virtual void _init() override;
    virtual void _tune_acc_gyro_offsets() override;
    virtual void _print_roll_pitch_yaw(UART_HandleTypeDef& uart_handle) override;
    virtual void _get_roll_pitch_yaw(double& r, double& p, double& y) override;
    template<typename Filter>
    void _get_roll_pitch_yaw(Filter& filter_data);

private:
    std::vector<MPUXX50*> MPUXX50s;

    I2C_HandleTypeDef *_pI2Cx;
    UART_HandleTypeDef *_pUARTx;

    size_t _num_devices;

    uint8_t _serial_buf[100];

    Attitude _attitude;
};

template<typename Filter>
void WMPU::_get_roll_pitch_yaw(Filter& filter_data)
{
    for (auto mpu : MPUXX50s) {
        mpu->_get_calculated_attitude<Filter>();
    }
}

// EOF: wmpu.h
