#pragma once

#include "ieeprom_mpu.h"

class EepromManager //: public IeepromMpu
{
public:
    static EepromManager* _get_instance();

private:
    static EepromManager* _instance;
    EepromManager();
    ~EepromManager();

};