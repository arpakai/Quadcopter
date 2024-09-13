#pragma once

#include "stm32f4xx_hal.h"
#include "string.h"
#include "stdio.h"
 
// #include "cmsis_os.h"

class QuadcopterManager{
public:
    static QuadcopterManager* getInstance();

    void init();

private:
    static QuadcopterManager* mInstance;
    QuadcopterManager();
};
