#include "imu_manager.h"

IMUManager* _instance = nullptr;

IMUManager* IMUManager::_get_instance(){
    if (!_instance){
        _instance = new IMUManager();
    }

    return _instance;
}