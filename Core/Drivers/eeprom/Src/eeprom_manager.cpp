#include "eeprom_manager.h"

eeprom_manager* _instance = nullptr;

eeprom_manager* eeprom_manager::_get_instance(){
    if(!_instance){
        _instance = new ieeprom_mpu();
    }
    return _instance;
}