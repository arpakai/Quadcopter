#include "eeprom_manager.h"

EepromManager* _instance = nullptr;

EepromManager* EepromManager::_get_instance(){
    if(!_instance){
        _instance = new EepromManager();
    }
    return _instance;
}