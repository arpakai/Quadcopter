#pragma once 

class IMUManager
{
public: 
    static IMUManager& _get_instance();

private:
    IMUManager();

    static IMUManager* _instance;

};
