#include "QuadcopterManager.h"

QuadcopterManager* mInstance = nullptr;

QuadcopterManager* QuadcopterManager::getInstance(){
    if (!mInstance){
        mInstance = new QuadcopterManager();
    }

    return mInstance;
}

QuadcopterManager::QuadcopterManager(){

}

