#ifndef TRAP_H
#define TRAP_H

#include "TrapCommon.h"
#include <iostream>
#include <string>
#include <mutex>
#include <thread>
#include <chrono>
#include <vector>
#include "unistd.h"

class TrapAgent
{
    private:
        mutex mMutex;
        int mTrapIndex = 0;
        
        uint16_t mTrapPort;

        uint8_t mTrapCount;
        vector<string> mTrapIPArray;
        vector<int> mSocketArray;
        vector<pthread_t> mFuncPth;
        vector<TrapStatusInfo> mTrapStatusInfoArray;
    public:
        explicit TrapAgent();

        static void *func(void*arg);

        static int connectToTrap(std::string ip,uint16_t port);

        void setTrapSwitch(int trapID,uint8_t type,bool value);      //开关控制
        void setTrapPower(int trapID,int value);        //功率控制
};

#endif