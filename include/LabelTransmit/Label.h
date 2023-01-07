#ifndef WHITE_CONTROL_H
#define WHITE_CONTROL_H
#include "UartControl.h"
#include "LabelCommon.h"
#include <iostream>
#include <string>
#include <mutex>
#include <thread>
#include <chrono>
#include <vector>
#include "unistd.h"
#include "paramRead.h"

class LabelAgent
{
private:
    UartControl *mUartControl;

    std::mutex mMutex;
    time_t mUpdateStamp;
    WhiteStateToServer mWhiteState;

    pthread_t mFuncPth = -1;
public:
    explicit LabelAgent();

    void dealData(char *buf,int size);
    bool auth(const WhiteListData &data);

    WhiteStateToServer getWhiteState();

    static void *func(void*arg);
};

#endif