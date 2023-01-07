#ifndef MAP_AGENT_H
#define MAP_AGENT_H
#include "MapCommon.h"
#include "TrapCommon.h"
#include "Trap.h"
#include <iostream>
#include <string>
#include <mutex>
#include <thread>
#include <chrono>
#include <vector>
#include <map>
#include <set>
#include "unistd.h"

struct ClientNetInfo
{
    int socket;
    struct sockaddr_in addr;

    bool operator<(const ClientNetInfo& rhs) const 
    {
        return socket < rhs.socket;
    }
};

class MapAgent
{
private:
    int mTcpServerSocket;
    struct sockaddr_in mTcpServerAddr;

    pthread_t mFuncPth = -1;

    std::mutex mMutex;
    std::set<ClientNetInfo> mClientSocketSet;
public:
    explicit MapAgent();

    void CreateTcpServer();

    void func_recv();

    bool checkPackage(char *buf,int buf_free,PackHead_t &head);

    static void* func_send(void *arg);

    void send_echo(char status, unsigned char cmd_id, char *info, int socketId);
};

#endif