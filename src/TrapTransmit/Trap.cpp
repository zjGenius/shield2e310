#include "Trap.h"

TrapAgent::TrapAgent()
{
    mTrapCount = GetIniKeyInt("Trap","TrapCount",_TRAP_INI_FILE);
    mTrapIPArray.resize(mTrapCount);
    mSocketArray.resize(mTrapCount);
    mFuncPth.resize(mTrapCount);
    mTrapStatusInfoArray.resize(mTrapCount);

    mTrapPort = GetIniKeyInt("Trap","TrapPort",_TRAP_INI_FILE);
    char ip_buf[256] = {0};
    GetIniKeyString(ip_buf,"Trap","TrapIP",_TRAP_INI_FILE);
    const char s[2] = ",";
	char *token;
	// 获取第一个子字符串
	token = strtok(ip_buf, s);
	/* 继续获取其他的子字符串 */
    for(int i = 0;i < mTrapCount;i++)
    {
        mTrapIPArray[i] = token;
        if(token == NULL) break;
        token = strtok(NULL, s);
    }
    for(int i = 0;i < mTrapCount;i++)
    {
        if (pthread_create(&mFuncPth[i], NULL, func, (void *)(this)) < 0)
        {
            char *error_message = strerror(errno);
            printf("pthread_create func(%d) error:'%s'\n", i,error_message);
            exit(-1);
        }        
    }
}

int TrapAgent::connectToTrap(std::string ip,uint16_t port)
{
    printf("ip = %s,port = %d\n",ip.c_str(),port);
    int sock_fd = socket(AF_INET,SOCK_STREAM, 0);
    struct sockaddr_in servaddr;
    memset(&servaddr, 0, sizeof(servaddr));
    servaddr.sin_family = AF_INET;
    servaddr.sin_port = htons(port);  ///服务器端口
    servaddr.sin_addr.s_addr = inet_addr(ip.c_str());
    printf("connect start,sock_fd = %d\n",sock_fd);
    while(connect(sock_fd, (struct sockaddr *)&servaddr, sizeof(servaddr)) < 0)
    {
        //jammer_open = js_waitconnect;
        printf("wait connect,error = %d\n",errno);
        sleep(1);
    }
    return sock_fd;
}

void *TrapAgent::func(void*arg)
{
    auto this_t = (TrapAgent*)arg;

    int trapID = 0;
    {
        std::unique_lock<mutex> autoMutex(this_t->mMutex);
        trapID = this_t->mTrapIndex;
        this_t->mTrapIndex++; 
    }

    int sock_t = TrapAgent::connectToTrap(this_t->mTrapIPArray[trapID],this_t->mTrapPort);
    {
        std::unique_lock<mutex> autoMutex(this_t->mMutex);
        this_t->mSocketArray[trapID] = sock_t;
    }

    int ret = 0;
    fd_set set,rset;
    FD_ZERO(&set);
    FD_ZERO(&rset);
    FD_SET(sock_t,&set);

    struct timeval tv;
    char recvjammerbuf[1024] = {0};
    while(1)
    {
        tv.tv_sec = 10;
        tv.tv_usec = 0;

        rset = set;
        ret = select(sock_t+1,&rset,NULL,NULL,&tv);
        if(ret <= 0)
        {
            printf("select err\n");
        }
        int r = recv(sock_t,recvjammerbuf,sizeof(TrapStatusInfo),0);
        if(r >= 10)
        {
            TrapStatusInfo tmp = *(TrapStatusInfo*)recvjammerbuf;
            if(tmp.packHead.orderNumber == 1 && tmp.packHead.orderType == 1)
            {
                {
                    std::unique_lock<mutex> autoMutex(this_t->mMutex);
                    this_t->mTrapStatusInfoArray[trapID] = tmp;
                }
                
                // info = tmp;
                // if(info.SimState == 3 && FirstConnectFlag == 1)
                // {
                //     if(able) {
                //         jammer_flag = jf_white;
                //     } else {
                //         jammer_flag = jf_manual;
                //     }
                    
                //     controlJammer(able);
                //     FirstConnectFlag = 0;
                // }
                // printf("\n\nSwitch : GPS = %d,GLO = %d,BD = %d\n",info.GPSFreqState,info.GLOFreqState,info.BDFreqState);
                // printf("Other : GPS = %d,GLO = %d,BD = %d\n",info.GpsEPHState,info.GloEPHState,info.BdEPHState);
                // printf("%d - %d,CurLong = %lf,CurLat = %lf,Height = %lf\n\n",info.curDate,info.curTime,info.curLon,info.curLat,info.curHeight);
                // if(info.GLOFreqState || info.GLOFreqState || info.BDFreqState)
                // {
                //     jammer_able = 1;
                // }
                // updateTime(info.curDate,info.curTime);
            }
        }
        else if(r <= 0)
        {
            break;
        }
    }  
}

void TrapAgent::setTrapSwitch(int trapID,uint8_t type, bool value)
{
    int sock_fd = mSocketArray[trapID];
    SwitchControlOrder switchControlOrder;
    switchControlOrder.packHead.head = JAMMER_CONTROL_FLAG;
    switchControlOrder.packHead.orderNumber = 2;
    switchControlOrder.packHead.orderType = 1;
    switchControlOrder.packHead.len = sizeof(SwitchControlOrder);
    switchControlOrder.type = type;
    switchControlOrder.enable = value;
    int r = send(sock_fd, &switchControlOrder, sizeof(SwitchControlOrder), 0);
    printf("openTrap\n\n");
    if (r < 0)
    {
        printf("openTrap failed\n");
    }
}
void TrapAgent::setTrapPower(int trapID,int value)
{
    int sock_fd = mSocketArray[trapID];
    PowerControlOrder powerControlOrder;
    powerControlOrder.packHead.head = JAMMER_CONTROL_FLAG;
    powerControlOrder.packHead.orderNumber = 2;
    powerControlOrder.packHead.orderType = 2;
    powerControlOrder.packHead.len = sizeof(PowerControlOrder);
    powerControlOrder.value = value;
    int r = send(sock_fd, &powerControlOrder, sizeof(PowerControlOrder), 0);
    printf("send control power:%d\n", value);
    if (r < 0)
    {
        printf("send data failed\n");
    }
}