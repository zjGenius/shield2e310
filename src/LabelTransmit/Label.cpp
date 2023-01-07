#include "Label.h"

LabelAgent::LabelAgent()
{
    char uartName[64] = {0};
    GetIniKeyString(uartName,"White","UartName",_TRAP_INI_FILE); 
    int baudrate = GetIniKeyInt("White","Baudrate",_TRAP_INI_FILE);

    mUartControl = new UartControl(uartName,baudrate);

    if (pthread_create(&mFuncPth, NULL, func, (void *)(this)) < 0)
    {
        char *error_message = strerror(errno);
        printf("pthread_create func error:'%s'\n", error_message);
        exit(-1);
    }  
}

void *LabelAgent::func(void*arg)
{
    auto this_t = (LabelAgent*)arg;

    int       ret, std;

    int SendLen = 0;

    char buf[32] = { 0 };
    char recv_buf[BUF_SIZE] = { 0 };
    char all_recv_buf[BUF_SIZE] = { 0 };
    int all_index = 0;
    //获取与之连接的模块名
    while (1)
    {
        if(this_t->mUartControl->fdValid() == false) {
            printf("Failed to open WiFi serial port. Please check the serial port and restart!\n");
            sleep(1);
            continue;
        }
        int num = 0;
        int x = this_t->mUartControl->PortRecv(recv_buf, BUF_SIZE);
        if(x == -1) {
            printf("uart not data recv!retun x = %d\n",x);
            continue;
        }
        char *allP = all_recv_buf;
        memcpy(allP + all_index,recv_buf,x);
        all_index += x;

        char *p = allP;
        int startIndex = -1,endIndex = -1;
        for(int i = 0;i < all_index - 4;i++)
        {
            int magic = *(int *)p;
        	if(magic == 0xEEEEEEEE) {
                num ++; 
                if(num == 1) {
                    startIndex = i;
                } else if(num >= 2) {
                    endIndex = i;
                }           
            }
            p++;
        }
        // printf("startIndex = %d,endIndex = %d\n",startIndex,endIndex);
        if(startIndex != -1 && endIndex != -1)
        {
            char sendBuf[BUF_SIZE] = {0};
            memcpy(sendBuf,allP + startIndex,endIndex - startIndex);
            this_t->dealData(sendBuf,endIndex - startIndex);
            unsigned char recvBufTmp[BUF_SIZE] = {0};
            memcpy(recvBufTmp,allP + endIndex,BUF_SIZE - endIndex);
            memcpy(all_recv_buf,recvBufTmp,BUF_SIZE);
            all_index -= endIndex;
        }
    }

}

/*******************************************
 *  白名单协议数据处理函数
********************************************/
void LabelAgent::dealData(char *buf,int size)
{
    int index = 0;
    while(index + sizeof(WhiteListData) <= size)
    {
        WhiteListData data = *(WhiteListData*)(buf + index);
        printf("head = %x,packCount = %d,flag = %d,tail %x,year %d,mon %d,day %d,hour %d,min %d,sec %d\n",
            data.head,data.packetNum,data.whiteFlag,data.tail,
            data.year,data.mon,data.day,data.hour,data.min,data.sec);
        
        {
            std::unique_lock<std::mutex> autoMutex(mMutex);
            mUpdateStamp = time(NULL);
            mWhiteState.id = 1;
            if(auth(data))
            {
                mWhiteState.state = data.whiteFlag?0:1;            
            }
            else
            {
                mWhiteState.state = 1;
            }            
        }
        index += sizeof(WhiteListData);
    }
}
/*******************************************
 *  白名单模块认证
********************************************/
bool LabelAgent::auth(const WhiteListData &data)
{
    time_t stamp = time(NULL);
    tm *p_tm;
    p_tm = localtime(&stamp);
    int d_sec_module = data.hour * 3600 + data.min * 60 + data.sec;
    int d_sec_local = p_tm->tm_hour * 3600 + p_tm->tm_min * 60 + p_tm->tm_sec;
    if(p_tm->tm_year == data.year && p_tm->tm_mon == data.mon && p_tm->tm_mday == data.day && (abs(d_sec_module - d_sec_local) < 3600))
    {
        return true;
    }
    return false;
}

WhiteStateToServer LabelAgent::getWhiteState()
{
    WhiteStateToServer whiteStateTemp = mWhiteState;
    if(time(NULL) - mUpdateStamp > 10)
    {
        whiteStateTemp.state = -1;
    }
    return whiteStateTemp;
}