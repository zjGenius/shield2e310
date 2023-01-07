#include "Map.h"

MapAgent::MapAgent()
{
    CreateTcpServer();
    if (pthread_create(&mFuncPth, NULL, func_send, (void *)(this)) < 0)
    {
        char *error_message = strerror(errno);
        printf("pthread_create func_send error:'%s'\n", error_message);
        exit(-1);
    }     
}

void MapAgent::CreateTcpServer()
{
    int                set = 1;
    mTcpServerSocket = socket(AF_INET, SOCK_STREAM, 0);
    if (mTcpServerSocket < 0)
    {
        printf("transmit - create recv_sock fail\n");
    }
    mTcpServerAddr.sin_family = AF_INET;
    mTcpServerAddr.sin_addr.s_addr = INADDR_ANY;
    mTcpServerAddr.sin_port = htons(RECV_PORT_NUM);
    setsockopt(mTcpServerSocket, SOL_SOCKET, SO_REUSEADDR, (void *)&set, sizeof(set));
    int a_return = bind(mTcpServerSocket, (struct sockaddr *)&mTcpServerAddr, sizeof(mTcpServerAddr));
    if (a_return < 0)
    {
        perror("bind");
    }
    listen(mTcpServerSocket, 10);
}

void* MapAgent::func_send(void *arg)
{
    auto this_t = (MapAgent*)arg;
    while(1)
    {
        {

            std::unique_lock<std::mutex> autoMutex(this_t->mMutex);
            for(std::set<ClientNetInfo>::iterator iter = this_t->mClientSocketSet.begin();
                            iter != this_t->mClientSocketSet.end();)
            {
                //发送消息
                // vector<TrapStatusInfo> TrapStatusInfoArray = TrapAgent::getVectorTrapStatusInfo();
                // for(uint8_t i = 0; i < TrapStatusInfoArray.size(); i++)
                // {
                //     int ret = send(iter->socket,&TrapStatusInfoArray[i],sizeof(TrapStatusInfo), 0);
                // }
            }
        }

        sleep(1);
    }
}

void MapAgent::func_recv()
{
    int                maxfd;
    fd_set             myset, rmyset;
    FD_ZERO(&myset);
    FD_SET(mTcpServerSocket, &myset);
    maxfd = mTcpServerSocket;

    while (1)
    {
        rmyset = myset;
        int ret = select(maxfd + 1, &rmyset, NULL, NULL, NULL);
        if (FD_ISSET(mTcpServerSocket, &rmyset))
        {
            if(mClientSocketSet.size() >= 3)
            {
                continue;
            }

            ClientNetInfo clientNetInfo;
            socklen_t client_len = sizeof(clientNetInfo.addr);
            clientNetInfo.socket = accept(mTcpServerSocket, (struct sockaddr *)&clientNetInfo.addr, &client_len);
            if (clientNetInfo.socket < 0)
            {
                perror("accept");
                continue;
            }
            printf("accept [%s] fd:%d\n", inet_ntoa(clientNetInfo.addr.sin_addr),clientNetInfo.socket);
            FD_SET(clientNetInfo.socket, &myset);
            if (clientNetInfo.socket > maxfd)
			{
                maxfd = clientNetInfo.socket;
			}
            std::unique_lock<std::mutex> autoMutex(mMutex);
            mClientSocketSet.insert(clientNetInfo);
			continue;
		}

        for(std::set<ClientNetInfo>::iterator iter = mClientSocketSet.begin();
                        iter != mClientSocketSet.end();)
        {
            if(FD_ISSET(iter->socket, &rmyset))
            {
                char buff[2048] = {0};
                memset(buff, 0, 2048);
                ret = recv(iter->socket, buff, 2048, 0);
                if (ret <= 0)
                {
                    std::unique_lock<std::mutex> autoMutex(mMutex);
                    FD_CLR(iter->socket, &myset);
                    if(iter->socket > 0)
                    {
                        close(iter->socket);
                    }
                    iter = mClientSocketSet.erase(iter);
                    continue;
                }

                int buf_tatal = ret;
                int buf_index = 0;
                int buf_free = buf_tatal;
                while(buf_free > 0)
                {
                    char *buf_p = buff + buf_index;

                    PackHead_t head = *(PackHead_t*)buf_p;
                    if(!checkPackage(buf_p,buf_free,head))
                    {
                        break;
                    }

                    switch(head.dataType)
                    {
                        case rdt_order:{
                            printf("接收到命令\n");
                            char command[128] = {0};
                            unsigned char cmd_id = *(buf_p + sizeof(struct PackHead_t));
                            memmove(command, buf_p + sizeof(PackHead_t) + 1, head.frameLen - sizeof(PackHead_t) - 6);
                            printf("command:%s\n", command);
                            if (strstr(command,"startmeasure") != NULL)
                            {
                                send_echo(ret == 0?1:0, cmd_id, "StartMeasure", iter->socket);
                                printf("StartMeasure\n");
                            }
                            else if(strstr(command,"stopmeasure") != NULL)
                            {
                                send_echo(ret == 0?1:0, cmd_id, "StopMeasure", iter->socket);
                                printf("StopMeasure\n");
                            }
                            else if(strstr(command,"resetdevice") != NULL)
                            {
                                send_echo(ret == 0?1:0, cmd_id, "ResetDevice", iter->socket);
                                printf("ResetDevice\n");
                            }
                            else if(strstr(command,"onwifi") != NULL)
                            {
                                send_echo(ret == 0?1:0, cmd_id, "OnWifi", iter->socket);
                                printf("OnWifi\n");
                            }
                            else if(strstr(command,"closewifi") != NULL)
                            {
                                send_echo(ret == 0?1:0, cmd_id, "CloseWifi", iter->socket);
                                printf("CloseWifi\n");
                            }
                        }break;
                        case rdt_file:{
                            FILE *fd = fopen(FREQLIST_FILE, "w+");
                            int len = *((int *)(buf_p + sizeof(struct PackHead_t) + 2));
                            if(len > 0)
                            {
                                fwrite(buf_p + sizeof(struct PackHead_t) + 6, 1, len - 1, fd);
                                fclose(fd);
                            }
                            printf("GET FILE OVER\n");
                        }break;
                        case rdt_timestamp:{    ///<授时
                            char time_flag = 0;
                            char time_buf[64] = {0};   
                            u_int64_t time_s = *(u_int64_t*)(buf_p + sizeof(struct PackHead_t) + 2);    //时间戳
                            time_flag = *(char*)(buf_p + sizeof(struct PackHead_t) + 1);                //时间选项
                            struct tm *time_now = localtime((time_t *)&time_s);
                            sprintf(time_buf,"sudo date -s '%d-%02d-%02d %02d:%02d:%02d'",1900 + time_now->tm_year,1 + time_now->tm_mon,
                                                            time_now->tm_mday,time_now->tm_hour,time_now->tm_min,time_now->tm_sec);
                            system(time_buf);
                            sprintf(time_buf,"%d",time_flag);
                            update_param_key(_TIME_INI_FILE,"UseTime",time_buf);
                            printf("SET TIME_SYSTEM SUCCESS\n");
                        }break;
                        case rdt_trapcontrol:{  ///<诱骗控制
                            TrapControlInfo *trapControlInfo = (TrapControlInfo *)(buf_p + sizeof(struct PackHead_t));
                            printf("trap switch = %d\n", trapControlInfo->switchState);
                            int ret = send(iter->socket, &trapControlInfo, sizeof(TrapControlInfo), 0);
                        }break;
                        case rdt_trappowercontrol:{ ///<功率控制     
                            TrapPowerInfo *trapPowerInfo = (TrapPowerInfo *)(buf_p + sizeof(struct PackHead_t));
                            printf("trap power = %d\n", trapPowerInfo->value);
                            int ret = send(iter->socket, &trapPowerInfo, sizeof(TrapPowerInfo), 0);
                        }break;
                    }

                    buf_index += head.frameLen;
                    buf_free = buf_tatal - buf_index;
                }
            }
            iter++;
        }

    }
}

bool MapAgent::checkPackage(char *buf,int buf_free,PackHead_t &head)
{
    if(buf_free < sizeof(PackHead_t))
    {
        printf("接收数据长度小于一个包头长度\n");
        return false;
    }
    head = *(PackHead_t*)buf;
    if(head.frameHead != RMTP_MESSAGE_START_FLAG)
    {
        printf("包头错误\n");
        return false;
    }
    if(buf_free < head.frameLen)
    {
        printf("接收数据长度长度小于一个包长度\n");
        return false;
    }

    PackTail_t tail = *(PackTail_t*)(buf + head.frameLen - sizeof(PackTail_t));
    if(tail.frameTail != RMTP_MESSAGE_END_FLAG)
    {
        printf("包尾错误\n");
        return false;
    }
    return true;
}

void MapAgent::send_echo(char status, unsigned char cmd_id, char *info, int socketId)
{
    struct PackHead_t echo_head;
    struct timeval  tv;
    struct timezone tz;
    struct tm *     time_now;
    char            echo_msg[1088] ={0};
    char            pack_check = 0;
    int             pack_end;
    int             echo_number = 0;
    char            send_buff[200];
    char            result[10];
    int             j;
    int             ret;

    memset(echo_msg, 0, sizeof(echo_msg));
    //回应包构造
    echo_head.frameHead = RMTP_MESSAGE_START_FLAG;
    echo_head.version = 0x100;
    pack_end = RMTP_MESSAGE_END_FLAG;
    memset(result, 0, sizeof(result));
    if (1 == status)
    {
        strncpy(result, "SUCCESSED", 9);
    }
    else
    {
        strncpy(result, "FAILURE", 7);
    }
    //填写时间戳
    gettimeofday(&tv, &tz);
    time_now = localtime(&tv.tv_sec);
    echo_head.year = (short)1900 + time_now->tm_year;
    echo_head.mon = (char)1 + time_now->tm_mon;
    echo_head.day = (char)time_now->tm_mday;
    echo_head.hour = (char)time_now->tm_hour;
    echo_head.minute = (char)time_now->tm_min;
    echo_head.sec = (char)time_now->tm_sec;
    echo_head.msec = tv.tv_usec / 1000;

    echo_number++;
    echo_head.frameCount = echo_number;
    echo_head.dataType = 6;
    sprintf(echo_msg, "RESULT:%s;Info:%s\n", result, info);
    echo_head.frameLen = sizeof(echo_head) + 6 + strlen(echo_msg);
    memmove(send_buff, &echo_head, sizeof(echo_head));
    memmove(send_buff + sizeof(echo_head), &cmd_id, 1);
    memmove(send_buff + sizeof(echo_head) + 1, &echo_msg, strlen(echo_msg));
    for (j = 4; j < (sizeof(echo_head) + strlen(echo_msg) + 1); j++)
    {
        pack_check += send_buff[j];
    }
    memmove(send_buff + sizeof(echo_head) + 1 + strlen(echo_msg), &pack_check, 1);    //校验
    memmove(send_buff + sizeof(echo_head) + 1 + strlen(echo_msg) + 1, &pack_end, 4);  //帧尾
    ret = send(socketId, send_buff, echo_head.frameLen, MSG_NOSIGNAL);
}
