#include "UartControl.h"

UartControl::UartControl(char *uartName,int baudrate)
{
    portinfo_t portinfo = {
        '0',   // print prompt after receiving
        baudrate,  // baudrate: 9600
        '8',   // databit: 8
        '0',   // debug: off
        '0',   // echo: off
        '0',   // flow control: software
        '0',   // default tty: COM1
        '0',   // parity: none
        '1',   // stopbit: 1
        0      // reserved
    };

    strcpy(mUartName,uartName);
    printf("\n uart:%s baudrate:%d\n", uartName, portinfo.baudrate);

    PortOpen(&portinfo, uartName);
    if(mFdcom < 0)
    {
        printf("Error: open serial port error.\n");
        mFdcom = -1;
        return;
    }
    PortSet(&portinfo);
}

/*******************************************
 *  波特率转换函数（请确认是否正确）
********************************************/
int UartControl::convbaud(unsigned long int baudrate)
{
    switch (baudrate)
    {
    case 2400:
        return B2400;
    case 4800:
        return B4800;
    case 9600:
        return B9600;
    case 19200:
        return B19200;
    case 38400:
        return B38400;
    case 57600:
        return B57600;
    case 115200:
        return B115200;
    default:
        return B9600;
    }
}

/*******************************************
 *  Setup comm attr
 *  mFdcom: 串口文件描述符，pportinfo: 待设置的端口信息（请确认）
 *
********************************************/
int UartControl::PortSet(const pportinfo_t pportinfo)
{
    struct termios termios_old, termios_new;
    int            baudrate, tmp;
    char           databit, stopbit, parity, fctl;

    bzero(&termios_old, sizeof(termios_old));
    bzero(&termios_new, sizeof(termios_new));
    cfmakeraw(&termios_new);
    tcgetattr(mFdcom, &termios_old);  // get the serial port attributions
    /*------------设置端口属性----------------*/
    // baudrates
    baudrate = convbaud(pportinfo->baudrate);
    cfsetispeed(&termios_new, baudrate);  //填入串口输入端的波特率
    cfsetospeed(&termios_new, baudrate);  //填入串口输出端的波特率
    termios_new.c_cflag |= CLOCAL;        //控制模式，保证程序不会成为端口的占有者
    termios_new.c_cflag |= CREAD;         //控制模式，使能端口读取输入的数据

    // 控制模式，flow control
    fctl = pportinfo->fctl;
    switch (fctl)
    {
    case '0':
    {
        termios_new.c_cflag &= ~CRTSCTS;  // no flow control
    }
    break;
    case '1':
    {
        termios_new.c_cflag |= CRTSCTS;  // hardware flow control
    }
    break;
    case '2':
    {
        termios_new.c_iflag |= IXON | IXOFF | IXANY;  // software flow control
    }
    break;
    }

    //控制模式，data bits
    termios_new.c_cflag &= ~CSIZE;  //控制模式，屏蔽字符大小位
    databit = pportinfo->databit;
    switch (databit)
    {
    case '5':
        termios_new.c_cflag |= CS5;
    case '6':
        termios_new.c_cflag |= CS6;
    case '7':
        termios_new.c_cflag |= CS7;
    default:
        termios_new.c_cflag |= CS8;
    }

    //控制模式 parity check
    parity = pportinfo->parity;
    switch (parity)
    {
    case '0':
    {
        termios_new.c_cflag &= ~PARENB;  // no parity check
    }
    break;
    case '1':
    {
        termios_new.c_cflag |= PARENB;  // odd check
        termios_new.c_cflag &= ~PARODD;
    }
    break;
    case '2':
    {
        termios_new.c_cflag |= PARENB;  // even check
        termios_new.c_cflag |= PARODD;
    }
    break;
    }

    //控制模式，stop bits
    stopbit = pportinfo->stopbit;
    if (stopbit == '2')
    {
        termios_new.c_cflag |= CSTOPB;  // 2 stop bits
    }
    else
    {
        termios_new.c_cflag &= ~CSTOPB;  // 1 stop bits
    }

    // other attributions default
    termios_new.c_oflag &= ~OPOST;  //输出模式，原始数据输出
    termios_new.c_cc[VMIN] = 1;     //控制字符, 所要读取字符的最小数量
    termios_new.c_cc[VTIME] = 1;    //控制字符, 读取第一个字符的等待时间    unit: (1/10)second

    tcflush(mFdcom, TCIFLUSH);                       //溢出的数据可以接收，但不读
    tmp = tcsetattr(mFdcom, TCSANOW, &termios_new);  //设置新属性，TCSANOW：所有改变立即生效    tcgetattr(fdcom, &termios_old);
    return (tmp);
}

/*******************************************
 *  Open serial port
 *  tty: 端口号 ttyS0, ttyS1, ....
 *  返回值为串口文件描述符
********************************************/
int UartControl::PortOpen(pportinfo_t pportinfo, char *ptty)
{
    char cmd[256] = {0};
    sprintf(cmd,"sudo chmod 777 %s",ptty);
    system("./getsudo.sh");
    system(cmd);
    // ptty = get_ptty(pportinfo);
    // mFdcom = open(ptty, O_RDWR | O_NOCTTY | O_NONBLOCK | O_NDELAY);
    mFdcom = open(ptty, O_RDWR | O_NOCTTY | O_NONBLOCK);
    return (mFdcom);
}

/*******************************************
 *  Close serial port
********************************************/
void UartControl::PortClose()
{
    close(mFdcom);
}

/********************************************
 *  send data
 *  fdcom: 串口描述符，data: 待发送数据，datalen: 数据长度
 *  返回实际发送长度
*********************************************/
int UartControl::PortSend(char *data, int datalen)
{
    int len = 0;

    len = write(mFdcom, data, datalen);  //实际写入的长度
    // printf("uart write len = %d\n", len);
    if (len == datalen)
    {
        return (len);
    }
    else
    {
        tcflush(mFdcom, TCOFLUSH);
        return -1;
    }
}

/*******************************************
 *  receive data
 *  返回实际读入的字节数
 *
********************************************/
int UartControl::PortRecv(char *data, int datalen)
{
    int            readlen, fs_sel;
    fd_set         fs_read;
    struct timeval tv_timeout;

    FD_ZERO(&fs_read);
    FD_SET(mFdcom, &fs_read);
    tv_timeout.tv_sec = 2;
    tv_timeout.tv_usec = 0;
	int recv_len = 0;
    fs_sel = select(mFdcom + 1, &fs_read, NULL, NULL, &tv_timeout);
    if (fs_sel)
    {
        readlen = read(mFdcom, data+recv_len, datalen);
        recv_len = recv_len+readlen;
    	//readlen = 0 ;
    	while(1)
    	{
        	readlen = read(mFdcom, data+recv_len, datalen);
        	if(readlen <= 0)
        	{
        		tv_timeout.tv_sec = 0;
    			tv_timeout.tv_usec = 50000;
                fs_sel = select(mFdcom + 1, &fs_read, NULL, NULL, &tv_timeout);
                if(fs_sel <= 0 ) 
                {
                    break;      	
                }
        		else
        		{
        			continue;
        		}
        	}
        	recv_len = recv_len+readlen;
            //printf("recv_len = %d\n",recv_len);
            if(recv_len > 0)
            {
                break;              
            }
        }
        return recv_len;
    }
    else
    {
        return -1;
    }
    return 0;
}