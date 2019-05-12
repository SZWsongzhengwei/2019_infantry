#include "serial.h"

float_char transform;//transform hex and float

//互斥锁
serial_data serial;
pthread_mutex_t mutex;

void * thread_serial(void *arg)
{

   serial_data temp_serial;
   temp_serial.status = 0x00;
   temp_serial.color = 0x00;
   temp_serial.send_angle[0] = 0;
   temp_serial.send_angle[1] = 0;
   temp_serial.recive_angle[0] = 0;
   temp_serial.recive_angle[1] = 0;

   int fd = init_uart();//初始化串口

   char deal_buff[35];//存放要处理的字节
   int deal_num = 0;//存放要处理的字节数组里有效的字节个数
   char read_buff[20];//存放读到的字节
   int read_num = 0;//存放读到的字节个数
   char cut_buff[11];//临时储存被截断的数据字节
   int cut_num = 0;//被截断的字节个数
   int temp_num = 0;//储存字节中0xA5后面的字节的数量，包括0xA5，用于判断字节是否完整

   //储存提取的数据，用于校验
   char temp_status[4];
   char temp_color[4];
   char temp_angle[11];

   while(1)
   {
       //检查角度更新，若更新则需要发送
       pthread_mutex_lock(&mutex);
       if(temp_serial.send_angle[0] != serial.send_angle[0] || temp_serial.send_angle[1] != serial.send_angle[1])
       {
           temp_serial.send_angle[0] = serial.send_angle[0];
           temp_serial.send_angle[1] = serial.send_angle[1];
           pthread_mutex_unlock(&mutex);
           send_angle(fd, temp_serial.send_angle);
       }
       else
       {
           pthread_mutex_unlock(&mutex);
       }

       //将读到的字节存进缓冲区，如果缓冲区有遗留字节则直接在后面加上
       read_num = read(fd, read_buff, 15);
       for(int i = 0; i < read_num; i++)
       {
           deal_buff[deal_num + i] = read_buff[i];
           //cout<<hex<<(int)read_buff[i] <<endl;
       }
       deal_num += read_num;

       //当缓冲区的字节大于3时开始处理
       if(deal_num > 3)
       {
           for(int i = 0; i < deal_num; i++)
           {
               if(deal_buff[i] == 0xA6)//判断数据标志
               {
                   if((i+1) < deal_num)//如果0xA6不是最后一个字节，则开始判断，如果是最后一个字节则储存
                   {
                       temp_num = deal_num - i;//0xA5到数组末尾的字节数，包括0xA5
                       //开始判断数据类型ID
                       switch (deal_buff[i+1]) {
                       case 0x01:
                           //判断数据是否完整
                           if(temp_num > 3)
                           {
                               //将这4个字节提取出来
                               temp_status[0] = deal_buff[i];
                               temp_status[1] = deal_buff[i+1];
                               temp_status[2] = deal_buff[i+2];
                               temp_status[3] = deal_buff[i+3];
                               //开始校验
                               if(check_sum(temp_status, 4) == 1)
                               {
                                   temp_serial.status = temp_status[2];
                               }
                           }
                           else
                           {
                               //如果不足一帧数据放到cut_buff储存
                               for(int j = 0; j < temp_num; j++)
                               {
                                   cut_buff[j] = deal_buff[i+j];
                               }
                               cut_num = temp_num;
                           }
                           break;
                       case 0x02:
                           //判断数据是否完整
                           if(temp_num > 3)
                           {
                               //将这4个字节提取出来
                               temp_color[0] = deal_buff[i];
                               temp_color[1] = deal_buff[i+1];
                               temp_color[2] = deal_buff[i+2];
                               temp_color[3] = deal_buff[i+3];
                               //开始校验
                               if(check_sum(temp_color, 4) == 1)
                               {
                                   temp_serial.color = temp_color[2];
                               }
                           }
                           else
                           {
                               //如果不足一帧数据放到cut_buff储存
                               for(int j = 0; j < temp_num; j++)
                               {
                                   cut_buff[j] = deal_buff[i+j];
                               }
                               cut_num = temp_num;
                           }
                           break;
                       case 0x03:
                       //cout<<"03"<<endl;
                           //判断数据是否完整
                           if(temp_num > 10)
                           {
                               //将这11个字节提取出来
                               for(int j = 0; j < 11; j++)
                               {
                                   temp_angle[j] = deal_buff[i+j];
                               }

                               //开始校验
                               if(check_sum(temp_angle, 11) == 1)
                               {
                                   //将16进制转化为float
                                   transform.ch[0] = temp_angle[2];
                                   transform.ch[1] = temp_angle[3];
                                   transform.ch[2] = temp_angle[4];
                                   transform.ch[3] = temp_angle[5];
                                   temp_serial.recive_angle[0] = transform.fl;
                                   transform.ch[0] = temp_angle[6];
                                   transform.ch[1] = temp_angle[7];
                                   transform.ch[2] = temp_angle[8];
                                   transform.ch[3] = temp_angle[9];
                                   temp_serial.recive_angle[1] = transform.fl;

                                   //test 表明收到了角度数据

                               }
                           }
                           else
                           {
                               //如果不足一帧数据放到cut_buff储存
                               for(int j = 0; j < temp_num; j++)
                               {
                                   cut_buff[j] = deal_buff[i+j];
                               }
                               cut_num = temp_num;
                           }
                           break;
                       default:
                           break;
                       }
                   }
                   else
                   {
                       cut_buff[0] = deal_buff[i];
                       cut_num = 1;
                   }
               }

           }

           //更新serial结构体，存入刚读取的信息
           pthread_mutex_lock(&mutex);
           serial.status = temp_serial.status;
           serial.color = temp_serial.color;
           serial.recive_angle[0] = temp_serial.recive_angle[0];
           serial.recive_angle[1] = temp_serial.recive_angle[1];
           pthread_mutex_unlock(&mutex);

           //将被截断的数据储存到deal_buff中下一次处理
           for(int i = 0; i < cut_num; i++)
           {
               deal_buff[i] = cut_buff[i];
           }
           deal_num = cut_num;
           cut_num = 0;
       }
   }
   return (void*)0;
}

int init_uart()
{
   int fd = open(UART_DEVICE, O_RDWR|O_NOCTTY|O_NONBLOCK );
   if(-1 == fd)
   {
       cout<<"Can't open serial"<<endl;
       return -1;
   }

   struct termios  options;//创建一个修改保留串口数据的结构体
   tcgetattr(fd, &options);//用于获取与终端相关的参数，返回的结果保留在 termios 中
   tcflush(fd, TCIOFLUSH);//清空输入输出缓存区，TCIFLUSH输入缓存区   TCOFLUSH输出缓存区    TCIFLUSH输入输出缓存

   //set speed
   cfsetispeed(&options, B115200);//设置输入速度
   cfsetospeed(&options, B115200);//设置输出速度

   //set 数据位数
   options.c_cflag &= ~CSIZE;
   options.c_cflag |= CS8;

   //set parity
   options.c_cflag &= ~PARENB;   //禁止校验
   options.c_iflag &= ~INPCK;     //禁止输入校验

   //set 停止位
   options.c_cflag &= ~CSTOPB;//1

   //阻塞模式读操作下起作用
   options.c_cc[VTIME] =0; //  seconds 单位为百毫秒
   options.c_cc[VMIN] = 0;//等待读取字符的最小数量

   //modern
   options.c_lflag &=~ (ICANON|ECHO|ECHOE|ISIG);//取消规范模式 取消本地回显 收到信号字符不会处理
   options.c_cflag &=~(INLCR|ICRNL);//不转换回车和换行
   options.c_oflag &=~OPOST;//不处理直接输出
   options.c_oflag &=~(ONLCR|OCRNL);//不转换回车和换行
   options.c_iflag &=~(ICRNL|INLCR);//不转换回车和换行
   //options.c_cflag |=CLOCAL;

   tcsetattr(fd, TCSANOW, &options);

   return fd;
}

void updata_angle(float yaw, float pitch)
{
   pthread_mutex_lock(&mutex);
   serial.send_angle[0] = yaw;
   serial.send_angle[1] = pitch;

   pthread_mutex_unlock(&mutex);
}

void get_angle(float &yaw, float &pitch)
{
   pthread_mutex_lock(&mutex);
   yaw = serial.recive_angle[0];
   pitch = serial.recive_angle[1] ;
   pthread_mutex_unlock(&mutex);
}

char get_status()
{
   char status;
   pthread_mutex_lock(&mutex);
   status = serial.status;
   pthread_mutex_unlock(&mutex);
   return status;
}

char get_color()
{
   char color;
   pthread_mutex_lock(&mutex);
   color = serial.color;
   pthread_mutex_unlock(&mutex);
   return color;
}

bool check_sum(char data[], int lenth)
{
   int sum = 0;
   for(int i = 0; i < lenth - 1; i++)
   {
       sum +=data[i];
   }
   char check = sum&0x01FF;
   if(check == data[lenth - 1])
   {
       return true;
   }
   else
   {
       return false;
   }
}

void send_angle(int fd, float s_angle[2])
{

    cout<<"send...."<<endl;
   char send_char[11];

   send_char[0] = 0xA5;
   send_char[1] = 0x03;//angle坐标的ID

   transform.fl = s_angle[0];
   send_char[2] = transform.ch[0];
   send_char[3] = transform.ch[1];
   send_char[4] = transform.ch[2];
   send_char[5] = transform.ch[3];

   transform.fl = s_angle[1];
   send_char[6] = transform.ch[0];
   send_char[7] = transform.ch[1];
   send_char[8] = transform.ch[2];
   send_char[9] = transform.ch[3];

   //设置和校验位
   int sum = 0;
   for(int i = 0; i<10; i++)
   {
       sum +=send_char[i];
   }
   send_char[10] = sum&0xFF;

   write(fd, send_char, 11);

   //cout<<write_num<<endl;
}

