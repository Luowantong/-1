#include"TASK7.h"
#include <cstdio>
#include<iostream>
#include<sys/ipc.h>
#include<sys/shm.h>
#include<unistd.h>
#include<cstring>
#define SHM_SIZE 1024
#define SHM_KEY 0x1234
int main()
{
    //启动发送端,创建共享内存
    int shmid=shmget(SHM_KEY, SHM_SIZE, IPC_CREAT | 0666);
    Interface* shared_msg=(Interface*)shmat(shmid, NULL, 0);
    Interface msg;
    snprintf(msg.data, sizeof(msg.data), "Hello,world");
    msg.id=1;
    *shared_msg=msg;
    //输出信息
    cout<<msg.data<<"id:"<<msg.id<<endl;
    sleep(2);
    snprintf(shared_msg->data, sizeof(shared_msg->data), "END");
    cout<<"信息发送结束"<<endl;
    shmdt(shared_msg);
    cout<<"发送端结束"<<endl;
    return 0;
}