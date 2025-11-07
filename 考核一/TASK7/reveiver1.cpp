#include "TASK7.h"
#include <cstring>
#include<iostream>
#include<sys/ipc.h>
#include<sys/shm.h>
#include<unistd.h>
#define SHM_SIZE 1024
#define SHM_KEY 0x1234
int main()
{
    sleep(1);
    int shmid=shmget(SHM_KEY, SHM_SIZE, 0666);
    Interface* shared_msg=(Interface*)shmat(shmid, NULL, 0);
    while (true) 
    {
        Interface msg=*shared_msg;
        if (strcmp(msg.data, "END")==0) 
        {
            cout<<"结束"<<endl;
            break;
        }
        cout<<msg.data<<"id:"<<msg.id<<endl;
        sleep(1);
    }
    shmdt(shared_msg);
    shmctl(shmid, IPC_RMID, NULL);
    cout<<"结束接收端"<<endl;
    return 0;
}