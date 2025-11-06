#include"TASK7.h"
#include<iostream>
#include <fcntl.h>
#include<sys/stat.h>
#include<fcntl.h>
#include<unistd.h>
#define FIFO_NAME "/tmp/hello_fifo"
int main()
{
    int fd=open(FIFO_NAME,O_RDONLY);
    while (true) 
    {
        Interface msg;
        if (read(fd,&msg,sizeof(Interface))>0) 
        {
            if (strcmp(msg.data,"END")==0) 
            {
                cout<<"结束"<<endl;
                break;
            }
            cout<<msg.data<<"id:"<<msg.id<<endl;
        }
    }
    close(fd);
    return 0;
}