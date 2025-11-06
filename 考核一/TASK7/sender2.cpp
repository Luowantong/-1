#include"TASK7.h"
#include<iostream>
#include<sys/stat.h>
#include<fcntl.h>
#include<unistd.h>
#define FIFO_NAME "/tmp/hello_fifo"
int main()
{
    mkfifo(FIFO_NAME,0666);
    int fd=open(FIFO_NAME,O_WRONLY);
    Interface msg;
    snprintf(msg.data, sizeof(msg.data), "Hello,world");
    msg.id=1;
    write(fd,&msg,sizeof(Interface));
    cout<<msg.data<<"id:"<<msg.id<<endl;
    sleep(2);
    Interface end_msg;
    snprintf(end_msg.data, sizeof(end_msg.data), "END");
    write(fd, &end_msg, sizeof(Interface));
    cout<<"结束"<<endl;
    close(fd);
    unlink(FIFO_NAME);
    return 0;
}