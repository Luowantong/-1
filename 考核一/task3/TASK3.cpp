#include<iostream>
#include<thread>
#include<chrono>
#include"ConfigManager.h"
using namespace std;
int main()
{
    ConfigManager config;
    while (true) 
    {
        config.Load();
        cout<<config.getValue<string>("app_name","默认应用")<<endl;
        cout<<config.getValue<int>("duankou", 8080)<<endl;
        cout<<config.getValue<bool>("tiaoshi",false)<<endl;
        this_thread::sleep_for(chrono::seconds(2));
    }
    return 0;
}