#define CONFIGMANAGER_H
#include<string>
#include<map>
#include<sstream>
#include<fstream>
using namespace std;
class ConfigManager
{
private:
    map<string, string> configMap;
public:
    bool Load()
    {
        ifstream file("TASK3.yml");
        if(!file.is_open()) 
        {
            return false;
        }
        configMap.clear();
        string line;
        while (getline(file,line)) 
        {
            stringstream ss(line);
            string key,value;
            if(getline(ss,key,':')&& getline(ss,value))
            {
                configMap[key]=value;
            }
        }
        file.close();
        return true;
    }
    template<class T>
    T getValue(const string&key,T defaultValue)
    {
        map<string, string>::iterator it=configMap.find(key);
        if (it==configMap.end()) 
        {
            return defaultValue;
        }
        string value=it->second;
        stringstream ss(value);
        T result;
        if (ss>>result) 
        {
            return result;
        }
        return defaultValue;
    }
};