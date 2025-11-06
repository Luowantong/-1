#include<iostream>
#include <stdexcept>
#include<string>
using namespace std;
class Person 
{
private:
   string m_name;
   int m_age;
   bool nameisSet=false;
   bool ageisSet=false;
   void Set();
public:
    Person():m_name(),m_age(0),nameisSet(false),ageisSet(false){}
    Person(string name,int age)
   {
        this->m_name=name;
        this->m_age=age;
        this->nameisSet=true;
        this->ageisSet=true;
   }
   void setName(string name)
   {
        m_name=name;
        nameisSet=true;
   }
   string getName()
   {
        if (!nameisSet) 
        {
            throw runtime_error("用户名未被定义");
        }
        return m_name;
   }
   bool isSetName()
   {
        return nameisSet;
   }
   void clearName()
   {
        nameisSet=false;
   }
   void setAge(int age)
   {
        m_age=age;
        ageisSet=true;
   }
   int getAge()
   {
        if (!ageisSet) 
        {
            throw runtime_error("用户年龄未被定义");
        }
        return m_age;
   }
   bool isSetAge()
   {
        return ageisSet;
   }
   void clearAge()
   {
        ageisSet=false;
   }
};
void test01()
{
    Person p1("张三",18);
    cout<<"姓名："<<p1.getName()<<"年龄:"<<p1.getAge()<<endl;
    p1.clearName();
    p1.clearAge();
    cout<<"姓名："<<p1.isSetName()<<"年龄："<<p1.isSetAge()<<endl;
}
int main()
{
    test01();
    return 0;
}