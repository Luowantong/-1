#include<iostream>
#include<string>
#include"TASK5.hpp"
class Person
{
private:
    Property<std::string>m_name;
    Property<int>m_age;
public:
    Person():m_age(),m_name(){}
    Person(std::string name,int age)
    {
        setName(name);
        setAge(age);
    }
    void setName(std::string name)
    {
        m_name.Set(name);
    }
    std::string getName()const
    {
        return m_name.Get();
    }
    bool isSetName()const
    {
        return m_name.IsSet();
    }
    const void clearName()
    {
        m_name.Clear();
    }
    void setAge(int age)
    {
        m_age.Set(age);
    }
    const int getAge()
    {
        return m_age.Get();
    }
    const bool isSetAge()
    {
        return m_age.IsSet();
    }
    const void clearAge()
    {
        m_age.Clear();
    }
    void display()
    {
        std::cout<<"姓名："<<getName()<<std::endl;
        std::cout<<"年龄："<<getAge()<<std::endl;
    }
};
void test()
{
    Person p1("张三",18);
    p1.display();
}
int main()
{
    test();
}