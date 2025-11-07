#include <stdexcept>
template <class T>
class Property
{
private:
    T m_value;
    bool isSet=false;
public:
    Property():m_value(),isSet(false){}
    void Set(const T &value)
    {
        m_value=value;
        isSet=true;
    }
    T Get()const
    {
        if (!isSet) 
        {
            throw std::runtime_error("用户名未被定义");
        }
        return m_value;
    }
    bool IsSet()const
    {
        return isSet;
    }
    void Clear()
    {
        isSet=false;
    }
};
