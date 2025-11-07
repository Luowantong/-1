#include <cmath>
#include<iostream>
#include <ostream>
#define _USE_MATH_DEFINES
using namespace std;
class Dual 
{
public:
    double val;  // 数值部分 f(x)
    double der;  // 导数部分 f'(x)
    Dual(double value = 0.0, double derivative = 0.0):val(value),der(derivative){};    
    //实现基本运算符重载
    Dual operator+(const Dual&per) const
    {
        Dual temp;
        temp.val=this->val+per.val;
        temp.der=this->der+per.der;
        return temp;
    }
    Dual operator-(const Dual&per) const
    {
        Dual temp;
        temp.val=this->val-per.val;
        temp.der=this->der-per.der;
        return temp;
    }
    Dual operator*(const Dual&per) const
    {
        Dual temp;
        temp.val=this->val*per.val;
        temp.der=this->val*per.der+this->der*per.val;
        return temp;
    }
    Dual operator/(const Dual&per) const
    {
        Dual temp;
        temp.val=this->val/per.val;
        temp.der=(this->der*per.val-this->val*per.der)/(per.val*per.val);
        return temp;
    }

    //混合运算符重载
    friend Dual operator+(double other1, const Dual& other2) 
    {
        return Dual(other1 + other2.val, other2.der);
    }      
    friend Dual operator-(double other1, const Dual& other2) 
    {
        return Dual(other1 - other2.val, -other2.der);
    }      
    friend Dual operator*(double other1, const Dual& other2) 
    {
        return Dual(other1 * other2.val, other1 * other2.der);
    }    
    friend Dual operator/(double other1, const Dual& other2) 
    {
        if (other2.val == 0.0) 
        {
            throw runtime_error("Division by zero");
        }
        return Dual(other1 / other2.val, -other1 * other2.der / (other2.val * other2.val));
    }

    // === 三角函数 ===
    friend Dual sin(const Dual&num);
    friend Dual cos(const Dual&num);
    friend Dual tan(const Dual&num);
    friend Dual cot(const Dual&num);
    friend Dual sec(const Dual&num);
    friend Dual csc(const Dual&num);

    // === 反三角函数 ===
    friend Dual asin(const Dual&num);
    friend Dual acos(const Dual&num);
    friend Dual atan(const Dual&num);
    friend Dual acot(const Dual&num);

    // === 双曲函数 ===
    friend Dual sinh(const Dual&num);
    friend Dual cosh(const Dual&num);
    friend Dual tanh(const Dual&num);
    friend Dual coth(const Dual&num);

    // === 指数与对数函数 ===
    friend Dual exp(const Dual&num);
    friend Dual log(const Dual&num);
    friend Dual log10(const Dual&num);

    // === 幂函数 ===
    friend Dual pow(const Dual&num, double n);
    friend Dual pow(const Dual&num1, const Dual&num2);

    // === 常见复合算术函数 ===
    friend Dual sqrt(const Dual&num);
    friend Dual abs(const Dual&num);
    friend Dual floor(const Dual&num);
    friend Dual ceil(const Dual&num);
    friend Dual erf(const Dual&num);
    friend Dual sigmoid(const Dual&num);   // 自定义常用函数 σ(x) = 1/(1+e^-x)
};
//实现常见数学函数
//支持以下函数，并能正确传播导数，每种至少原则3个（除了幂函数）可使用cmath
// === 基本初等函数 ===
// === 三角函数 ===
Dual sin(const Dual&num)
{
    return Dual(std::sin(num.val),std::cos(num.val)*num.der);
}
Dual cos(const Dual&num)
{
    return Dual(std::cos(num.val),-std::sin(num.val)*num.der);
}
Dual tan(const Dual&num)
{
    return Dual(std::tan(num.val),1+std::tan(num.val)*std::tan(num.val)*num.der);
}
Dual cot(const Dual&num)
{
    return Dual(1/std::tan(num.val),-1/std::sin(num.val)*sin(num.val)*num.der);
}
Dual sec(const Dual&num)
{
    return Dual(1/std::cos(num.val),std::tan(num.val)/std::sin(num.val)*num.der);
}
Dual csc(const Dual&num)
{
    return Dual(1/std::sin(num.val),-1/std::sin(num.val)*tan(num.val)*num.der);
}

// === 反三角函数 ===
Dual asin(const Dual&num)
{
    return Dual(std::asin(num.val),1/std::sqrt(1-num.val*num.val)*num.der);
}
Dual acos(const Dual&num)
{
    return Dual(std::acos(num.val),-1/std::sqrt(1-num.val*num.val)*num.der);
}
Dual atan(const Dual&num)
{
    return Dual(std::atan(num.val),(1.0/(1+num.val*num.val))*num.der);
}
Dual acot(const Dual&num)
{
    return Dual(std::atan(1/num.val),(-1.0/1+num.val*num.val)*num.der);
}

// === 双曲函数 ===
Dual sinh(const Dual&num)
{
    return Dual(std::sinh(num.val),std::cosh(num.val)*num.der);
}
Dual cosh(const Dual&num)
{
    return Dual(std::cosh(num.val),std::sinh(num.val)*num.der);
}
Dual tanh(const Dual&num)
{
    return Dual(std::tanh(num.val),1-std::tanh(num.val)*tanh(num.val)*num.der);
}
Dual coth(const Dual&num)
{
    return Dual(1/std::tanh(num.val),-1/sinh(num.val)*sinh(num.val)*num.der);
}

// === 指数与对数函数 ===
Dual exp(const Dual&num)
{
    return Dual(std::exp(num.val),std::exp(num.val)*num.der);
}
Dual log(const Dual&num)
{
    return Dual(std::log(num.val),(1/num.val)*num.der);
}
Dual log10(const Dual&num)
{
    return Dual(std::log10(num.val),(1/(num.val*std::log(10.0)))*num.der);
}

// === 幂函数 ===
Dual pow(const Dual&num, double n)
{
    return Dual(std::pow(num.val, n),n*std::pow(num.val, n-1)*num.der);
}
Dual pow(const Dual&num1, const Dual& num2)
{
    double val=std::pow(num1.val,num2.val);
    double der=val*(num2.der*std::log(num1.val)+num2.val*num1.der/num1.val);
    return Dual(val,der);
}

// === 常见复合算术函数 ===
Dual sqrt(const Dual&num)
{
    return Dual(std::sqrt(num.val),(1.0/(2*std::sqrt(num.val)))*num.der);
}
Dual abs(const Dual&num)
{
    if (num.val==0) 
    {
        cout<<"不能计算零点导数"<<endl;
    }
    double der=(num.val>0.0?num.der:-num.der);
    return Dual(std::abs(num.val),der);
}
Dual floor(const Dual&num)
{
    if (std::abs(num.val - std::round(num.val)) < 1e-10) 
    {
        throw std::runtime_error("floor not differentiable at integers");
    }
    return Dual(std::floor(num.val), 0.0);
}
Dual ceil(const Dual&num)
{
    if (std::abs(num.val - std::round(num.val)) < 1e-10) 
    {
        throw std::runtime_error("ceil not differentiable at integers");
    }
    return Dual(std::ceil(num.val), 0.0);
}
Dual erf(const Dual&num)
{
    return Dual(std::erf(num.val),2/std::sqrt(M_PI)*std::exp(-num.val*num.val)*num.der);
}
Dual sigmoid(const Dual&num)// 自定义常用函数 σ(x) = 1/(1+e^-x)
{
    return Dual(1/(1+std::exp(-num.val)),1/(1+std::exp(-num.val))*(1-1/(1+std::exp(-num.val)))*num.der);
}
void test01()
{
    Dual x(2.0, 1.0);
    Dual y = sin(x) + x * x;
    std::cout << "f(x) = " << y.val << ", f'(x) = " << y.der << std::endl;
}
void test02()
{
    Dual x(1.0, 1.0);
    Dual y = exp(x) / (1.0 + exp(x)); // Sigmoid
    std::cout << "f(x) = " << y.val << ", f'(x) = " << y.der << std::endl;
}
int main()
{
    test01();
    test02();
}