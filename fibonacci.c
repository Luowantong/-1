#include<stdio.h>
#define MAX 90
#define MIN 0
long long feibonacci_nth(int n)
{
    if (n<=1) 
    {
        return n;
    }
    return feibonacci_nth(n-1)+feibonacci_nth(n-2);
}
void fibonacci_sequence(int n)
{
    int i=0;
    while (i<=n) 
    {
        printf("%lld ",feibonacci_nth(i));
        if ((i+1)%8 == 0) 
        {
            printf("\n");
        }
        i++;
    }
}
long long fibonacci_sum(int n)
{
    int j=0;
    long long t = 0;
    while (j<=n) 
    {
        t += feibonacci_nth(j);
        j++;
    }
    return t;
}
int main()
{
    int n;
    printf("你想知道第几个斐波那契数列的数值:");
    scanf("%d",&n);
    printf("这个数的大小为：%lld ",feibonacci_nth(n));
    printf("\n");
    printf("前n项斐波那契数列为:");
    printf("\n");
    fibonacci_sequence(n);
    printf("\n");
    printf("计算这前n项的和:");
    printf("\n");
    printf("%lld ",fibonacci_sum(n));
    printf("\n");
    return 0;
}