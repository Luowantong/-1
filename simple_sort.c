#include<stdio.h>
#include<time.h>
#include<stdlib.h>
void Bubble_sort(int arr[],int n)
{
    for (int i=0; i<n-1; i++) 
    {
        for (int j=0; j<n-1-i; j++) 
        {
            if (arr[j]>arr[j+1]) 
            {
                int temp;
                temp=arr[j];
                arr[j]=arr[j+1];
                arr[j+1]=temp;
            }
        }
    }
}
void Selection_sort(int arr[],int n)
{
    for (int i=0; i<n-1; i++) 
    {
        int a=i;
        for (int j=i+1; j<n; j++) 
        {
            if (arr[a]>arr[j]) 
            {
                a=j;
            }
        }
        if (a!=i) 
        {
            int temp;
            temp=arr[i];
            arr[i]=arr[a];
            arr[a]=temp;
        }
    }
}
void insertion_sort(int arr[],int n)
{
    for (int i=1; i<n; i++) 
    {
        int a,j;
        a=arr[i];
        j=i-1;
        while (j>=0&&arr[j]>a) 
        {
           arr[j+1]=arr[j];
           j--; 
        }
        arr[j+1]=a;
    }
}
void copyArray(int arr[],int dest[],int n)
{
    for (int i=0; i<n; i++) 
    {
        dest[i]=arr[i];
    }
}
int main()
{
    srand((size_t)time (NULL));
    int len=10;
    int str[10]={0};
    int Bubblesort[10];
    int Selectionsort[10];
    int insertionsort[10];
    int value=0;
    int flag=0;
    for (int i=0; i<len; i++) 
    {
        value=rand()%99;
        int j;
        for (j=0; j<flag; j++) 
        {
            if (value==str[j]) 
            {
                i--;
                break;
            }
        }
        if (j==flag) 
            {
                str[flag]=value;
                flag++;
            }
    }
    printf("该数组为：");
    for (int i=0; i<len; i++) 
    {
        printf("%d ",str[i]);
    }
    copyArray(str, Bubblesort, len);
    copyArray(str, Selectionsort, len);
    copyArray(str, insertionsort, len);
    Bubble_sort(Bubblesort, len);
    printf("\n");
    printf("该数组升序排序之后的数组为：");
    for (int i=0; i<len; i++ )
    {
        printf("%d ",Bubblesort[i]);
    }
    printf("\n");
    Selection_sort(Selectionsort, len);
    printf("该数组升序排序之后的数组为：");
    for (int i=0; i<len; i++ )
    {
        printf("%d ",Selectionsort[i]);
    }
    printf("\n");
    insertion_sort(insertionsort, len);
    printf("该数组升序排序之后的数组为：");
    for (int i=0; i<len; i++ )
    {
        printf("%d ",insertionsort[i]);
    }
    printf("\n");
    return 0;
}