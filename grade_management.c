#include<stdio.h>
#define Max 100
void inputScores(int *scores, int n)
{
    printf("请输入学生分数：");
    for (int i=0; i<n; i++) 
    {    
        scanf("%d ",&*(scores+i));
    }
}
double calculateAverage(int *scores, int n)
{
    double sum,average;
    sum = 0;
    for (int i=0; i<n; i++) 
    {
        sum += *(scores+i);
    }
    average=sum/n;
    return average;
}
void findMinMax(int *scores, int n, int *max, int *min)
{
    *max=*(scores+0);
    *min=*(scores+0);
    for (int i=1; i<n; i++) 
    {
        if (*(scores+i)>*max) 
        {
            *max=*(scores+i);
        }
    }
    for (int i=0; i<n; i++) 
    {
        if (*(scores+i)<*min) 
        {
            *min=*(scores+i);
        }
    }
}
void sortScores(int *scores, int n)
{
    for (int i = 0; i < n - 1; i++) 
    {
        for (int j = 0; j < n - 1 - i; j++) 
        {
            if (*(scores+j) < *(scores+j+1)) 
            {
                int flag;
                flag = *(scores+j);
                *(scores+j) = *(scores+j+1);
                *(scores+j+1) = flag;
            }
        }
    }
}
void countGrades(int *scores, int n, int *counts)
{
    for (int i=0; i<5; i++) 
    {
        counts[i]=0;
    }
    for (int i=0; i<n; i++) 
    {
        if (90<= *(scores+i)&&*(scores+i)<=100) 
        {
            counts[0]++;
        }
        else if (80<=*(scores+i)&&*(scores+i)<=89) 
        {
            counts[1]++;
        }
        else if (70<=*(scores+i)&&*(scores+i)<=79) 
        {
            counts[2]++;
        }
        else if (60<=*(scores+i)&&*(scores+i)<=69) 
        {
            counts[3]++;
        }
        else 
        {
            counts[4]++;
        }
    }
}
int main() 
{
 int numStudents;
 
 printf("请输入学生人数: ");
 scanf("%d", &numStudents);
 
 int scores[numStudents];
 int gradeCounts[5] = {0};
 
 // 调用各功能函数
 inputScores(scores, numStudents);
 
 double avg = calculateAverage(scores, numStudents);
 printf("平均分: %.2f\n", avg);
 
 int maxScore, minScore;
 findMinMax(scores, numStudents, &maxScore, &minScore);
 printf("最高分: %d, 最低分: %d\n", maxScore, minScore);
 
 sortScores(scores, numStudents);
 printf("成绩降序排列: ");
 for(int i = 0; i < numStudents; i++) 
 {
 printf("%d ", scores[i]);
 }
 printf("\n");
 
 countGrades(scores, numStudents, gradeCounts);
 printf("等级统计:\n");
 printf("优秀(90-100): %d人\n", gradeCounts[0]);
 printf("良好(80-89): %d人\n", gradeCounts[1]);
 printf("中等(70-79): %d人\n", gradeCounts[2]);
 printf("及格(60-69): %d人\n", gradeCounts[3]);
 printf("不及格(0-59): %d人\n", gradeCounts[4]);
 
 return 0;
}