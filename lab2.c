#include <fcntl.h>
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/sem.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

void printTid();
void *myThread();
void delSemvalue();
int setSemvalue();
void *subp1();
void *subp2();
int P(int index);
int V(int index);

pthread_t id1; //thread id
pthread_t id2;
int sem_id = 0;
int threadCount = 2;
union semun {
    int val;
    struct semid_ds *buf;
    unsigned short *arry;
};

// struct sembuf
// {
//     short sem_num; // index
//     short sem_op;  // Operation for semaphore, -1->P, +1->V.
//     short sem_flg; // Operation flags. Let OS know where is this sem, and delete it when something bad happens
// };

int main()
{
    printf("------------hello world!----------\n");

    //initalize semaphore (we have two)
    sem_id = semget((key_t)IPC_PRIVATE, 2, 0666 | IPC_CREAT);
    if (sem_id == -1)
    {
        printf("failed to initalize semaphore\n");
        exit(0);
    }
    if (setSemvalue() == 0)
    {
        printf("setSemvalue failed\n");
        exit(1);
    }
    printf("successfully initalized\n");

    int result1 = pthread_create(&id1, NULL, subp1, NULL);
    int result2 = pthread_create(&id2, NULL, subp2, NULL);
    if (result1 != 0 || result2 != 0)
    {
        printf("result failed\n");
        exit(1);
    }
    printf("threads have been created\n");
    // printTid();
    while (threadCount > 0)
    {
        sleep(2);
    }
    delSemvalue(); //delete
    return 0;
}

//init sem's value
int setSemvalue()
{
    union semun arg1;
    arg1.val = 0;
    union semun arg2;
    arg2.val = 1;
    if (semctl(sem_id, 0, SETVAL, arg1) == -1)
    {
        printf("setSemvalue failed\n");
        return 0;
    }
    if (semctl(sem_id, 1, SETVAL, arg2) == -1)
    {
        printf("setSemvalue failed\n");
        return 0;
    }
    return 1;
}

void delSemvalue()
{
    union semun sem_union;
    if (semctl(sem_id, 1, IPC_RMID, sem_union) == -1)
    {
        printf("delSemvalue failed\n");
        exit(1);
    }
}

int sum = 0;

void *subp1()
{
    printTid();
    int i;
    for (i = 1; i <= 100; i++)
    {
        P(1);
        printf("thread 1 in loops, %d times\n", i);
        sum += i;
        V(0);
    }
    threadCount--;
    return ((void *)0);
}

void *subp2()
{
    printTid();
    int i;
    for (i = 1; i <= 100; i++)
    {
        P(0);
        printf("thread 2 in loops, %d times\n", i);
        printf("%d\n", sum);
        V(1);
    }
    threadCount--;
    return ((void *)0);
}

void printTid()
{
    printf("tid: (0x%lx)\n", (unsigned long)pthread_self());
}

int P(int index)
{   
    //add 1 to sem
    struct sembuf sem_b;
    sem_b.sem_num = index;
    sem_b.sem_op = -1; 
    sem_b.sem_flg = 0;
    if (semop(sem_id, &sem_b, 1) == -1)
    {
        printf("P failed\n");
        return 0;
    }
    return 1;
}

int V(int index)
{
    //delete 1 from sem
    struct sembuf sem_b;
    sem_b.sem_num = index;
    sem_b.sem_op = 1;
    sem_b.sem_flg = 0;
    if (semop(sem_id, &sem_b, 1) == -1)
    {
        printf("V failed\n");
        return 0;
    }
    return 1;
}