#include <fcntl.h>
#include <pthread.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/sem.h>
#include <sys/shm.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>
// #include <sys/wait.h>

#define LENGTH 1024

int pid1;
int pid2;
int sem_id = 0;
int shmid;
FILE *readFile;
FILE *writeFile;

void writeChild();
void readChild();
int setSemvalue();
void delSemvalue();
int P(int index);
int V(int index);

union semun {
	int val;
	struct semid_ds *buf;
	unsigned short *arry;
};

int main(int argc, char *argv[])
// int main()
{
	printf("-------------start copying!----------\n");

	// FILE *readFile = fopen(argv[1], "r+");
	// FILE *writeFile = fopen(argv[2], "w+");
	readFile = fopen("r.txt", "r+");
	writeFile = fopen("w.txt", "w+");
	if (readFile == NULL || writeFile == NULL)
	{
		printf("File error.\n");
		exit(1);
	}

	//initalize shared memory
	if ((shmid = shmget(IPC_PRIVATE, LENGTH * sizeof(char), 0666 | IPC_CREAT)) == -1) //connect
	{
		printf("Create Share Memory Error");
		exit(1);
	}

	//initalize sem
	sem_id = semget((key_t)IPC_PRIVATE, 2, 0666 | IPC_CREAT);
	if (sem_id == -1)
	{
		printf("failed to initalize semaphore\n");
		exit(0);
	}
	if (setSemvalue() == 0)
	{
		exit(1);
	}

	printf("successfully initalized\n");

	//initalize process
	pid1 = fork();
	if (pid1 == 0)
	{
		readChild();
		exit(0);
	}
	pid2 = fork();
	if (pid2 == 0)
	{
		writeChild();
		exit(0);
	}

	wait(pid1);
	wait(pid2);

	printf("finish copying\n");
	delSemvalue();
	// shmdt(head_addr);//deattach
	if (shmctl(shmid, IPC_RMID, 0) < 0) //release shared memory
	{
		printf("release error\n");
		exit(1);
	}

	fclose(readFile);
	fclose(writeFile);
	return 0;
}

void writeChild()
{
	char *head_addr = (char *)shmat(shmid, 0, 0);
	
	int index = 0;
	//write
	char get = ' ';
	while (fread(&get, sizeof(char), 1, readFile) != 0)
	{
		P(1); //init with 1024
		head_addr[index] = get;
		index++;
		index = index % 1024;
		V(0);
	}
	//set end notation
	P(1);
	head_addr[index] = '\0';
	V(0);

}

void readChild()
{
	char *head_addr = (char *)shmat(shmid, 0, 0);
	int index = 0;
	//read
	while (1)
	{
		P(0); //init with 0
		char get = head_addr[index];
		if (get == '\0')
		{
			V(1);
			return; //reach the end
		}
		fwrite(&get, sizeof(char), 1, writeFile);
		// printf("read:%c\n", get);
		head_addr[index] = '\0';
		index++;
		index = index % 1024;
		V(1);
	}
}

//init sem's value
int setSemvalue()
{
	union semun arg1;
	arg1.val = 0; //first one refers to "place to write"
	union semun arg2;
	arg2.val = LENGTH; //refer to "place to read"
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