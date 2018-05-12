#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

int child1(int *filedis);
int child2(int *filedis);
int handle();
int pid1 = 0;
int pid2 = 0;
int pipe_field[2];// 一读一写 0写1读

int main()
{	
	// signal(SIGINT,SIG_IGN);
	if (pipe(pipe_field) < 0)
	{
		printf("pipe create failed!\n");
		return -1;
	}
	else
	{
		printf("------successfully-------\n");

		pid1 = fork();
		if (pid1 == 0)
		{
			child1(pipe_field);
			// exit(0);
		}
		pid2 = fork();
		if (pid2 == 0)
		{
			child2(pipe_field);
			// exit(0);
		}
		
		//parent
		signal(SIGINT,handle);
		wait(pid1);
		wait(pid2);

		//close pipe
		close(pipe_field[0]);
		close(pipe_field[1]);

		printf("Parent Process is Killed!\n");
	}
	return 0;
}

int handle(){
	kill(pid1, SIGKILL);
	printf("Child Process l is Killed by Parent!\n");
	kill(pid2, SIGKILL);
	printf("Child Process 2 is Killed by Parent!\n");
}

int child1(int *filedis)
{
	int count = 0;
	close(filedis[0]);
	while (1)
	{
		// printf("1 is in loops\n");
		char string[50];
		sprintf(string, "I send you %d", count);
		strcat(string, " times.\n");
		write(filedis[1], string, 50);
		sleep(1);
		count++;
	}
	close(filedis[1]);
}

int child2(int *filedis)
{
	char cache[50];
	cache[0] = 0;
	close(filedis[1]);
	while (1)
	{
		// printf("1 is in loops\n");
		read(filedis[0], cache, 50);
		printf("%s",cache);
	}
	close(filedis[0]);
}