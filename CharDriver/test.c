#include <sys/types.h>  
#include <sys/stat.h>  
#include <stdlib.h>  
#include <string.h>  
#include <stdio.h>  
#include <fcntl.h>  
#include <unistd.h>  
#define MAX_SIZE 1024  
  
int main(void)  
{  
    int fd;  
    char rdbuf[MAX_SIZE];  
    char wrbuf[MAX_SIZE];  
    char rerdbuf[MAX_SIZE];
    char device[20], dir[50] = "/dev/gray_driver";
    
    fd = open(dir, O_RDWR | O_NONBLOCK);
    if (fd != -1)  
    {  
        read(fd, rdbuf, sizeof(rdbuf));  
        printf("The string in the device is: %s\n", rdbuf);  
        printf("Input a string  :\n");
		scanf("%[^\n]",wrbuf);
        write(fd, wrbuf, sizeof(wrbuf));  
        read(fd,rerdbuf, sizeof(rerdbuf));
        printf("\nThe string in the device now is : %s\n", rerdbuf);  
        close(fd);  
        return 0;  
    }  
    else  
    {  
        printf("Fail to open device!!\n"); 
		perror("open"); 
        return -1;  
    }  
}
