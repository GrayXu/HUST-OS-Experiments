
#include<stdio.h>
#include<stdlib.h>//exit()
#include<sys/sem.h>
#include<fcntl.h>//open()

int x;
void readbuf(char *addr, int steam){
	write(steam, addr, x);
}

void writebuf(char *addr, int steam){
	x = read(steam, addr, 10);
}

int main(){
	x = 10;
	char addrs[10];
	int inFile, outFile;
	inFile = open("original", O_RDONLY);
	outFile = open("copy", O_CREAT);
	if(inFile < 0){
		printf("-1\n");
		exit(1);
    }
    if(outFile < 0){
		printf("-1\n");
		exit(1);
    }
	while(x > 0){
		writebuf(addrs, inFile);
		readbuf(addrs, outFile);
	}
	
	close(inFile);
	close(outFile);
	
	printf("1\n");
	return 0;
}
