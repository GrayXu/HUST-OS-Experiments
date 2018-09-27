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

int main(int argc, char * argv[]){
	if(argc != 3){ 
		printf("args error");
		return;
	}
	
	x = 10;
	char addrs[10];
	int inFile, outFile;
	inFile = open(argv[1], O_RDONLY);
	outFile = open(argv[2], O_CREAT);
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
