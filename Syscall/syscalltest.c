#include <sys/syscall.h>
#include <linux/unistd.h>

int main(){
	
	printf("%d\n",syscall(385,"original","copy"));
	
	return 0;
}
