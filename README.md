# HUST-OS-Experiment

Hust operating system experiment in 2018 Spring.

Update: add summer assignment in this repo.

## Enviorment

	gcc version 5.4.0 20160609 
	
	(Ubuntu 5.4.0-6ubuntu1~16.04.9 in Windows Subsystem Linux)

	Thread model: posix

## File

 - **lab directory**

	Codes of each labs which are all independent of others, executable files of those codes and some test files

 - **.vscode**

	Configuration of running linux c codes on vscode-windows


## Detail

 - **lab1:** 

	Use pipe to transfer data between two prosess.
		
 - **lab2:**

	Use semaphore to synchronize threads' behavior.
	
 
 - **lab3:**

	Use shared memory and semaphore to let two processes copy data in order.


 - **labextra:**

	Simulate linux basic function 'ls'.
	Extended arguments implmented:

	 - Specified path
	 - `-l` 
	 - `-lR`

 - **lab_tinyOS**

	TinyOS labs have several parts.
	
	Count: count 0 to 7, and print info through serial port.
	
	Loop, Task, Split Task: big calculation task situation simulation
	
	SensorDemo: get sensors' data and send them for output.

 - **CharDriver**
 
	Add new char device to linux and write it's own driver
	
 - **ParallelWindows**	
 
	Three windows showing different info and update at the same time
	
 - **Syscall**		
 
	Compile a new linux kernel. (only new system call code in this directory)

 - **Monitor**		
 
	System process manager that can kill process and monitor up-to-date process info including cpu and memory usage.	
	
 - **FileSystem**		
 
	Simulate a linux file system based on disk blocks	
	
 - **doc.pdf**

	Document of this experiment.
