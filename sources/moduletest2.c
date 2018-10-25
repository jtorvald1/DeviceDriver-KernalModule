#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/types.h>
#include <fcntl.h>
#include <errno.h>
#include <sys/ioctl.h>

#define DM510_IOC_MAGIC  82

#define DM510_IOCBUFFERSET    _IO(DM510_IOC_MAGIC, 0)
#define DM510_IOCBUFFERGET    _IO(DM510_IOC_MAGIC, 1)
#define DM510_IOCPROCSET      _IO(DM510_IOC_MAGIC, 2)
#define DM510_IOCPROCGET      _IO(DM510_IOC_MAGIC, 3)

#define DM510_IOC_MAXNR 4

int test1(int fd);
int test2(int fd);
int test3(int fd, int rd);
int test4(int fd, int rd);
int test5(int fd);

int main(int argc, char** argv) {

  int res = 0;
  int choice;
  
  
  
  int fd = open("/dev/dm510-0", O_RDWR);
  int rd = open("/dev/dm510-1", O_RDWR);
	perror("w open");	

  printf("Which test would you like to run? \n");
  scanf(" %i", &choice);
  printf("Running test %i.\n", choice);

  if(choice == 1) {
	res = test1(fd);
  }

  if(choice == 2) {
	res = test2(fd);
  }

  if(choice == 3) {
	res = test3(fd, rd);
  }

  if(choice == 4) {
	res = test4(fd, rd);
  }
  
  if(choice == 5) {
	res = test5(fd);
  }

return res;
}

int test1(int fd){
  int buf = 22;

  printf("This test will attempt to write a message in which the number of bytes to write is incorrectly specified as a negative value. \n");

  /* Write */
  int ret;
        ret = write(fd, &buf, -1);
        if (ret == -1) {
            perror("write");
            exit(1);
        }
 
  return ret;
}

int test2(int fd){
  int buf = NULL;

  printf("This test will attempt to write a message, passing a NULL value as the buffer parameter \n");

  /* Write */
  int ret;
        ret = write(fd, NULL, 5);
        if (ret == -1) {
            perror("write");
            exit(1);
        }
        
  return ret;
}

int test3(int fd, int rd){
  int buf = 1;

  printf("This test will first write a message with one device, then attempt to read that message with another device, passing a NULL value as the buffer parameter \n");

  /* Write */
  int ret;
  ret = write(fd, &buf, 5);
        if (ret == -1) {
            perror("write");
            exit(1);
        }

  /* Read */
  ret = read(rd, NULL, 5);
        if (ret == -1) {
            perror("read");
            exit(1);
        }
 
  return ret;
}

int test4(int fd, int rd){
  int buf = 123;
  int retbuf;

  printf("This test will simply attempt to write a message with one device, and then read using another device, using acceptable data as parameters. \n");

  /* Write */
  int ret;
  ret = write(fd, &buf, 4);
        if (ret == -1) {
            perror("write");
            exit(1);
        }
 
  /* Read */
  ret = read(rd, &retbuf, 4);
        if (ret == -1) {
            perror("read");
            exit(1);
        }
  return ret;
}

int test5(int fd){

  printf("Now the ioctl function will be tested, which includes the functionality to get and set the buffer size, as well as get and set the number of allowed read processes.  \n");
  printf("\n");
  printf("Running ioctl buffer tests.\n");
  printf("\n");
  printf("Using ioctl to get the original buffersize. Current buffersize is: %d.\n", ioctl(fd,DM510_IOCBUFFERGET));
  printf("Using ioctl to set a new buffersize. \n");
  ioctl(fd,DM510_IOCBUFFERSET,5000);
  printf("The new buffersize is: %d,\n", ioctl(fd,DM510_IOCBUFFERGET));
  
  printf("\n");
  printf("Running ioctl allowed-read-processes tests.\n");
  printf("Using ioctl to get the original number of allowed read processes. The current number of allowed read processes is: %d,\n", ioctl(fd,DM510_IOCPROCGET));
  printf("Using ioctl to set a new amount of allowed read processes. \n");
  ioctl(fd,DM510_IOCPROCSET,50);
  printf("The new amount of allowed read processes is: %d,\n", ioctl(fd,DM510_IOCPROCGET));
  
  return 0;

  
}
