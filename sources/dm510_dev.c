/* Prototype module for third mandatory DM510 assignment */
#ifndef __KERNEL__
#  define __KERNEL__
#endif
#ifndef MODULE
#  define MODULE
#endif

#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>	
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/errno.h>
#include <linux/types.h>
#include <linux/wait.h>
#include <asm/uaccess.h>
#include <linux/semaphore.h>
/* #include <asm/system.h> */
#include <asm/switch_to.h>
#include <linux/cdev.h>
#include <linux/sched.h>
#include "linux/stddef.h"

#define init_MUTEX(LOCKNAME) sema_init(LOCKNAME,1);

/* Prototypes - this would normally go in a .h file */
static int dm510_open( struct inode*, struct file* );
static int dm510_release( struct inode*, struct file* );
static ssize_t dm510_read( struct file*, char*, size_t, loff_t* );
static ssize_t dm510_write( struct file*, const char*, size_t, loff_t* );
long dm510_ioctl(struct file *filp, unsigned int cmd, unsigned long arg);

#define DEVICE_NAME "dm510_dev" /* Dev name as it appears in /proc/devices */
#define MAJOR_NUMBER 254
#define MIN_MINOR_NUMBER 0
#define MAX_MINOR_NUMBER 1

#define DEVICE_COUNT 2
#define BUFFER_SIZE 2000
#define PROC_NUMBER 1 /* number of processes allowed to read */

/*
 * Ioctl definitions
 */

#define DM510_IOC_MAGIC  82

#define DM510_IOCBUFFERSET    _IO(DM510_IOC_MAGIC, 0)
#define DM510_IOCBUFFERGET    _IO(DM510_IOC_MAGIC, 1)
#define DM510_IOCPROCSET      _IO(DM510_IOC_MAGIC, 2)
#define DM510_IOCPROCGET      _IO(DM510_IOC_MAGIC, 3)


#define DM510_IOC_MAXNR 4
/* end of what really should have been in a .h file */

/* structure for our char buffers */
struct my_buffer
{
    char *buffer, *end;           /* pointers to the beginning and end of the buffer */
    int buffersize;
    int count;                    /* keeps track of how far into the buffer we are */ 
    int rp, wp;                   
    int numreaders, numwriters;   /* number of readers and writers */
    struct semaphore sem;
    wait_queue_head_t queue;      /* queue of sleeping processes waiting for access to buffer */
};

static struct my_buffer *buffers;

/* structure for our device */
struct my_device {
  struct my_buffer *rbuf, *wbuf;  /* pointers to the read-buffer and the write-buffer */
  struct semaphore sem;
  struct cdev cdev;
};

static struct my_device *dm510_device;

/*other variables */
int ret;                            /* return value, for error checking */
int dev_buffer_size = BUFFER_SIZE;
int dev_number_proc = PROC_NUMBER;

/* file operations struct */
static struct file_operations dm510_fops = {
	      .owner   = THIS_MODULE,
	      .read    = dm510_read,
	      .write   = dm510_write,
	      .open    = dm510_open,
	      .release = dm510_release,
        .unlocked_ioctl   = dm510_ioctl
};

static void setup_cdev(struct my_device *dev, int count) {
  cdev_init(&dev->cdev, &dm510_fops);
  dev->cdev.owner = THIS_MODULE;
  dev->cdev.ops = &dm510_fops;
  
  int devno = MKDEV(MAJOR_NUMBER,MIN_MINOR_NUMBER+count);
  
  ret = cdev_add(&dev->cdev, devno, 1);
  if (ret) {
    printk(KERN_ALERT "DM510: Unable to add cdev to kernel.\n");
    return ret;
  }
}

/* called when module is loaded */
int dm510_init_module( void ) {
  int i;
	/* initialization code belongs here */
  printk("About to init_module\n");
	
	ret = register_chrdev_region(MAJOR_NUMBER,DEVICE_COUNT,DEVICE_NAME);
  if (ret < 0) {
    printk(KERN_ALERT "DM510: Failed to allocate a device region. Error %d\n", ret);
    return ret;
  }
  
  /* allocate memory for the devices */
  dm510_device = kmalloc(DEVICE_COUNT*sizeof(struct my_device), GFP_KERNEL);
  if (dm510_device == NULL) {
    printk(KERN_NOTICE "Unable to allocate memory! Out of memory!\n");
    printk("errno = -12 ENOMEM\n");
    unregister_chrdev_region(MAJOR_NUMBER, DEVICE_COUNT);
    return -ENOMEM;
  }
  memset(dm510_device, 0, DEVICE_COUNT*sizeof(struct my_device));
  
  printk("RegionAlloc finished. About to set up cdev.\n");
  
  /* allocate memory for the buffers */
  buffers = kmalloc(DEVICE_COUNT*sizeof(struct my_buffer), GFP_KERNEL);
  if (buffers == NULL) {
    printk(KERN_NOTICE "Unable to allocate memory! Out of memory!\n");
    printk("errno = -12 ENOMEM\n");
    return -ENOMEM;
  }
  
  for (i = 0; i < DEVICE_COUNT; i++) {
    setup_cdev(&dm510_device[i], i);
    buffers[i].buffer = kmalloc(dev_buffer_size, GFP_KERNEL);
    if (!buffers[i].buffer) {
      printk(KERN_NOTICE "Unable to allocate memory! Out of memory!\n");
      printk("errno = -12 ENOMEM\n");
      return -ENOMEM;
    }
    buffers[i].count = 0;
    buffers[i].wp = 0;
    buffers[i].rp = 0;
    buffers[i].numwriters = 0;
    buffers[i].numreaders = 0;
    buffers[i].buffersize = dev_buffer_size;
    buffers[i].end = buffers[i].buffer + buffers[i].buffersize;
    init_waitqueue_head(&buffers[i].queue);
    init_MUTEX(&buffers[i].sem);
    init_MUTEX(&dm510_device[i].sem);
  }
  
  dm510_device[0].rbuf = dm510_device[1].wbuf = &buffers[0];
  dm510_device[1].rbuf = dm510_device[0].wbuf = &buffers[1];

	printk(KERN_INFO "DM510: Hello from your device!\n");
	return 0;
}

/* Called when module is unloaded */
void dm510_cleanup_module( void ) {

  int i;
  
  if(!dm510_device) {
    return;
  }
  for(i = 0; i<DEVICE_COUNT; i++) {
    cdev_del(&dm510_device[i].cdev);
    kfree(buffers[i].buffer);
    kfree(buffers);
  }
  kfree(dm510_device);
  unregister_chrdev_region(MAJOR_NUMBER, DEVICE_COUNT);
  dm510_device = NULL;

	printk(KERN_INFO "DM510: Module unloaded.\n");
}


/* Called when a process tries to open the device file */
static int dm510_open( struct inode *inode, struct file *filp ) {
	
	/* device claiming code belongs here */
	struct my_device *dev; /* device information */

	dev = container_of(inode->i_cdev, struct my_device, cdev);
	
	if(down_interruptible(&dev->sem)) {
	  printk(KERN_ALERT "DM510: Could not lock device during open.\n");
	  return -ERESTARTSYS;
	}
	
	filp->private_data = dev; /* for other methods */

	filp->private_data = dev;
	if(filp->f_mode & FMODE_READ) {
	  dev->rbuf->numreaders++;
	  if(dev->rbuf->numreaders > dev_number_proc) {
	    return -EACCES;
	  }
	}
	if(filp->f_mode & FMODE_WRITE) {
	  dev->wbuf->numwriters++;
	  if(dev->wbuf->numwriters > 1) {
	    return -EACCES;
	  }
	}
	
  printk(KERN_INFO "DM510: Device opened.\n");

	return nonseekable_open(inode,filp);
}


/* Called when a process closes the device file. */
static int dm510_release( struct inode *inode, struct file *filp ) {

	struct my_device *dev = filp->private_data;
	
	if (filp->f_mode & FMODE_READ) {
	  dev->rbuf->numreaders--;
	}
	if (filp->f_mode & FMODE_WRITE) {
	  dev->wbuf->numwriters--;
	}
	up(&dev->sem);

	printk(KERN_INFO "DM510: Released device.\n");
		
	return 0;
}


/* Called when a process, which already opened the dev file, attempts to read from it. */
static ssize_t dm510_read( struct file *filp,
    char *buf,      /* The buffer to fill with data     */
    size_t count,   /* The max number of bytes to read  */
    loff_t *f_pos )  /* The offset in the file           */
{
	
	int data = 0;
	struct my_device *dev = filp->private_data;
	
	
	if(down_interruptible(&(dev->rbuf->sem))) {
	  return -ERESTARTSYS;
	}
	
	while (dev->rbuf->count == 0) {
	  /* buffer is empty, prepare for sleep */
	  up(&(dev->rbuf->sem));
	  wake_up_interruptible(&(dev->rbuf->queue));
	  if (filp->f_flags & O_NONBLOCK) {
	    return -EAGAIN;
	  }
	  /* Sleepy time! Put process to sleep and wait for more data. */
	  if (wait_event_interruptible(dev->rbuf->queue, (dev->rbuf->count != 0))) {
	    return -ERESTARTSYS;
	  }
	  /* reacquire lock and loop */
	  if(down_interruptible(&(dev->rbuf->sem))) {
	  return -ERESTARTSYS;
	  }
	}
	while((data < count) && dev->rbuf->count > 0) {
	  /* loop and read until the specified amount is read or the buffer is empty, one character at a time */
    ret = copy_to_user(buf+data,dev->rbuf->buffer+dev->rbuf->rp,1);
  	if (ret) {
  	  up(&(dev->rbuf->sem));
  	  return -EFAULT;
  	}
  	data++;
  	dev->rbuf->rp++;
  	dev->rbuf->count--;
  	if(dev->rbuf->rp == dev_buffer_size) {
  	  dev->rbuf->rp = 0;
  	}
	}
	
	up(&(dev->rbuf->sem));
	/* Data has been cleared from buffer! Wake up any sleepy processes */
	wake_up_interruptible(&(dev->rbuf->queue));
	
	return count;
}

/* Called when a process writes to dev file */
static ssize_t dm510_write( struct file *filp,
    const char *buf,/* The buffer to get data from      */
    size_t count,   /* The max number of bytes to write */
    loff_t *f_pos )  /* The offset in the file           */
{

	int data = 0;
	struct my_device *dev = filp->private_data;
	
	if(down_interruptible(&(dev->wbuf->sem))) {
	  return -ERESTARTSYS;
	}

	while (data < count) {
	  /* not all data is written to buffer yet */ 
	  while(dev->wbuf->count == dev_buffer_size) {
	    /* buffer is full, prepare for sleep */
	    up(&(dev->wbuf->sem));
	    /* data is in buffer, wake up any sleepy processes */
	    wake_up_interruptible(&(dev->wbuf->queue));
	    
	    if(filp->f_flags & O_NONBLOCK) {
	      return -EAGAIN;
	    }
	    /* Sleepy time! Put process to sleep and wait for more space in buffer. */
	    if(wait_event_interruptible(dev->wbuf->queue, (dev->wbuf->count < dev_buffer_size))) {
	      return -ERESTARTSYS;
	    }
	    /* reacquire lock and loop */
	    if (down_interruptible(&(dev->wbuf->sem))) {
	      return -ERESTARTSYS;
	    }
	  }
	  /* loop and write until there's no more data or no more space, one character at a time */
	  ret = copy_from_user(dev->wbuf->buffer + dev->wbuf->wp, buf + data, 1);
  	if(ret) {
  	  up(&(dev->wbuf->sem));
  	  return -EFAULT;
  	}
  	data++;
  	dev->wbuf->wp++;
  	dev->wbuf->count++;
  	if(dev->wbuf->wp == dev_buffer_size) {
  	  dev->wbuf->wp = 0;
  	}
	}
	up(&(dev->wbuf->sem));
	/* data has been written to buffer, wake up any sleepy processes */
	wake_up_interruptible(&dev->wbuf->queue);
	
	return data;
}

/* called by system call icotl */ 
long dm510_ioctl( 
    struct file *filp, 
    unsigned int cmd,   /* command passed from the user */
    unsigned long arg ) /* argument of the command */
{
	/* ioctl code belongs here */
	printk(KERN_INFO "DM510: ioctl called.\n");
	int i;
	int err = 0;
	if (_IOC_TYPE(cmd) != DM510_IOC_MAGIC) return -ENOTTY;
	if (_IOC_NR(cmd) > DM510_IOC_MAXNR) return -ENOTTY;
	/*
	 * the direction is a bitmask, and VERIFY_WRITE catches R/W
	 * transfers. `Type' is user-oriented, while
	 * access_ok is kernel-oriented, so the concept of "read" and
	 * "write" is reversed
	 */
	if (_IOC_DIR(cmd) & _IOC_READ)
		err = !access_ok(VERIFY_WRITE, (void __user *)arg, _IOC_SIZE(cmd));
	else if (_IOC_DIR(cmd) & _IOC_WRITE)
		err =  !access_ok(VERIFY_READ, (void __user *)arg, _IOC_SIZE(cmd)); 
	if (err) return -EFAULT;
  
	switch(cmd) {
    
    case DM510_IOCBUFFERSET:
    	dev_buffer_size = arg;
    	for (i = 0; i<DEVICE_COUNT; i++) {
    	  kfree(buffers[i].buffer);
    	  buffers[i].buffersize = dev_buffer_size;
    	  buffers[i].buffer = kmalloc(dev_buffer_size, GFP_KERNEL);
    	  if(!buffers[i].buffer) {
    	    printk(KERN_NOTICE "Unable to allocate memory! Out of memory!\n");
          printk("errno = -12 ENOMEM\n");
          return -ENOMEM;
        }
    	}
    	break;
    	
    case DM510_IOCBUFFERGET:
      return dev_buffer_size;
      
    case DM510_IOCPROCSET:
      dev_number_proc = arg;
      break;
		
		case DM510_IOCPROCGET:
      return dev_number_proc;
		
		default:
		  return -ENOTTY;
	}
	//return retval;

	return 0; //has to be changed
}

module_init( dm510_init_module );
module_exit( dm510_cleanup_module );

MODULE_AUTHOR( "Torvald Johnson" );
MODULE_LICENSE("GPL");