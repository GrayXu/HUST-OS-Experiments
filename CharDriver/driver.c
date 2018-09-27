#include "linux/kernel.h"  
#include "linux/module.h"  
#include "linux/fs.h"  
#include "linux/init.h"  
#include "linux/types.h"  
#include "linux/errno.h"  
#include "linux/uaccess.h"  
#include "linux/kdev_t.h"  
#define BUFFER_SIZE 1024

static int my_open(struct inode *inode, struct file *file);  
static int my_release(struct inode *inode, struct file *file);
static ssize_t my_read(struct file *file, char __user *user, size_t t, loff_t *f);  
static ssize_t my_write(struct file *file, const char __user *user, size_t t, loff_t *f);

static char message[BUFFER_SIZE] = "Gray's drive";
static int device_num = 0;//
static int counter = 0;//

static char* devname = "gray_driver";

struct file_operations pstruct = {
	.read = my_read,
	.write = my_write,
	.open = my_open,
	.release = my_release
};

int init_module(){
	int ret;
	ret = register_chrdev(0,devname,&pstruct);
	if(ret < 0){
		printk("register failure\n");
		return -1;
	}
	else {
		printk("the device has been registered!\n");  
        device_num = ret;  
        // printk("<1>the virtual device's major number %d.\n", device_num);  
        // printk("<1>Or you can see it by using\n");  
        // printk("<1>------more /proc/devices-------\n");  
        // printk("<1>To talk to the driver,create a dev file with\n");  
        // printk("<1>------'mknod /dev/myDevice c %d 0'-------\n", device_num);  
        // printk("<1>Use \"rmmode\" to remove the module\n");  
        return 0;
	}
}

void cleanup_module(){
	unregister_chrdev(device_num,devname);
	printk("unregistered\n");
}

static int my_open(struct inode *inode,struct file *file){
	printk("<1>main  device : %d\n", MAJOR(inode->i_rdev));  
    printk("<1>slave device : %d\n", MINOR(inode->i_rdev));  
    printk("<1>%d times to call the device\n", ++counter);  
    try_module_get(THIS_MODULE);  
    return 0;
}

static int my_release(struct inode *inode,struct file *file){
	printk("released\n");  
    module_put(THIS_MODULE);  
    return 0;
}

static ssize_t my_write(struct file *file,const char __user *user,size_t t,loff_t *f){
	if(copy_from_user(message,user,sizeof(message)))  
    {  
        return -EFAULT;  
    }  
    return sizeof(message);
}

static ssize_t my_read(struct file *file,char __user *user,size_t t,loff_t *f){
	if(copy_to_user(user,message,sizeof(message)))  
    {  
        return -EFAULT;  
    }  
    return sizeof(message);
}
