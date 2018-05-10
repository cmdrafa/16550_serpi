/*                                                     
 * $Id: serpi.c
 */
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/kernel.h> /* printk() */
#include <linux/slab.h>   /* kmalloc() */
#include <linux/fs.h>     /* everything... */
#include <linux/errno.h>  /* error codes */
#include <linux/types.h>  /* size_t */
#include <linux/proc_fs.h>
#include <linux/fcntl.h> /* O_ACCMODE */
#include <linux/aio.h>
#include <linux/delay.h>
#include <linux/timer.h>
#include <asm/uaccess.h>
#include <linux/kdev_t.h>
#include <asm/io.h>
#include <linux/ioctl.h>
#include <linux/interrupt.h>
#include <linux/cdev.h>
#include <linux/workqueue.h>
#include <linux/kfifo.h>
#include <linux/ioport.h>
#include <linux/sched.h>
#include <linux/poll.h>
#include <linux/wait.h>
#include <asm/semaphore.h>
#include <linux/spinlock.h>
#include "serial_reg.h"

#define FIFO_SIZE 4096

MODULE_LICENSE("Dual BSD/GPL");
MODULE_AUTHOR("Rafael Kraemer");

struct dev
{
    dev_t uartdevice;
    spinlock_t lock;           // the device
    wait_queue_head_t w_queue; // the wait_queue
    struct cdev cdev;          //cdev struct
    struct semaphore sem;      //a semaphore
    char *data;                // device data
    char *devname;             // device name
    int cnt;                   // A count
    int timer_state;           //
    int irq;                   // Irq identifier
    int wq_flag;               //Wait queue flag
};

struct dev *uartdev;

static struct timer_list read_timer;

static struct kfifo *dev_fifo;

void configure_serpi_device(void);
irqreturn_t int_handler(int irq, void *dev_id);

int serpi_open(struct inode *inodep, struct file *filep)
{
    int ret;
    struct dev *uartdev;

    uartdev = container_of(inodep->i_cdev, struct dev, cdev);
    filep->private_data = uartdev;

    ret = nonseekable_open(inodep, filep);

    printk(KERN_INFO "Device has been opened\n");

    return 0;
}

void timer_callback(unsigned long data)
{
    printk(KERN_ALERT "Timer callback\n");
    uartdev->timer_state = 1;
}

int serpi_release(struct inode *inodep, struct file *filep)
{
    int ret;

    ret = del_timer(&read_timer);

    printk(KERN_INFO "Device has been sucessfully closed\n");

    return 0;
}

// Wait for signal from int_handler that there is data to read
// copy to user from kfifo, gotta figure out how to work that shit
ssize_t serpi_read(struct file *filep, char __user *buff, size_t count, loff_t *offp)
{
    unsigned long cp, uncp;
    int i = 0;
    int ret;
    //unsigned char escape = 0;
    //int data_read = 0;

    struct dev *uartdev = filep->private_data;
    if (down_interruptible(&uartdev->sem))
    {
        return -ERESTARTSYS;
    }

    uartdev->data = kmalloc(sizeof(char) * (count + 1), GFP_KERNEL);
    if (!uartdev->data)
    {
        printk(KERN_ERR "Error allocating memory for the read operation!!\n");
        return -1;
    }
    memset(uartdev->data, 0, sizeof(char) * (count + 1));

    ret = wait_event_interruptible(uartdev->w_queue, uartdev->wq_flag != 0);
    uartdev->wq_flag = 0;

    if (kfifo_len(dev_fifo) != 0)
    {
        msleep_interruptible(3);
        cp = kfifo_get(dev_fifo, (uartdev->data), count);
    }

    uncp = copy_to_user(buff, uartdev->data, count);
    kfree(uartdev->data);

    up(&uartdev->sem);
    return cp;
}

ssize_t serpi_write(struct file *filep, const char __user *buff, size_t count, loff_t *offp)
{

    return count;
}

struct file_operations uart_fops = {
    .owner = THIS_MODULE,
    .llseek = no_llseek,
    .open = serpi_open,
    .read = serpi_read,
    .write = serpi_write,
    .release = serpi_release,
};

/** Deal with basically two issues:
 * Transfer data to/from the UART and
 * signal any user process that may be blocked or
 * waiting for the event that caused the interrupt **/
irqreturn_t int_handler(int irq, void *dev_id)
{
    unsigned char buf, ret;

    if ((inb(BASE + UART_IIR) & UART_IIR_RDI) != 0)
    {
        buf = inb(BASE + UART_RX);
        //ret = mod_timer(&read_timer, jiffies + msecs_to_jiffies(2000));
        kfifo_put(dev_fifo, &buf, sizeof(unsigned char));
        uartdev->wq_flag = 1;
        wake_up_interruptible(&uartdev->w_queue);
    }
    uartdev->timer_state = 0;

    return IRQ_HANDLED;
}

static int serpi_init(void)
{
    int ret, Major, Minor, reg, req;

    uartdev = kmalloc(sizeof(struct dev), GFP_KERNEL);
    if (!uartdev)
    {
        printk(KERN_ERR "Failled allocating memory for the serpi device!\n");
        return -1;
    }
    memset(uartdev, 0, sizeof(struct dev));
    spin_lock_init(&uartdev->lock);

    dev_fifo = kfifo_alloc(FIFO_SIZE, GFP_KERNEL, &uartdev->lock);

    init_waitqueue_head(&uartdev->w_queue);
    uartdev->devname = "serpi";
    uartdev->irq = 4;
    init_MUTEX(&uartdev->sem);

    if (!request_region(BASE, 8, uartdev->devname))
    {
        printk(KERN_ERR "Request_region failed !!\n");
        return -1;
    }

    configure_serpi_device();

    // Allocate Major Numbers
    ret = alloc_chrdev_region(&uartdev->uartdevice, 0, 1, uartdev->devname);
    if (ret < 0)
    {
        printk(KERN_ERR "Major number allocation failed!\n");
        return ret;
    }
    Major = MAJOR(uartdev->uartdevice);
    Minor = MINOR(uartdev->uartdevice);
    printk(KERN_INFO "Allocated Major number: %d\n", Major);

    //Register after allocation
    cdev_init(&uartdev->cdev, &uart_fops);
    uartdev->cdev.owner = THIS_MODULE;
    uartdev->cdev.ops = &uart_fops;

    reg = cdev_add(&uartdev->cdev, uartdev->uartdevice, 1);
    if (reg < 0)
    {
        printk(KERN_ERR "Error in cdev_add\n");
    }
    // Request IRQ line
    req = request_irq(uartdev->irq, int_handler, SA_INTERRUPT,
                      uartdev->devname, &uartdev);
    if (req < 0)
    {
        printk(KERN_ERR "Failed allocating interrupt line!\n");
        return req;
    }

    setup_timer(&read_timer, timer_callback, 0);
    uartdev->timer_state = 0;

    return 0;
}

void configure_serpi_device()
{
    unsigned char lcr = 0;
    lcr = UART_IER_RDI;         // Enable receiver interrupt
    outb(lcr, BASE + UART_IER); // write to it
    lcr = 0;
    lcr = UART_LCR_WLEN8 | UART_LCR_EPAR | UART_LCR_STOP; //Set len to 8, Even parity and 2 stop bits
    outb(lcr, BASE + UART_LCR);                           // write to it
    lcr |= UART_LCR_DLAB;                                 // Select d_dlab
    outb(lcr, BASE + UART_LCR);                           // Acess dlab
    outb(UART_DIV_1200, BASE + UART_DLL);                 // 1200bps least significant bits
    outb(0, BASE + UART_DLM);                             // 1200bps most significant bits
    lcr &= ~UART_LCR_DLAB;                                // reset DLAB
    outb(lcr, BASE + UART_LCR);                           // write to it
}

static void serpi_exit(void)
{
    int Major;

    Major = MAJOR(uartdev->uartdevice);
    free_irq(uartdev->irq, &uartdev);
    cdev_del(&uartdev->cdev);
    unregister_chrdev_region(uartdev->uartdevice, 1);
    kfree(uartdev);
    kfifo_free(dev_fifo);
    release_region(BASE, 8);

    printk(KERN_INFO "Major number: %d unloaded\n", Major);
}

module_init(serpi_init);
module_exit(serpi_exit);
