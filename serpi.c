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

#define FIFO_SIZE 4096 // Its the size in bytes of the fifo structure

MODULE_LICENSE("Dual BSD/GPL");
MODULE_AUTHOR("Rafael Kraemer");

struct dev
{
    dev_t uartdevice;
    spinlock_t lock;                    // the device
    wait_queue_head_t r_queue, w_queue; // the read_queue and wait_queue decs
    struct cdev cdev;                   //cdev struct
    struct semaphore sem;               //a semaphore
    char *data;                         // device data
    char *devname;                      // device name
    int cnt;                            // A count
    int timer_state;                    // a variable for checking the timer
    int irq;                            // Irq identifier = 4
    int rq_flag;                        // Wait queue flag, reading
    int wq_flag;                        // Wait queue flag, writing
};

struct dev *uartdev; // the device structure

static struct timer_list read_timer; // the timer

static struct kfifo *dev_fifo; // The fifo structure

// Prototypes
void configure_serpi_device(void);
void write_work(void);

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

// Timer, not used for now
void timer_callback(unsigned long data)
{
    printk(KERN_ALERT "Timer callback\n");
    uartdev->timer_state = 1;
}

// close()
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
    unsigned long cp = 0, uncp;
    int ret;

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

    ret = wait_event_interruptible(uartdev->r_queue, uartdev->rq_flag != 0);
    uartdev->rq_flag = 0;

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
    unsigned long cp, uncp;
    char b_data;
    int data_checker;
    int ret;
    int i;
    struct dev *uartdev = filep->private_data;

    data_checker = 0;
    b_data = 0;
    i = 0;

    if (down_interruptible(&uartdev->sem))
    {
        return -ERESTARTSYS;
    }

    // Allocate memory for the data coming from the user_space
    uartdev->data = kmalloc(sizeof(char) * (count + 1), GFP_KERNEL);
    if (!uartdev->data)
    {
        printk(KERN_ERR "Error aloccating memory for write operation!!\n");
        return -1;
    }
    memset(uartdev->data, 0, sizeof(char) * (count + 1));
    uartdev->cnt = count;

    // Wait for signal from the interrupt line
    ret = wait_event_interruptible(uartdev->w_queue, uartdev->wq_flag != 0);
    uartdev->wq_flag = 0;

    uncp = copy_from_user(uartdev->data, buff, count);
    udelay(100);
    cp = kfifo_put(dev_fifo, (uartdev->data), count);

    outb((uartdev->data[0]), BASE + UART_TX); // Gotta write the first char here

    kfree(uartdev->data);

    up(&uartdev->sem);
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

void write_work()
{
    int i;
    int cp;
    char buf_w[uartdev->cnt];

    cp = kfifo_get(dev_fifo, buf_w, uartdev->cnt);
    if (cp > 0)
    {
        for (i = 1; i < uartdev->cnt; i++) // The "0" was already written in
        //the write routine
        {
            outb(buf_w[i], BASE + UART_TX);
            udelay(100); // Small delay, necessary for clearing bits from the registers
        }
    }
    printk(KERN_INFO "%d bytes written\n", uartdev->cnt); // Debug
}

/** Deal with basically two issues:
 * Transfer data to/from the UART and
 * signal any user process that may be blocked or
 * waiting for the event that caused the interrupt **/
irqreturn_t int_handler(int irq, void *dev_id)
{
    unsigned char buf;

    /** Gotta figure a better way to look for interrupts
     * This one looks ugly
     * */

    //Write interrupt
    if ((inb(BASE + UART_IIR) & UART_IIR_THRI) != 0)
    {
        write_work(); // Routine for writing data
        uartdev->wq_flag = 1;
        wake_up_interruptible(&uartdev->w_queue);
    }

    //Read interrupt
    if ((inb(BASE + UART_IIR) & UART_IIR_RDI) != 0)
    {
        // Read data from the register and put it in the buffer
        // Its is done this way because the register is active while
        // there is still data to be written (reading one char a time)
        buf = inb(BASE + UART_RX);
        kfifo_put(dev_fifo, &buf, sizeof(unsigned char));

        uartdev->rq_flag = 1;
        wake_up_interruptible(&uartdev->r_queue);
    }

    return IRQ_HANDLED;
}

static int serpi_init(void)
{
    int ret, Major, Minor, reg, req;

    // Allocate structure for the device
    uartdev = kmalloc(sizeof(struct dev), GFP_KERNEL);
    if (!uartdev)
    {
        printk(KERN_ERR "Failled allocating memory for the serpi device!\n");
        return -1;
    }
    memset(uartdev, 0, sizeof(struct dev));
    uartdev->devname = "serpi";
    uartdev->irq = 4;

    // Init the spin lock, necessary for the kfifo
    spin_lock_init(&uartdev->lock);

    //Allocate the kfifo structure
    dev_fifo = kfifo_alloc(FIFO_SIZE, GFP_KERNEL, &uartdev->lock);

    // Init queues and mutex
    init_waitqueue_head(&uartdev->r_queue); // the read queue
    init_waitqueue_head(&uartdev->w_queue); // The write queue
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

    // Setup the timer
    setup_timer(&read_timer, timer_callback, 0); // Not used for now
    uartdev->timer_state = 0;                    // Not used for now

    return 0;
}

void configure_serpi_device()
{
    unsigned char lcr = 0;
    lcr = UART_IER_RDI | UART_IER_THRI;                   // Enable receiver interrupt
    outb(lcr, BASE + UART_IER);                           // write to it
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
