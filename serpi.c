/*                                                     
 * $Id: serpi.c
 */
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/errno.h>
#include <linux/types.h>
#include <linux/proc_fs.h>
#include <linux/fcntl.h>
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
#include <linux/ioctl.h>

#include "serial_reg.h"
#include "serpi.h"

#define FIFO_SIZE 4096 // The size in bytes of the fifo structure

MODULE_LICENSE("Dual BSD/GPL");
MODULE_AUTHOR("Rafael Kraemer");

struct dev
{
    dev_t uartdevice;                   // the device
    spinlock_t lock;                    // A spinlock used in the kfifo struct
    wait_queue_head_t r_queue, w_queue; // the read_queue and write_queue decs
    struct cdev cdev;                   //cdev struct
    struct semaphore sem;               //a semaphore
    char *data;                         // device data
    char *devname;                      // device name
    int cnt;                            // A count
    int timer_state;                    // a variable for checking the timer
    int irq;                            // Irq identifier = 4
    int rq_flag;                        // Wait queue flag, reading
    int wq_flag;                        // Wait queue flag, writing
    atomic_t serpi_available;           // Flag for checking device availability
    int serpi_owner;                    // Access control var
    int serpi_count;                    // Access control var
    struct kfifo *dev_fifo;             // The fifo structure
};

struct dev *uartdev;             // the device structure
struct ioctl_serpi *ioctl_serpi; // The ioctl structure, command list

static struct timer_list read_timer; // the timer

// Prototypes
void configure_serpi_device(void);
void write_work(void);

/**
 * set/get the values of the communication parameters, i.e. the bitrate, 
 * the char width, the parity and the number of bits.
 * implement a single "set" command that allows to modify all the parameters. 
 * Furthermore, you should provide a "get" command that allows to read the current value of all the 
 * serial communication parameters.**/
int serpi_ioctl(struct inode *inode, struct file *filep, unsigned int cmd, unsigned long arg)
{

    char lcr_w, lcr_par, lcr_stop, lcr_br, lcr;
    int ret1, ret2;

    // Check for access to user_space args;
    if (!access_ok(VERIFY_WRITE, (void __user *)arg, _IOC_SIZE(cmd)))
    {
        return -EFAULT;
    }

    switch (cmd)
    {
    // Set commands
    case SERPI_IOCSALL:

        ret1 = copy_from_user(ioctl_serpi, (struct ioctl_serpi *)arg, sizeof(struct ioctl_serpi));

        lcr = UART_IER_RDI | UART_IER_THRI; // Enable receiver interrupt and
                                            //Transmitter holding register interrupt
        outb(lcr, BASE + UART_IER);

        switch (ioctl_serpi->wlen)
        {
        case 1:
            lcr_w = UART_LCR_WLEN8;
            break;
        case 2:
            lcr_w = UART_LCR_WLEN7;
            break;
        case 3:
            lcr_w = UART_LCR_WLEN6;
            break;
        case 4:
            lcr_w = UART_LCR_WLEN5;
            break;
        default:
            lcr_w = UART_LCR_WLEN8;
        }

        switch (ioctl_serpi->par)
        {
        case 1:
            lcr_par = UART_LCR_SPAR;
            break;
        case 2:
            lcr_par = UART_LCR_EPAR;
            break;
        default:
            lcr_par = UART_LCR_EPAR;
            break;
        }

        switch (ioctl_serpi->nb)
        {
        case 1:
            break;
        case 2:
            lcr_stop = UART_LCR_STOP;
        default:
            lcr_stop = UART_LCR_STOP;
        }
        // Write the wordlenght, the parity and the stop bits
        if (lcr_stop == UART_LCR_STOP)
        {
            lcr = lcr_w | lcr_par | UART_LCR_STOP;
            outb(lcr, BASE + UART_LCR);
        }
        else
        {
            lcr = lcr_w | lcr_par;
            outb(lcr, BASE + UART_LCR);
        }

        switch (ioctl_serpi->br)
        {
        case 1:
            lcr_br = UART_DIV_1200;
            break;
        case 2:
            lcr_br = UART_DIV_9600;
            break;
        case 3:
            lcr_br = UART_DIV_19200;
            break;
        case 4:
            lcr_br = UART_DIV_28800;
            break;
        case 5:
            lcr_br = UART_DIV_38400;
            break;
        case 6:
            lcr_br = UART_DIV_57600;
            break;
        case 7:
            lcr_br = UART_DIV_115200;
            break;
        default:
            lcr_br = UART_DIV_1200;
            break;
        }
        // Write the bit_rate to the serial_port, maybe a routine just for that ?
        lcr |= UART_LCR_DLAB;
        outb(lcr, BASE + UART_LCR);
        outb(lcr_br, BASE + UART_DLL);
        outb(0, BASE + UART_DLM);
        lcr &= ~UART_LCR_DLAB;
        outb(lcr, BASE + UART_LCR);
        break;

    // Get commands
    case SERPI_IOCGALL:
        // Send the config to the user
        ret2 = copy_to_user((struct ioctl_serpi *)arg, ioctl_serpi, sizeof(struct ioctl_serpi));
        break;

    default:
        // Default error for this case
        return -ENOTTY;
    }

    return 0;
}

int serpi_open(struct inode *inodep, struct file *filep)
{
    int ret;
    struct dev *uartdev = container_of(inodep->i_cdev, struct dev, cdev);

    // Check is device is available, SINGLE OPEN OPERATION
    /*if (!atomic_dec_and_test(&uartdev->serpi_available))
    {
        atomic_inc(&uartdev->serpi_available);
        return -EBUSY; //already open
    }*/

    filep->private_data = uartdev;
    ret = nonseekable_open(inodep, filep);

    // Check if the user is the current owner
    spin_lock(&uartdev->lock);
    if (uartdev->serpi_count &&
        (uartdev->serpi_owner != current->uid) &&
        (uartdev->serpi_owner != current->euid) &&
        !capable(CAP_DAC_OVERRIDE))
    {
        spin_unlock(&uartdev->lock);
        return -EBUSY;
    }

    if (uartdev->serpi_count == 0)
    {
        uartdev->serpi_owner == current->uid;
    }
    uartdev->serpi_count++;
    spin_unlock(&uartdev->lock);

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

    // atomic_inc(&uartdev->serpi_available); // Single open operation

    spin_lock(&uartdev->lock);
    uartdev->serpi_count--;
    spin_unlock(&uartdev->lock);

    printk(KERN_INFO "Device has been sucessfully closed\n");

    return 0;
}

// Wait for signal from int_handler that there is data to read
// copy to user from kfifo
ssize_t serpi_read(struct file *filep, char __user *buff, size_t count, loff_t *offp)
{
    unsigned long cp = 0, uncp;
    int ret;

    struct dev *uartdev = filep->private_data;

    // If semaphore fails
    if (down_interruptible(&uartdev->sem))
    {
        return -ERESTARTSYS;
    }

    // Allocate memory for the reading data
    uartdev->data = kmalloc(sizeof(char) * (count + 1), GFP_KERNEL);
    if (!uartdev->data)
    {
        printk(KERN_ERR "Error allocating memory for the read operation!!\n");
        return -ENOMEM;
    }
    memset(uartdev->data, 0, sizeof(char) * (count + 1));

    // Wait for signal from the int_handler
    ret = wait_event_interruptible(uartdev->r_queue, uartdev->rq_flag != 0);
    uartdev->rq_flag = 0;

    // Get data from the kfifo buffer
    if (kfifo_len(uartdev->dev_fifo) != 0)
    {
        msleep_interruptible(3);
        cp = kfifo_get(uartdev->dev_fifo, (uartdev->data), count);
    }

    uncp = copy_to_user(buff, uartdev->data, count);
    kfree(uartdev->data);

    if (uncp == 0)
    {
        up(&uartdev->sem);
        return cp;
    }
    else
    {
        up(&uartdev->sem);
        return -EFAULT;
    }
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

    // If semaphore fails
    if (down_interruptible(&uartdev->sem))
    {
        return -ERESTARTSYS;
    }

    // Allocate memory for the data coming from the user_space
    uartdev->data = kmalloc(sizeof(char) * (count + 1), GFP_KERNEL);
    if (!uartdev->data)
    {
        printk(KERN_ERR "Error aloccating memory for write operation!!\n");
        return -ENOMEM;
    }
    memset(uartdev->data, 0, sizeof(char) * (count + 1));
    uartdev->cnt = count;

    // Wait for signal from the interrupt line
    ret = wait_event_interruptible(uartdev->w_queue, uartdev->wq_flag != 0);
    uartdev->wq_flag = 0;

    uncp = copy_from_user(uartdev->data, buff, count);
    udelay(100);
    cp = kfifo_put(uartdev->dev_fifo, (uartdev->data), count);

    outb((uartdev->data[0]), BASE + UART_TX); // Gotta write the first char here

    kfree(uartdev->data);
    if (uncp == 0)
    {
        up(&uartdev->sem);
        return count;
    }
    else
    {
        up(&uartdev->sem);
        return -EFAULT;
    }
}

struct file_operations uart_fops = {
    .owner = THIS_MODULE,
    .llseek = no_llseek,
    .open = serpi_open,
    .read = serpi_read,
    .write = serpi_write,
    .ioctl = serpi_ioctl,
    .release = serpi_release,
};

// The actual write work, called from the interrupt handler
void write_work()
{
    int i;
    int cp;
    char buf_w[uartdev->cnt];

    // Blocking until all data from the fifo is passed to the UART_TX
    cp = kfifo_get(uartdev->dev_fifo, buf_w, uartdev->cnt);
    if (cp > 0)
    {
        for (i = 1; i < uartdev->cnt; i++) // The "0" was already written in
        //the write routine
        {
            outb(buf_w[i], BASE + UART_TX);
            udelay(200); // Small delay, necessary for clearing bits from the registers
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
        kfifo_put(uartdev->dev_fifo, &buf, sizeof(unsigned char));

        uartdev->rq_flag = 1;
        wake_up_interruptible(&uartdev->r_queue);
    }

    return IRQ_HANDLED;
}

static int serpi_init(void)
{
    int ret, Major, Minor, reg, req;

    // Allocate memory for the device struct
    uartdev = kmalloc(sizeof(struct dev), GFP_KERNEL);
    if (!uartdev)
    {
        printk(KERN_ERR "Failled allocating memory for the serpi device!\n");
        return -ENOMEM;
    }
    memset(uartdev, 0, sizeof(struct dev));

    uartdev->devname = "serpi";
    uartdev->irq = 4;

    // Allocate for the ioctl struct
    ioctl_serpi = kmalloc(sizeof(struct ioctl_serpi), GFP_KERNEL);
    if (!ioctl_serpi)
    {
        printk(KERN_ERR "Failed allocating memory for the ioctl interface!\n");
        return -ENOMEM;
    }
    memset(ioctl_serpi, 0, sizeof(struct ioctl_serpi));

    // Init the spin lock, necessary for the kfifo
    spin_lock_init(&uartdev->lock);

    //Allocate the kfifo structure
    uartdev->dev_fifo = kfifo_alloc(FIFO_SIZE, GFP_KERNEL, &uartdev->lock);

    // Init routines, queues, mutex, etc
    //atomic_set(&uartdev->serpi_available, 1);
    init_waitqueue_head(&uartdev->r_queue); // the read queue
    init_waitqueue_head(&uartdev->w_queue); // The write queue
    init_MUTEX(&uartdev->sem);              // The sempaphore

    if (!request_region(BASE, 8, uartdev->devname))
    {
        printk(KERN_ERR "Request_region failed !!\n");
        return -1;
    }

    // Use the default configurations first, user may change by ioctl()
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
        return reg;
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

    // Default values in the structure
    ioctl_serpi->br = 1;   // 9600 bps
    ioctl_serpi->wlen = 1; // Wlen 8
    ioctl_serpi->par = 2;  // Even parity
    ioctl_serpi->nb = 2;   // Two stop bits

    // Configure it

    lcr = UART_IER_RDI | UART_IER_THRI; // Enable receiver interrupt and
                                        //Transmitter holding register interrupt
    outb(lcr, BASE + UART_IER);         // write to it

    lcr = UART_LCR_WLEN8 | UART_LCR_EPAR | UART_LCR_STOP; //Set len to 8, Even parity and 2 stop bits
    outb(lcr, BASE + UART_LCR);                           // write to it

    lcr |= UART_LCR_DLAB;                 // Select d_dlab
    outb(lcr, BASE + UART_LCR);           // Acess dlab
    outb(UART_DIV_1200, BASE + UART_DLL); // 1200bps least significant bits
    outb(0, BASE + UART_DLM);             // 1200bps most significant bits

    lcr &= ~UART_LCR_DLAB;      // reset DLAB
    outb(lcr, BASE + UART_LCR); // write to it
}

static void serpi_exit(void)
{
    int Major;

    Major = MAJOR(uartdev->uartdevice);
    free_irq(uartdev->irq, &uartdev);
    cdev_del(&uartdev->cdev);
    unregister_chrdev_region(uartdev->uartdevice, 1);
    kfree(ioctl_serpi);
    kfifo_free(uartdev->dev_fifo);
    kfree(uartdev);

    release_region(BASE, 8);

    printk(KERN_INFO "Major number: %d unloaded\n", Major);
}

module_init(serpi_init);
module_exit(serpi_exit);
