#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <fcntl.h>
#include <string.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <stdbool.h>

#include "serpi.h"

#define buffersize 4096

int set_ioctl(int fd)
{
    struct ioctl_serpi args;
    system("clear");
    fflush(stdin);

    printf("Lets do some config of the serial port: \n");

    do
    {
        printf("Enable Fifo:\n 1)No 2)1 byte Fifo 3)4 bytes Fifo 4)8 bytes Fifo 5)14 bytes Fifo\n");
        scanf("%d", &args.fifo);
    } while (&args.fifo == NULL);

    do
    {
        printf("Choose a bitrate:\n1)1200bps 2)9600bps 3)19200bps 4)28800bps 5)38400bps 6)57600bps 7)115200bps\n");
        scanf("%d", &args.br);
    } while (&args.br == NULL);

    do
    {
        printf("Choose the Wordlenght:\n 1)8 bits \t 2)7 bits \t 3)6 bits \t 4)5 bits\n");
        scanf("%d", &args.wlen);
    } while (&args.wlen == NULL);

    do
    {
        printf("Choose the Parity scheme:\n 1)Stick \t 2)Even \t \n");
        scanf("%d", &args.par);
    } while (&args.par == NULL);

    do
    {
        printf("Choose the number of Stop bits:\n 1) 1 \t 2) 2 \t \n");
        scanf("%d", &args.nb);
    } while (&args.nb == NULL);

    int io = ioctl(fd, SERPI_IOCSALL, &args);
    if (io < 0)
    {
        perror("Setting ioctl: ");
        return -1;
    }

    return 0;
}

int get_ioctl(int fd)
{
    struct ioctl_serpi args;
    char option;

    int io_g = ioctl(fd, SERPI_IOCGALL, &args);
    if (io_g < 0)
    {
        perror("Getting ioctl params: ");
        return -1;
    }

    while (option != '0')
    {
        system("clear");
        fflush(stdin);

        printf("\n \t      DEVICE PARAMETERS \n");
        printf("***************************************\n");
        if (args.fifo == 1)
        {
            printf("No fifo enabled\n");
        }
        else if (args.fifo == 2)
        {
            printf("Fifo: 1 byte\n");
        }
        else if (args.fifo == 3)
        {
            printf("Fifo: 4 bytes\n");
        }
        else if (args.fifo == 4)
        {
            printf("Fifo: 8 bytes\n");
        }
        else
        {
            printf("Fifo: 14 bytes\n");
        }

        if (args.br == 1)
        {
            printf("Bitrate: 1200 bps\n");
        }
        else if (args.br == 2)
        {
            printf("Bitrate: 9600 bps\n");
        }
        else if (args.br == 3)
        {
            printf("Bitrate: 19200 bps\n");
        }
        else if (args.br == 4)
        {
            printf("Bitrate: 28800 bps\n");
        }
        else if (args.br == 5)
        {
            printf("Bitrate: 38400 bps\n");
        }
        else if (args.br == 6)
        {
            printf("Bitrate: 57600 bps\n");
        }
        else
        {
            printf("Bitrate: 115200 bps\n");
        }

        if (args.wlen == 1)
        {
            printf("Wordlenght: 8 bits\n");
        }
        else if (args.wlen == 2)
        {
            printf("Wordlenght: 7 bits\n");
        }
        else if (args.wlen == 3)
        {
            printf("Wordlenght: 6 bits\n");
        }
        else
        {
            printf("Wordlenght: 5 bits\n");
        }

        if (args.par == 1)
        {
            printf("Parity: Stick\n");
        }
        else
        {
            printf("Parity: Even\n");
        }

        if (args.nb == 1)
        {
            printf("Stop bits: 1\n");
        }
        else
        {
            printf("Stop bits: 2\n");
        }
        printf("***************************************\n");
        printf("Type '0' to go back to the menu\n");
        option = getchar();
    }

    return 0;
}

int read_serpi(fd)
{
    char *buf = malloc(buffersize + 2);
    char *q = "back\n";
    char *q_2 = "0";
    system("clear");
    fflush(stdin);

    printf("Send the string 'back' or '0' to get back to the menu!\n");
    while (1)
    {
        int rc_w = read(fd, buf, buffersize);
        if (rc_w < 0)
        {
            perror("Reading FD: ");
            return -1;
        }
        if (strcmp(buf, "") != 0)
        {
            printf("Received String: %s\n", buf);
        }
        if (strcmp(buf, q) == 0 || strcmp(buf, q_2) == 0)
        {
            break;
        }
    }

    free(buf);

    return 0;
}

int write_serpi(fd)
{
    char *buf = malloc(buffersize + 2);
    char *q = "back\n";
    char *q_2 = "0\n";
    system("clear");
    fflush(stdin);

    printf("Type string to send or 'back' or '0' to go back to the menu!\n");
    while (1)
    {
        fgets(buf, buffersize + 2, stdin);
        int len = strlen(buf);
        int rc_w = write(fd, buf, len);
        if (rc_w < 0)
        {
            perror("error writing to the device: ");
            return -1;
        }
        if (strcmp(buf, q) == 0 || strcmp(buf, q_2) == 0)
        {
            break;
        }
    }

    free(buf);

    return 0;
}

int main(int argc, char *argv[])
{
    if (argc < 2)
    {
        printf("Error!!\nYou must pass the device address as the second argument\n");
        return -1;
    }

    char *dev = argv[1];
    char option;
    bool isRunning = true;

    int fd = open(dev, O_RDWR);
    if (fd < 0)
    {
        perror("Opening device: ");
        return -1;
    }

    while (isRunning == true)
    {
        system("clear");
        fflush(stdin);
        puts("\n \t      COMMUNICATION WITH A UART DEVICE"
             "\n \t            FEUP - SO-2017/2018"
             "\n \t            SERPI DEVICE DRIVER"
             "\n***************************************************************"
             "\n[1]Configure the communcation parameters for the device (ioctl)"
             "\n[2]Get current configuration for the device (ioctl)"
             "\n[3]Read from the serial port"
             "\n[4]Write to the serial port"
             "\n[5]Exit"
             "\n***************************************************************");
        option = getchar();

        switch (option)
        {
        case '1':
            set_ioctl(fd);
            break;
        case '2':
            get_ioctl(fd);
            break;
        case '3':
            read_serpi(fd);
            break;
        case '4':
            write_serpi(fd);
            break;
        case '5':
            isRunning = false;
            printf("Goodbye\n");
            break;
        }
    }

    close(fd);

    return 0;
}
