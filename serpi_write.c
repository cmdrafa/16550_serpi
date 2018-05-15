#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <fcntl.h>
#include <string.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include "serpi.h"

#define buffersize 1024

int set_ioctl(int fd)
{
    struct ioctl_serpi args;
    args.nb = 0;

    printf("Lets do some config of the serial port: \n");
    do
    {
        printf("Choose a bitrate:\n 1) 9600 bps \t 2)1200 bps\n");
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

    int io_s = ioctl(fd, SERPI_IOCSALL, &args);
    if (io_s < 0)
    {
        perror("Setting ioctl values: ");
    }

    return 0;
}

int get_ioctl(int fd)
{
    struct ioctl_serpi args;
    args.nb = 0;

    int io_g = ioctl(fd, SERPI_IOCGALL, &args);
    if (io_g < 0)
    {
        perror("Getting ioctl params: ");
    }
    if (args.br == 1)
    {
        printf("Bitrate: 9600 bps\n");
    }
    else
    {
        printf("Bitrate: 1200\n");
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
        printf("Stick parity\n");
    }
    else
    {
        printf("Even parity\n");
    }

    return 0;
}

int main(int argc, char *argv[])
{
    if (argc < 2)
    {
        printf("You must pass the device address as the first argument\n");
        return -1;
    }
    char *buf = malloc(buffersize + 2);
    char *dev = argv[1];
    char *q = "quit\n";
    char *g = "get\n";

    int fd = open(dev, O_RDWR);
    if (fd < 0)
    {
        perror("Opening device: ");
        return -1;
    }

    set_ioctl(fd);

    while (fgets(buf, buffersize + 2, stdin) != NULL)
    {
        int len = strlen(buf);
        //printf("LEN %d\n", len);
        int rc_w = write(fd, buf, len);
        if (rc_w < 0)
        {
            perror("error writing: ");
        }
        if (strcmp(buf, q) == 0)
        {
            printf("Goodbye\n");
            break;
        }
        if (strcmp(buf, g) == 0)
        {
            get_ioctl(fd);
        }
        printf("Type string to send, 'quit' to quit or 'get' to get ioctl params: ");
    }
    fd = close(fd);

    free(buf);

    return 0;
}
