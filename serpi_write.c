#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <fcntl.h>
#include <string.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include "serpi.h"

#define buffersize 1024

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
    struct ioctl_serpi args;
    args.nb = 0;

    int fd = open(dev, O_RDWR);
    if (fd < 0)
    {
        perror("Opening device: ");
        return -1;
    }

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

    int io = ioctl(fd, SERPI_IOCSALL, &args);
    printf("Return value io: %d \n", io);

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
        printf("Enter data to send to dd or 'quit' to quit: ");
    }

    fd = close(fd);

    free(buf);

    return 0;
}
