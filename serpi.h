/** Here are the ioctl definitions for the serpi device driver **/
// Magic = 0x66

/**
 * 'S': set
 * 'G: get 
 * **/

struct ioctl_serpi {
    int br; // Bit rate
    int wlen; // Word-lenght
    int par; // Parity
    int nb; // Stop bits
    int fifo; // Fifo Select
};

#define SERPI_IOC_MAGIC 0x66

//#define SERPI_IOCRESET _IO(SERPI_IOC_MAGIC, 0) // reset

#define SERPI_IOCSALL _IOW(SERPI_IOC_MAGIC, 1, int) // setting all

/*#define SERPI_IOCSCW _IOW(SERPI_IOC_MAGIC, 2, int) // setting char width

#define SERPI_IOCSPAR _IOW(SERPI_IOC_MAGIC, 3, int) // setting parity

#define SERPI_IOCSNB _IOW(SERPI_IOC_MAGIC, 4, int) // setting number of bits*/

#define SERPI_IOCGALL _IOR(SERPI_IOC_MAGIC, 1, int) // getting all

/*#define SERPI_IOCGCW _IOR(SERPI_IOC_MAGIC, 2, int) // getting char width

#define SERPI_IOCGPAR _IOR(SERPI_IOC_MAGIC, 3, int) // getting parity

#define SERPI_IOCGNB _IOR(SERPI_IOC_MAGIC, 4, int) // getting number of bits*/

#define SERPI_IOC_MAXNR 5
