#include <getopt.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/types.h>
#include <linux/spi/spidev.h>

#define uint8_t unsigned char
#define uint16_t unsigned short
#define uint32_t unsigned int

#define ARRAY_SIZE(a) (sizeof(a) / sizeof((a)[0]))
static void pabort(const char *s)
{
      perror(s);
      abort();
}

static void usage(char *name) {
        fprintf(stderr, "Usage: %s -B bus -S slave -a [cpha 0|1] -l [cpol 0|1] -s speed -b bits buf\n",
               name);
		exit(1);
}

uint8_t tx[256] = {      //定义待发送的数据
             0x01, 0x20, 0x00, 0x00, 0x00, 0x00,
             0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
             0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
             0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
             0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
             0x00, 0x00,
};



static char device[20]; //设备名
static uint8_t mode;

static uint32_t bus = 7;
static uint32_t slave = 0;
static uint32_t cpol = 0;
static uint32_t cpha = 1;
static uint32_t speed = 4500000;
static uint32_t bits = 8;

static struct option opts[] = {
	{"bus", required_argument, NULL, 'B'},
	{"slave", required_argument, NULL, 'S'},
	{"cpha", required_argument, NULL, 'a'},
	{"cpol", required_argument, NULL, 'l'},
	{"speed", required_argument, NULL, 's'},
	{"bits", required_argument, NULL, 'b'},
	{"help", no_argument, NULL, 'h'},
};

static uint16_t delay;
static void transfer(int fd)
{
      int ret;

      uint8_t rx[ARRAY_SIZE(tx)] = {0, };
      struct spi_ioc_transfer tr = {
             .tx_buf = (unsigned long)tx,   //定义发送缓冲区指针
             .rx_buf = (unsigned long)rx,   //定义接收缓冲区指针
             .len = ARRAY_SIZE(tx),                    
             .delay_usecs = delay,
             .speed_hz = speed,
             .bits_per_word = bits,
      };

      ret = ioctl(fd, SPI_IOC_MESSAGE(1), &tr);//执行spidev.c中ioctl的default进行数据传输
      if (ret == 1)
             pabort("can't send spi message");
	  printf("spi rx:\n");
      for (ret = 0; ret < ARRAY_SIZE(tx); ret++) {
             if (!(ret % 32))
                    puts("");
             printf("%.2X ", rx[ret]);      //打印接收到的数据
      }
      puts("");
}

int main(int argc, char *argv[])
{

      int ret = 0;
      int fd;
      char data;
      int c;
      int i;

      while((c = getopt_long(argc, argv, "B:S:a:l:s:b:h", opts, NULL)) != -1) {

        switch (c) {
        case 'B':
            bus = atol(optarg);
            break;
        case 'S':
            slave = atol(optarg);
            break;
        case 'a':
	    	cpha = atol(optarg);
            break;
        case 'l':
	    	cpol = atol(optarg);
            break;
        case 's':
            speed = atol(optarg);
            break;
        case 'b':
            bits = atol(optarg);
            break;
        case 'h':
			usage(argv[0]);
		case '?':
			printf("not support option %c\n", optopt);
			usage(argv[0]);
            break;
        }
      }


      if (cpha) {
      	mode |= SPI_CPHA;
      }
      if (cpol) {
      	mode |= SPI_CPOL;
      }
      mode &= ~SPI_CS_HIGH;

	  printf("argc %d\n", argc);
	  printf("optind %d\n", optind);
	  if (argc > optind) {
      	for (i = 0; i < argc - optind; i++) {
	     	sscanf(argv[optind + i], "%x", &data);
	      	tx[i] = data;
      	}
	  }
      printf("spi tx:\n");
      for (i = 0; i < 256; i++) {
             if (!(i % 32))
                    puts("");
             printf("%.2X ", tx[i]);      //打印接收到的数据
 
      }
      printf("\n");

      //sprintf(device, "/dev/spidev%d.%d", bus, slave);
      fd = open("/dev/spidev0.1", O_RDWR);       //打开"/dev/spidev1.0"
      if (fd < 0)
             pabort("can't open device");

      ret = ioctl(fd, SPI_IOC_WR_MODE, &mode);  //SPI模式设置可写
      if (ret == -1)
            pabort("can't set spi mode");
            ret = ioctl(fd, SPI_IOC_RD_MODE, &mode); //SPI模式设置可读

            if (ret == -1)
                  pabort("can't get spi mode");

      ret = ioctl(fd, SPI_IOC_WR_BITS_PER_WORD, &bits);  //SPI的bit/word设置可写
      if (ret == -1)
             pabort("can't set bits per word");

      ret = ioctl(fd, SPI_IOC_RD_BITS_PER_WORD, &bits);   //SPI的bit/word设置可读
      if (ret == -1)
             pabort("can't get bits per word");

      ret = ioctl(fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed);     //SPI的波特率设置可写
      if (ret == -1)
             pabort("can't set max speed hz");

      ret = ioctl(fd, SPI_IOC_RD_MAX_SPEED_HZ, &speed);   //SPI的波特率设置可读
      if (ret == -1)
             pabort("can't get max speed hz");

      printf("spi mode: %d\n", mode);
      printf("bits per word: %d\n", bits);
      printf("max speed: %d Hz (%d KHz)\n", speed, speed/1000);
      transfer(fd);                                                        //数据传输
      close(fd);

      return ret;

}
