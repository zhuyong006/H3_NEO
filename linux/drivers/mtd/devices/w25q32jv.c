/*
 * MTD SPI driver for ST w25qxx (and similar) serial flash chips
 *
 * Author: Mike Lavender, mike@steroidmicros.com
 *
 * Copyright (c) 2005, Intec Automation Inc.
 *
 * Some parts are based on lart.c by Abraham Van Der Merwe
 *
 * Cleaned up and generalized based on mtd_dataflash.c
 *
 * This code is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#include <linux/err.h>
#include <linux/errno.h>
#include <linux/module.h>
#include <linux/device.h>

#include <linux/mtd/mtd.h>
#include <linux/mtd/partitions.h>


#include <linux/spi/spi.h>
#include <linux/spi/flash.h>
#include <linux/mtd/spi-nor.h>

#define	MAX_CMD_SIZE		6

#define SPI_TRANSFER_TEST
#ifdef SPI_TRANSFER_TEST
#include <linux/types.h>
#include <linux/spi/spidev.h>

struct w25q {
	struct spi_device	*spi;
	struct device	*dev;
	struct spi_nor		spi_nor;
	u8			command[MAX_CMD_SIZE];
	char *rx_buffer;
	int rx_len;
	char *tx_buffer;
	int tx_len;
	int speed_hz;
};

static unsigned bufsiz = 4096;
struct w25q* W25Q = NULL;

#else
struct w25q {
	struct spi_device	*spi;
	struct spi_nor		spi_nor;
	u8			command[MAX_CMD_SIZE];
};
#endif

static int w25q32jv_read_reg(struct spi_nor *nor, u8 code, u8 *val, int len)
{
	struct w25q *flash = nor->priv;
	struct spi_device *spi = flash->spi;
	int ret;

	ret = spi_write_then_read(spi, &code, 1, val, len);
	if (ret < 0)
		dev_err(&spi->dev, "error %d reading %x\n", ret, code);

	return ret;
}

static void w25q_addr2cmd(struct spi_nor *nor, unsigned int addr, u8 *cmd)
{
	/* opcode is in cmd[0] */
	cmd[1] = addr >> (nor->addr_width * 8 -  8);
	cmd[2] = addr >> (nor->addr_width * 8 - 16);
	cmd[3] = addr >> (nor->addr_width * 8 - 24);
	cmd[4] = addr >> (nor->addr_width * 8 - 32);
}

static int w25q_cmdsz(struct spi_nor *nor)
{
	return 1 + nor->addr_width;
}

static int w25q32jv_write_reg(struct spi_nor *nor, u8 opcode, u8 *buf, int len)
{
	struct w25q *flash = nor->priv;
	struct spi_device *spi = flash->spi;

	flash->command[0] = opcode;
	if (buf)
		memcpy(&flash->command[1], buf, len);

	return spi_write(spi, flash->command, len + 1);
}

static ssize_t w25q32jv_write(struct spi_nor *nor, loff_t to, size_t len,
			    const u_char *buf)
{
	struct w25q *flash = nor->priv;
	struct spi_device *spi = flash->spi;
	unsigned int inst_nbits, addr_nbits, data_nbits, data_idx;
	struct spi_transfer t[3] = {};
	struct spi_message m;
	int cmd_sz = w25q_cmdsz(nor);
	ssize_t ret;

	/* get transfer protocols. */
	inst_nbits = spi_nor_get_protocol_inst_nbits(nor->write_proto);
	addr_nbits = spi_nor_get_protocol_addr_nbits(nor->write_proto);
	data_nbits = spi_nor_get_protocol_data_nbits(nor->write_proto);

	spi_message_init(&m);

	if (nor->program_opcode == SPINOR_OP_AAI_WP && nor->sst_write_second)
		cmd_sz = 1;

	flash->command[0] = nor->program_opcode;
	w25q_addr2cmd(nor, to, flash->command);

	t[0].tx_buf = flash->command;
	t[0].tx_nbits = inst_nbits;
	t[0].len = cmd_sz;
	spi_message_add_tail(&t[0], &m);

	/* split the op code and address bytes into two transfers if needed. */
	data_idx = 1;
	if (addr_nbits != inst_nbits) {
		t[0].len = 1;

		t[1].tx_buf = &flash->command[1];
		t[1].tx_nbits = addr_nbits;
		t[1].len = cmd_sz - 1;
		spi_message_add_tail(&t[1], &m);

		data_idx = 2;
	}

	t[data_idx].tx_buf = buf;
	t[data_idx].tx_nbits = data_nbits;
	t[data_idx].len = len;
	spi_message_add_tail(&t[data_idx], &m);

	ret = spi_sync(spi, &m);
	if (ret)
		return ret;

	ret = m.actual_length - cmd_sz;
	if (ret < 0)
		return -EIO;
	return ret;
}

/*
 * Read an address range from the nor chip.  The address range
 * may be any size provided it is within the physical boundaries.
 */
static ssize_t w25q32jv_read(struct spi_nor *nor, loff_t from, size_t len,
			   u_char *buf)
{
	struct w25q *flash = nor->priv;
	struct spi_device *spi = flash->spi;
	unsigned int inst_nbits, addr_nbits, data_nbits, data_idx;
	struct spi_transfer t[3];
	struct spi_message m;
	unsigned int dummy = nor->read_dummy;
	ssize_t ret;
	int cmd_sz;

	/* get transfer protocols. */
	inst_nbits = spi_nor_get_protocol_inst_nbits(nor->read_proto);
	addr_nbits = spi_nor_get_protocol_addr_nbits(nor->read_proto);
	data_nbits = spi_nor_get_protocol_data_nbits(nor->read_proto);

	/* convert the dummy cycles to the number of bytes */
	dummy = (dummy * addr_nbits) / 8;

	if (spi_flash_read_supported(spi)) {
		struct spi_flash_read_message msg;

		memset(&msg, 0, sizeof(msg));

		msg.buf = buf;
		msg.from = from;
		msg.len = len;
		msg.read_opcode = nor->read_opcode;
		msg.addr_width = nor->addr_width;
		msg.dummy_bytes = dummy;
		msg.opcode_nbits = inst_nbits;
		msg.addr_nbits = addr_nbits;
		msg.data_nbits = data_nbits;

		ret = spi_flash_read(spi, &msg);
		if (ret < 0)
			return ret;
		return msg.retlen;
	}

	spi_message_init(&m);
	memset(t, 0, (sizeof t));

	flash->command[0] = nor->read_opcode;
	w25q_addr2cmd(nor, from, flash->command);

	t[0].tx_buf = flash->command;
	t[0].tx_nbits = inst_nbits;
	t[0].len = w25q_cmdsz(nor) + dummy;
	spi_message_add_tail(&t[0], &m);

	/*
	 * Set all dummy/mode cycle bits to avoid sending some manufacturer
	 * specific pattern, which might make the memory enter its Continuous
	 * Read mode by mistake.
	 * Based on the different mode cycle bit patterns listed and described
	 * in the JESD216B specification, the 0xff value works for all memories
	 * and all manufacturers.
	 */
	cmd_sz = t[0].len;
	memset(flash->command + cmd_sz - dummy, 0xff, dummy);

	/* split the op code and address bytes into two transfers if needed. */
	data_idx = 1;
	if (addr_nbits != inst_nbits) {
		t[0].len = 1;

		t[1].tx_buf = &flash->command[1];
		t[1].tx_nbits = addr_nbits;
		t[1].len = cmd_sz - 1;
		spi_message_add_tail(&t[1], &m);

		data_idx = 2;
	}

	t[data_idx].rx_buf = buf;
	t[data_idx].rx_nbits = data_nbits;
	t[data_idx].len = min3(len, spi_max_transfer_size(spi),
			       spi_max_message_size(spi) - cmd_sz);
	spi_message_add_tail(&t[data_idx], &m);

	ret = spi_sync(spi, &m);
	if (ret)
		return ret;

	ret = m.actual_length - cmd_sz;
	if (ret < 0)
		return -EIO;
	return ret;
}

#ifdef SPI_TRANSFER_TEST
static ssize_t
spidev_sync(struct w25q *spidev, struct spi_message *message)
{
	DECLARE_COMPLETION_ONSTACK(done);
	int status;
	struct spi_device *spi;

	spi = spidev->spi;

	if (spi == NULL)
		status = -ESHUTDOWN;
	else
		status = spi_sync(spi, message);

	if (status == 0)
		status = message->actual_length;

	return status;
}

static int spidev_message(struct w25q *spidev,
		struct spi_ioc_transfer *u_xfers, unsigned n_xfers)
{
	struct spi_message	msg;
	struct spi_transfer	*k_xfers;
	struct spi_transfer	*k_tmp;
	struct spi_ioc_transfer *u_tmp;
	unsigned		n, total, tx_total, rx_total;
	u8			*tx_buf, *rx_buf;
	int			status = -EFAULT;
	
	spi_message_init(&msg);
	k_xfers = kcalloc(n_xfers, sizeof(*k_tmp), GFP_KERNEL);
	if (k_xfers == NULL)
		return -ENOMEM;

	/* Construct spi_message, copying any tx data to bounce buffer.
	 * We walk the array of user-provided transfers, using each one
	 * to initialize a kernel version of the same transfer.
	 */
	tx_buf = spidev->tx_buffer;
	rx_buf = spidev->rx_buffer;
	total = 0;
	tx_total = 0;
	rx_total = 0;
	for (n = n_xfers, k_tmp = k_xfers, u_tmp = u_xfers;
			n > 0;
			n--, k_tmp++, u_tmp++) {
		k_tmp->len = u_tmp->len;

		total += k_tmp->len;
		/* Since the function returns the total length of transfers
		 * on success, restrict the total to positive int values to
		 * avoid the return value looking like an error.  Also check
		 * each transfer length to avoid arithmetic overflow.
		 */
		if (total > INT_MAX || k_tmp->len > INT_MAX) {
			status = -EMSGSIZE;
			goto done;
		}

		if (u_tmp->rx_buf) {
			/* this transfer needs space in RX bounce buffer */
			rx_total += k_tmp->len;
			if (rx_total > bufsiz) {
				status = -EMSGSIZE;
				goto done;
			}
			k_tmp->rx_buf = (void *)(u_xfers->rx_buf);

			//rx_buf += k_tmp->len;
		}
		if (u_tmp->tx_buf) {
			/* this transfer needs space in TX bounce buffer */
			tx_total += k_tmp->len;
			if (tx_total > bufsiz) {
				status = -EMSGSIZE;
				goto done;
			}
			k_tmp->tx_buf = tx_buf;
			strncpy(tx_buf,(char *)u_tmp->tx_buf,u_tmp->len);
			tx_buf += k_tmp->len;
		}

		k_tmp->cs_change = !!u_tmp->cs_change;
		k_tmp->tx_nbits = u_tmp->tx_nbits;
		k_tmp->rx_nbits = u_tmp->rx_nbits;
		k_tmp->bits_per_word = u_tmp->bits_per_word;
		k_tmp->delay_usecs = u_tmp->delay_usecs;
		k_tmp->speed_hz = u_tmp->speed_hz;
		if (!k_tmp->speed_hz)
			k_tmp->speed_hz = spidev->speed_hz;
		
		spi_message_add_tail(k_tmp, &msg);
	}

	status = spidev_sync(spidev, &msg);
	if (status < 0)
	{
		printk("%s LINE = %d\n",__func__,__LINE__);
		goto done;
	}

	status = total;

done:
	kfree(k_xfers);
	return status;
}


int transfer_demo(struct w25q *flash)
{
	int ret = 0;
	u8 recevie_buf[4] = {0x0};
	u8 send_buf[4] = {0x0};
    struct spi_ioc_transfer tr = {
             .tx_buf = (unsigned long)send_buf,   //定义发送缓冲区指针
             .rx_buf = (unsigned long)recevie_buf,   //定义接收缓冲区指针
             .len = 4,         
             .delay_usecs = 0,
             .speed_hz = 1000000,
             .bits_per_word = 8,
     };	


	W25Q = flash;
	W25Q->dev = &flash->spi->dev;
	W25Q->tx_buffer = kmalloc(bufsiz, GFP_KERNEL);
	if (!W25Q->tx_buffer) {
		dev_err(&W25Q->spi->dev, "alloc tx_buffer failed\n");
		return -ENOMEM;
	}

	W25Q->rx_buffer = kmalloc(bufsiz, GFP_KERNEL);
	if (!W25Q->rx_buffer) {
		dev_err(&W25Q->spi->dev, "alloc rx_buffer failed\n");
		return -ENOMEM;
	}

	
	send_buf[0] = 0x9f;
	ret = spidev_message(W25Q,&tr,1);
	printk("transfer_demo result : %02x:%02x:%02x\n",
		recevie_buf[1],recevie_buf[2],recevie_buf[3]);
	return ret;
}

#endif

/*
 * board specific setup should have ensured the SPI clock used here
 * matches what the READ command supports, at least until this driver
 * understands FAST_READ (for clocks over 25 MHz).
 */
static int w25q_probe(struct spi_device *spi)
{
	struct flash_platform_data	*data;
	struct w25q *flash;
	struct spi_nor *nor;
	struct spi_nor_hwcaps hwcaps = {
		.mask = SNOR_HWCAPS_READ |
			SNOR_HWCAPS_READ_FAST |
			SNOR_HWCAPS_PP,
	};
	char *flash_name;
	int ret;

	data = dev_get_platdata(&spi->dev);

	flash = devm_kzalloc(&spi->dev, sizeof(*flash), GFP_KERNEL);
	if (!flash)
		return -ENOMEM;



	nor = &flash->spi_nor;

	/* install the hooks */
	nor->read = w25q32jv_read;
	nor->write = w25q32jv_write;
	nor->write_reg = w25q32jv_write_reg;
	nor->read_reg = w25q32jv_read_reg;

	nor->dev = &spi->dev;
	spi_nor_set_flash_node(nor, spi->dev.of_node);
	nor->priv = flash;
	spi_set_drvdata(spi, flash);
	flash->spi = spi;

	if (spi->mode & SPI_RX_QUAD) {
		hwcaps.mask |= SNOR_HWCAPS_READ_1_1_4;

		if (spi->mode & SPI_TX_QUAD)
			hwcaps.mask |= (SNOR_HWCAPS_READ_1_4_4 |
					SNOR_HWCAPS_PP_1_1_4 |
					SNOR_HWCAPS_PP_1_4_4);
	} else if (spi->mode & SPI_RX_DUAL) {
		hwcaps.mask |= SNOR_HWCAPS_READ_1_1_2;

		if (spi->mode & SPI_TX_DUAL)
			hwcaps.mask |= SNOR_HWCAPS_READ_1_2_2;
	}

	if (data && data->name)
		nor->mtd.name = data->name;

	/* For some (historical?) reason many platforms provide two different
	 * names in flash_platform_data: "name" and "type". Quite often name is
	 * set to "w25q32jv" and then "type" provides a real chip name.
	 * If that's the case, respect "type" and ignore a "name".
	 */
	if (data && data->type)
		flash_name = data->type;
	else if (!strcmp(spi->modalias, "spi-nor"))
		flash_name = NULL; /* auto-detect */
	else
		flash_name = spi->modalias;

#ifdef SPI_TRANSFER_TEST
	ret = transfer_demo(flash);
#endif

	ret = spi_nor_scan(nor, flash_name, &hwcaps);
	if (ret)
		return ret;

	return mtd_device_register(&nor->mtd, data ? data->parts : NULL,
				   data ? data->nr_parts : 0);
}


static int w25q_remove(struct spi_device *spi)
{
	struct w25q	*flash = spi_get_drvdata(spi);

	/* Clean up MTD stuff. */
	return mtd_device_unregister(&flash->spi_nor.mtd);
}

/*
 * Do NOT add to this array without reading the following:
 *
 * Historically, many flash devices are bound to this driver by their name. But
 * since most of these flash are compatible to some extent, and their
 * differences can often be differentiated by the JEDEC read-ID command, we
 * encourage new users to add support to the spi-nor library, and simply bind
 * against a generic string here (e.g., "jedec,spi-nor").
 *
 * Many flash names are kept here in this list (as well as in spi-nor.c) to
 * keep them available as module aliases for existing platforms.
 */
static const struct spi_device_id w25q_ids[] = {
	/*
	 * Allow non-DT platform devices to bind to the "spi-nor" modalias, and
	 * hack around the fact that the SPI core does not provide uevent
	 * matching for .of_match_table
	 */
	{"spi-nor"},

	/*
	 * Entries not used in DTs that should be safe to drop after replacing
	 * them with "spi-nor" in platform data.
	 */
	{"s25sl064a"},	{"w25x16"},	{"w25q10"},	{"w25qx64"},

	/*
	 * Entries that were used in DTs without "jedec,spi-nor" fallback and
	 * should be kept for backward compatibility.
	 */
	{"at25df321a"},	{"at25df641"},	{"at26df081a"},
	{"mx25l4005a"},	{"mx25l1606e"},	{"mx25l6405d"},	{"mx25l12805d"},
	{"mx25l25635e"},{"mx66l51235l"},
	{"n25q064"},	{"n25q128a11"},	{"n25q128a13"},	{"n25q512a"},
	{"s25fl256s1"},	{"s25fl512s"},	{"s25sl12801"},	{"s25fl008k"},
	{"s25fl064k"},
	{"sst25vf040b"},{"sst25vf016b"},{"sst25vf032b"},{"sst25wf040"},
	{"w25q40"},	{"w25q32jv"},	{"w25q16"},	{"w25q32"},
	{"w25q64"},	{"w25q128"},
	{"w25x80"},	{"w25x32"},	{"w25q32"},	{"w25q32dw"},
	{"w25q80bl"},	{"w25q128"},	{"w25q256"},

	/* Flashes that can't be detected using JEDEC */
	{"w25q05-nonjedec"},	{"w25q10-nonjedec"},	{"w25q20-nonjedec"},
	{"w25q40-nonjedec"},	{"w25q32jv-nonjedec"},	{"w25q16-nonjedec"},
	{"w25q32-nonjedec"},	{"w25q64-nonjedec"},	{"w25q128-nonjedec"},

	/* Everspin MRAMs (non-JEDEC) */
	{ "mr25h256" }, /* 256 Kib, 40 MHz */
	{ "mr25h10" },  /*   1 Mib, 40 MHz */
	{ "mr25h40" },  /*   4 Mib, 40 MHz */

	{ },
};
MODULE_DEVICE_TABLE(spi, w25q_ids);

static const struct of_device_id w25q_of_table[] = {
	/*
	 * Generic compatibility for SPI NOR that can be identified by the
	 * JEDEC READ ID opcode (0x9F). Use this, if possible.
	 */
	{ .compatible = "w25q32jv,spi-nor" },
	{}
};
MODULE_DEVICE_TABLE(of, w25q_of_table);

static struct spi_driver w25q32jv_driver = {
	.driver = {
		.name	= "w25q32jv",
		.of_match_table = w25q_of_table,
	},
	.id_table	= w25q_ids,
	.probe	= w25q_probe,
	.remove	= w25q_remove,

	/* REVISIT: many of these chips have deep power-down modes, which
	 * should clearly be entered on suspend() to minimize power use.
	 * And also when they're otherwise idle...
	 */
};

static int __init w25q32jv_init(void)
{
	int status;

	status = spi_register_driver(&w25q32jv_driver);
	if (status < 0) {
		pr_err("w25q32jv_driver init failed\n");
		return status;
	}
	
	pr_err("w25q32jv_driver init success\n");
	return status;
}
module_init(w25q32jv_init);

static void __exit w25q32jv_exit(void)
{
	pr_err("w25q32jv_driver exit\n");
	spi_unregister_driver(&w25q32jv_driver);
}
module_exit(w25q32jv_exit);



MODULE_LICENSE("GPL");
MODULE_AUTHOR("Mike Lavender");
MODULE_DESCRIPTION("MTD SPI driver for ST w25qxx flash chips");
