/*
 * OD2101 I2C转串口驱动
 */
#include <linux/module.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/device.h>
#include <linux/serial_core.h>
#include <linux/serial.h>
#include <linux/freezer.h>
#include <linux/i2c.h>
#include <linux/wakelock.h>
#include <linux/gpio.h>

#define OD2101_RESET_GPIO  S5PV210_GPH2(1)

#define MAX_OD2101_CHIPS 4
#define OD2101_MAJOR	222
#define OD2101_MINOR	209

enum {
	OD2101_UART_9600,
	OD2101_UART_300,
	OD2101_UART_600,
	OD2101_UART_900,
	OD2101_UART_1200,

	OD2101_UART_1800,
	OD2101_UART_2400,
	OD2101_UART_3600,
	OD2101_UART_4800,
	OD2101_UART_7200,

	OD2101_UART_14400,
	OD2101_UART_19200,
	OD2101_UART_28800,
	OD2101_UART_38400,

	OD2101_UART_57600,
	OD2101_UART_115200
};

#define OD2101_UART_WR_REG 0x00
#define OD2101_UART_RD_REG 0x00
#define OD2101_UARTBUF_REG 0x01
#define OD2101_I2CBUF_REG  0x02
#define OD2101_CTRL_REG	   0x03
#define OD2101_I2CBUF_SIZE 64

struct od2101_port {
	struct uart_port port;
	struct i2c_client *i2c;
	int cts;
	int rts;
	int rts_commit;
	int parity;
	int rx_enabled;
	int irq;
	int minor;
	struct workqueue_struct *workqueue;
	struct work_struct twork;
	struct work_struct rwork;
	struct mutex	mutex; 
};

static struct od2101_port *od2101s[MAX_OD2101_CHIPS] = {NULL};

static int od2101_i2c_write_reg(struct i2c_client *i2c, unsigned char reg, unsigned char value)
{
	int ret;

	ret = i2c_smbus_write_byte_data(i2c, reg, value);
	return ret;
}

static int od2101_i2c_read_reg(struct i2c_client *i2c, unsigned char reg, unsigned char *value)
{
	int ret;

	ret = i2c_smbus_read_byte_data(i2c, reg);
	if (ret < 0) return ret;

	ret &= 0xff;
	*value = ret;

	return 0;
}

static int od2101_i2c_block_read(struct i2c_client *i2c, unsigned char reg, int count, unsigned char *buf)
{
	int ret;

	ret = i2c_smbus_read_i2c_block_data(i2c, reg, count, buf);
	if (ret < 0) {
		return ret;
	}
	return 0;
}

static int od2101_i2c_block_write(struct i2c_client *i2c, unsigned char reg, int count, unsigned char *buf)
{
	int ret;

	ret = i2c_smbus_write_i2c_block_data(i2c, reg, count, buf);
	if (ret < 0) {
		return ret;
	}
	return 0;
}

void od2101_uart_test(void)
{
	struct i2c_client *client = od2101s[0]->i2c;
	int ret;
	int i;
	unsigned char rbuf[64];
	unsigned char value;
	ret = od2101_i2c_read_reg(client, 0x03,  &value);
	if (ret < 0) {
		printk(KERN_ERR "read 0x03 ctrl register error\n");
		return;
	}
	value = (value & 0xf0) | OD2101_UART_115200;
	ret = od2101_i2c_write_reg(client, 0x03, value);
	if (ret < 0) {
		printk(KERN_ERR "write 0x03 ctrl register error\n");
		return;
	}

	rbuf[0] = 'g';
	rbuf[1] = 'o';
	rbuf[2] = 'o';
	rbuf[3] = 'd';
	ret = od2101_i2c_block_write(client, 0x00, 4, rbuf);
#if 1
	while(1) {
		//读取接受缓冲区个数
		ret = od2101_i2c_read_reg(client, 0x01, &value);	
		if (ret < 0) {
			printk(KERN_ERR "read 0x01 uartbufcount register error\n");
			break;
		}
		if (value) {
			ret = od2101_i2c_block_read(client, 0x00, value, rbuf); 
			if (ret < 0) {
				printk(KERN_ERR "block read %d from 0x00 error\n", value);
				break;
			}
			//debug
			printk(KERN_INFO "od2101_rx:");
			for (i = 0; i < value; i++) {
				printk("0x%02x,", rbuf[i]);
			}
			printk("\n");

			//send back
			od2101_i2c_block_write(client, 0x00, value, rbuf); 

			for (i = 0; i < value; i++) {
				if (rbuf[i] == 'q') break;
				if (rbuf[i] == 'Q') break;
			}
		}
	}
#endif
	return;
}

static unsigned int od2101_tx_empty(struct uart_port *port)
{
	struct od2101_port *od2101 = container_of(port, struct od2101_port, port);
	int ret;
	unsigned char val;
	//fifo中数据发送完成，则返回1,否则0
	printk("%s %s %d\n", __FILE__, __func__);
	mutex_lock(&od2101->mutex);
	
	ret = od2101_i2c_read_reg(od2101->i2c, OD2101_I2CBUF_REG, &val); 
	if (ret < 0) {
		printk(KERN_ERR "od2101 i2c read buf error in tx_empty?\n");
		ret = 0;
	} else {
		printk(KERN_INFO "od2101 tx_empty  i2cbuf avail=%d\n", val);
		if (val == OD2101_I2CBUF_SIZE) ret = 1; 
		else ret = 0;
	}

	mutex_unlock(&od2101->mutex);
	printk("%s %s %d\n", __FILE__, __func__);
	return 0;
}

static unsigned int od2101_get_mctrl(struct uart_port *port)
{
	struct od2101_port *od2101 = container_of(port, struct od2101_port, port);
	return TIOCM_DSR | TIOCM_CAR;
}

static void od2101_set_mctrl(struct uart_port *port, unsigned int mctrl)
{
	struct od2101_port *od2101 = container_of(port, struct od2101_port, port);
	printk("%s %s %d %p\n", __FILE__, __func__, __LINE__, od2101);
	return;	
}

static void od2101_stop_tx(struct uart_port *port)
{
	struct od2101_port *od2101 = container_of(port, struct od2101_port, port);
	return;
}

static void od2101_start_tx(struct uart_port *port)
{
	struct od2101_port *od2101 = container_of(port, struct od2101_port, port);

	queue_work(od2101->workqueue, &od2101->twork);	
	return;
}

static void od2101_stop_rx(struct uart_port *port)
{
	struct od2101_port *od2101 = container_of(port, struct od2101_port, port);

	disable_irq_nosync(od2101->irq);
	return;
}

static void od2101_enable_ms(struct uart_port *port)
{
	//struct od2101_port *od2101 = container_of(port, struct od2101_port, port);
}

static void od2101_break_ctl(struct uart_port *port, int break_state)
{
//	struct od2101_port *od2101 = container_of(port, struct od2101_port, port);
	return;
}

static void od2101_tx_work(struct work_struct *w)
{
	struct od2101_port *od2101 = container_of(w, struct od2101_port, twork);
	struct uart_port *port = &od2101->port;
	struct i2c_client *client = od2101->i2c;
	struct circ_buf *xmit = &port->state->xmit;
	int ret, i;
	int max_count = 64;
	unsigned char buf[64];
	unsigned char avail, final;
	
	if (uart_circ_empty(xmit) || uart_tx_stopped(port)) {
		return;
	}

	//有数据需要发送，并且本次还可以发送
	mutex_lock(&od2101->mutex);
	while(!uart_circ_empty(xmit) && max_count > 0) {
		ret = od2101_i2c_read_reg(client, OD2101_I2CBUF_REG, &avail);	
		if (ret < 0) {
			printk(KERN_ERR "od2101 i2c read i2cbuf error\n");
			goto out;
		}
		if (avail > max_count) {	//最多发送剩余的64个字节
			avail = max_count;
		}
		if (avail > 32) avail = 32;		//一次最多32个字节???
		if (avail > 0) {
			final = 0;				//根据最终的字符长度，进行调整
			for (i = 0; i < avail; i++) {
				buf[i] = xmit->buf[xmit->tail];
				xmit->tail = (xmit->tail + 1) & (UART_XMIT_SIZE - 1);
				port->icount.tx++;
				final++;
				if (uart_circ_empty(xmit)) break;
			}
			printk("od send %d\n", final);
			od2101_i2c_block_write(client, OD2101_UART_WR_REG, final, buf); 
			max_count -= final;
		}
	}
	if (uart_circ_chars_pending(xmit) < WAKEUP_CHARS) 
		uart_write_wakeup(port);

	if (!uart_circ_empty(xmit))  {//还有数据需要发送
		queue_work(od2101->workqueue, &od2101->twork);
	}
out:
	mutex_unlock(&od2101->mutex);
}

static void od2101_rx_work(struct work_struct *w)
{
	struct od2101_port *od2101 = container_of(w, struct od2101_port, rwork);
	struct uart_port *port = &od2101->port;
	struct i2c_client *client = od2101->i2c;
	struct tty_struct *tty = port->state->port.tty;
	int max_count = 64;							//一次最多接收count个数据
	int ret, i, rxchars = 0;
	struct circ_buf *xmit = &od2101->port.state->xmit;
	unsigned char buf[64];
	unsigned char fifocount;
	
	mutex_lock(&od2101->mutex);
	do {
		ret = od2101_i2c_read_reg(client, OD2101_UARTBUF_REG, &fifocount); 
		if (ret < 0) {
			printk(KERN_ERR "read uartbuf count reg register error\n");
			goto exit;
		}
		printk("rxfifo=%d:\n", fifocount);
		if (fifocount > 32) fifocount = 32;
		if (fifocount) {	//有接收到数据
			ret = od2101_i2c_block_read(client, OD2101_UART_RD_REG, fifocount, buf);
			if (ret < 0) {
				printk(KERN_ERR "od2101 block read %d from rd_reg error\n", fifocount);
				goto exit;
			}
			for (i = 0; i < fifocount; i++) {
				if (uart_handle_sysrq_char(port, buf[i])) continue;
				uart_insert_char(port, 0, 0, buf[i], TTY_NORMAL); 
				port->icount.rx++;
			}
			max_count -= fifocount;
		} else {
			break;
		}
	}while(max_count > 0);
	tty_flip_buffer_push(tty);
exit:
	mutex_unlock(&od2101->mutex);
	enable_irq(od2101->irq);
}

static irqreturn_t od2101_rx_irq(int irqno, void *dev_id)
{
	struct od2101_port *od2101 = dev_id;

	//先关闭接收中断
	disable_irq_nosync(od2101->irq);
	queue_work(od2101->workqueue, &od2101->rwork);
	return IRQ_HANDLED;
}

static int od2101_startup(struct uart_port *port)
{
	struct od2101_port *od2101 = container_of(port, struct od2101_port, port);
	char name[12];
	//创建工作队列
	od2101->rx_enabled = 1;
	
	sprintf(name, "od2101-%d", od2101->minor);
	od2101->workqueue = create_freezable_workqueue(name);
	if (!od2101->workqueue) {
		printk(KERN_ERR "cannot create tx and rx workqueue\n");
		return -EBUSY;
	}
	INIT_WORK(&od2101->twork, od2101_tx_work);
	INIT_WORK(&od2101->rwork, od2101_rx_work);
	
	if (request_irq(od2101->irq, od2101_rx_irq,
				IRQF_TRIGGER_LOW, "od2101", od2101) < 0) {
		printk(KERN_ERR "canniot allocate irq %d\n", od2101->irq);
		od2101->irq = 0;
		goto irq_req_error;
	}
	return 0;
irq_req_error:
	destroy_workqueue(od2101->workqueue);
	return -EBUSY;
}

static void od2101_shutdown(struct uart_port *port)
{
	struct od2101_port *od2101 = container_of(port, struct od2101_port, port);
	
	if (od2101->workqueue) {
		flush_workqueue(od2101->workqueue);
		destroy_workqueue(od2101->workqueue);
		od2101->workqueue = NULL;
	}
	if (od2101->irq) {
		free_irq(od2101->irq, od2101);
	}
	return;
}

static void od2101_set_termios(struct uart_port *port, struct ktermios *termios, struct ktermios *old)
{
	struct od2101_port *od2101 = container_of(port, struct od2101_port, port);
	unsigned int baud;
	int baudreg;
	int ret;
	unsigned char reg;

	termios->c_cflag &= ~(HUPCL | CMSPAR);
	termios->c_cflag |= CLOCAL;

	baud = tty_termios_baud_rate(termios); 
	switch(baud) {
		case 9600: 
			baudreg = OD2101_UART_9600;
			break;
		case 4800:
			baudreg = OD2101_UART_9600;
			break;
		
		case 19200:
			baudreg = OD2101_UART_19200;
			break;

		case 38400:
			baudreg = OD2101_UART_38400;
			break;

		case 57600:
			baudreg = OD2101_UART_57600;
			break;

		case 115200:
		default:
			baudreg = OD2101_UART_115200;
			break;
	}
	printk("%s %s %d\n", __FILE__, __func__, __LINE__);
	mutex_lock(&od2101->mutex);
	ret = od2101_i2c_read_reg(od2101->i2c, OD2101_CTRL_REG, &reg); 	
	if (ret < 0) {
		printk(KERN_ERR "od2101 set termious error, read ctrl reg\n");
	}
	reg &= ~(0x0f);
	reg |= baudreg;
	ret = od2101_i2c_write_reg(od2101->i2c, OD2101_CTRL_REG, reg);
	if (ret < 0) {
		printk(KERN_ERR "od2101 set termious error, write ctrl reg\n");
	}
	mutex_unlock(&od2101->mutex);
	printk("%s %s %d\n", __FILE__, __func__, __LINE__);
	tty_termios_encode_baud_rate(termios, baud, baud);

	od2101->port.state->port.tty->low_latency = 1;
	uart_update_timeout(port, termios->c_cflag, baud);

	return;
}

static const char *od2101_type(struct uart_port *port)
{
	struct od2101_port *od2101 = container_of(port, struct od2101_port, port);
	return "OD2101";
}

static void od2101_release_port(struct uart_port *port)
{
	struct od2101_port *od2101 = container_of(port, struct od2101_port, port);
	return;
}

static void od2101_config_port(struct uart_port *port, int flags)
{
	struct od2101_port *od2101 = container_of(port, struct od2101_port, port);

	if (flags & UART_CONFIG_TYPE) {
		od2101->port.type = PORT_MAX3100;
	}
}

static int od2101_verify_port(struct uart_port *port, struct serial_struct *ser)
{
	struct od2101_port *od2101 = container_of(port, struct od2101_port, port);
	int ret = -EINVAL;

	if (ser->type == PORT_UNKNOWN || ser->type == PORT_MAX3100) 
		return 0;
	return ret;
}

static int od2101_request_port(struct uart_port *port)
{
	struct od2101_port *od2101 = container_of(port, struct od2101_port, port);
	printk("%s %s %d %p\n", __FILE__, __func__, __LINE__, od2101);
	return 0;
}

static struct uart_ops od2101_ops = {
	.tx_empty	= od2101_tx_empty,
	.set_mctrl	= od2101_set_mctrl,
	.get_mctrl	= od2101_get_mctrl,
	.stop_tx        = od2101_stop_tx,
	.start_tx	= od2101_start_tx,
	.stop_rx	= od2101_stop_rx,
	.enable_ms      = od2101_enable_ms,
	.break_ctl      = od2101_break_ctl,
	.startup	= od2101_startup,
	.shutdown	= od2101_shutdown,
	.set_termios	= od2101_set_termios,
	.type		= od2101_type,
	.release_port   = od2101_release_port,
	.request_port   = od2101_request_port,
	.config_port	= od2101_config_port,
	.verify_port	= od2101_verify_port,
};

static struct uart_driver od2101_uart_driver = {
	.owner = THIS_MODULE,
	.driver_name = "ttyOD",
	.dev_name = "ttyOD",
	.major = OD2101_MAJOR,
	.nr = MAX_OD2101_CHIPS,
};

static int od2101_uart_driver_registered =  0;
static int od2101_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int i;
	int ret;
	char queuename[12];

	if (!od2101_uart_driver_registered) {
		od2101_uart_driver_registered = 1;
		ret = uart_register_driver(&od2101_uart_driver);
		if (ret) {
			printk(KERN_ERR "Couldn't register od2101 uart driver\n");
			return ret;
		}
	}
	for (i = 0; i < MAX_OD2101_CHIPS; i++) 
		if (!od2101s[i]) break;

	if (i == MAX_OD2101_CHIPS) {
		printk(KERN_ERR "too may od2101 chips\n");
		return -ENOMEM;
	}

	od2101s[i] = kzalloc(sizeof(struct od2101_port), GFP_KERNEL);
	if (!od2101s[i]) {
		printk(KERN_ERR "kmalloc for od2101 structure %d failed!\n", i);
		return -ENOMEM;
	}
	
	od2101s[i]->i2c = client; 
	od2101s[i]->rx_enabled = 0;
	od2101s[i]->minor = i;

	od2101s[i]->irq = IRQ_EINT(17); 
	if (gpio_request(S5PV210_GPH2(1), "i2c-uart")) {
		printk(KERN_ERR "-------request gpio h2(1) error\n");
		return -ENOMEM;
	}
	gpio_direction_input(S5PV210_GPH2(1));
	s3c_gpio_setpull(S5PV210_GPH2(1), S3C_GPIO_PULL_UP);	
#if 1
	printk(KERN_INFO "%s: adding port %d\n", __func__, i);
	od2101s[i]->port.irq = od2101s[i]->irq;
	od2101s[i]->port.uartclk = 3686400;
	od2101s[i]->port.fifosize = 64;
	od2101s[i]->port.ops = &od2101_ops;
	od2101s[i]->port.flags = UPF_SKIP_TEST | UPF_BOOT_AUTOCONF;
	od2101s[i]->port.line = i;
	od2101s[i]->port.type = PORT_MAX3100;
	od2101s[i]->port.dev = &client->dev;

	mutex_init(&od2101s[i]->mutex);

	ret = uart_add_one_port(&od2101_uart_driver, &(od2101s[i]->port));
	if (ret < 0) {
		printk(KERN_ERR "uart add one port for line %d with error %d\n", 
				i, ret);
		kfree(od2101s[i]);
		od2101s[i] = NULL;
	}
#endif
	printk("--------------------od2101 i2c-->uart probed\n");
	return 0;
}

static int od2101_remove(struct i2c_client *client)
{
	struct od2101_port *odport = i2c_get_clientdata(client);
	gpio_free(S5PV210_GPH2(1));
	return 0;
}

static int od2101_suspend(struct i2c_client *client,
		pm_message_t state)
{
	struct od2101_port *odport = i2c_get_clientdata(client);
	return 0;
}

static int od2101_resume(struct i2c_client *client)
{
	struct od2101_port *odport = i2c_get_clientdata(client);

	return 0;
}

/* Every chip have a unique id */
static const struct i2c_device_id od2101_id[] = {
	{ "od2101", 0 },
	{ }
};

/* Every chip have a unique id and we need to register this ID using MODULE_DEVICE_TABLE*/
MODULE_DEVICE_TABLE(i2c, od2101_id);

static struct i2c_driver od2101_i2c_driver = {
	.driver	= {
		.name	= "od2101",
	},
	.probe		= od2101_probe,
	.remove		= __devexit_p(od2101_remove),
	.suspend	= od2101_suspend,    
	.resume		= od2101_resume,  
	.id_table	= od2101_id,
};

static int __init od2101_init(void)
{
	return i2c_add_driver(&od2101_i2c_driver);
}
module_init(od2101_init);

static void __exit od2101_exit(void)
{
	i2c_del_driver(&od2101_i2c_driver);
}
module_exit(od2101_exit);

MODULE_AUTHOR("GZ Evision");
MODULE_DESCRIPTION("I2C To UART driver");
MODULE_LICENSE("GPL");
