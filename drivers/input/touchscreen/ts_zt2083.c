#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/interrupt.h>
#include <linux/errno.h>
#include <linux/device.h>
#include <linux/fs.h>
#include <linux/time.h>
#include <linux/delay.h>
#include <linux/irq.h>
#include <linux/miscdevice.h>
#include <linux/errno.h>
#include <linux/types.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/poll.h>
#include <linux/gpio.h>
#include <ls1x_gpio.h>

#define ZT2083_DEV_NAME  "zt2083"
#define ZT2083_IRQ_PIN      GPIO_32

#define ZT2083_INIT             0xc8
#define ZT2083_INIT2             0xcc
#define ZT2083_Z1_EN             0xa0
#define ZT2083_Z2_EN             0xa0
#define ZT2083_X_EN             0x80
#define ZT2083_Y_EN             0x90
#define ZT2083_PWRDOWN         0x8c

#define ZT2083_ADDR             0x48

#define TS_POLL_DELAY			6
#define TS_POLL_PERIOD			30
#define ZT2083_12BIT 			1  
#define MAX_12BIT               2047

extern gpio_bank_ls1c[3];

struct ts_event {
	unsigned int	x;
	unsigned int	y;
	unsigned int	z1, z2;
};

struct zt2083_data {
	struct input_dev	*input;
	char			phys[32];
	struct delayed_work	work;
	struct i2c_client	*client;
	bool			pendown;
	int			irq;
};
static struct i2c_client *zt2083_adcclient = NULL;

static int _i2c_read_byte(unsigned char addr,unsigned char *data,unsigned long num)
{
    int err = 0;
    struct i2c_client *client = zt2083_adcclient;
    struct i2c_msg msg;
    msg.addr = addr;
    msg.len = num;
    msg.buf = data;
    msg.flags = I2C_M_RD;
    if((err = i2c_transfer(client->adapter,&msg,1))!=1){
        printk(KERN_ERR "msg failed!!!\n");
        return 0;
    }
   return err;
}
static int _i2c_write_byte(unsigned char addr,unsigned char *data,unsigned long num)
{
    int err = 0;
    struct i2c_client *client = zt2083_adcclient;
    struct i2c_msg msg;
    msg.addr = addr;
    msg.len = num;
    msg.buf = data;
    msg.flags = 0;
    if((err = i2c_transfer(client->adapter,&msg,1)) != 1){
        printk(KERN_ERR "msg failed!!!\n");
        return 0;
    }
   return err;
}
static int zt2083_get_xypos(void)
{
#if 1
    int err=0,ret=0;
    unsigned char data[2]={0};
    err =  _i2c_read_byte(ZT2083_ADDR,data,2);
    if(!err)
        printk(KERN_ERR "zt2083 get data failed!!!\n");
     ret=data[0];
    return ((data[1]>>4) | ret<<4);
#endif
}
static int zt2083_set_cmd(unsigned char cmd)
{
#if 1
    int err=0;
    unsigned char data[1]={0};
    data[0]=cmd;
    err = _i2c_write_byte(ZT2083_ADDR,(unsigned char*)data,1); 
    if(!err)
        printk(KERN_ERR "zt2083 write reg failed!!!\n");
    return err;
#endif
}

static void zt2083_get_values(struct ts_event *tc)
{
    zt2083_set_cmd(ZT2083_X_EN);
    //zt2083_set_cmd(0xcc);
    zt2083_set_cmd(0xc0);
    udelay(4);
    tc->x = zt2083_get_xypos();
    zt2083_set_cmd(ZT2083_Y_EN);
    //zt2083_set_cmd(0xdc);
    zt2083_set_cmd(0xd0);
    udelay(4);
    tc->y = zt2083_get_xypos();

    zt2083_set_cmd(ZT2083_Z1_EN);
    //zt2083_set_cmd(0xec);
    zt2083_set_cmd(0xe0);
    udelay(4);
    tc->z1 = zt2083_get_xypos();

    zt2083_set_cmd(ZT2083_Z2_EN);
    //zt2083_set_cmd(0xfc);
    zt2083_set_cmd(0xf0);
    udelay(4);
    tc->z2 = zt2083_get_xypos();
    
	//zt2083_set_cmd(ZT2083_INIT2);
    return ;
}
static unsigned int zt2083_calc_pressure(struct zt2083_data *ts, struct ts_event *tc)
{
	u32 rt = 0;
	if (tc->x == MAX_12BIT)
		tc->x = 0;

	if (likely(tc->x && tc->z1)) {
		rt = tc->z2 - tc->z1;
		rt *= tc->x;
		//rt *= tsc->x_plate_ohms;
		rt *= 600;
		rt /= tc->z1;
		rt = (rt + 2047) >> 12;
	}

	return rt;
}

static void en_inter(unsigned int irq)
{
    (*(volatile unsigned long *)0xbfd01094) |= 1;//clr int flags
	enable_irq(irq);
}

void ts_work_handler(struct work_struct *work)
{
	unsigned int rt=0;
	struct ts_event tc;
	struct zt2083_data *ts = container_of(to_delayed_work(work), struct zt2083_data, work);
	
	zt2083_get_values(&tc);
	rt = zt2083_calc_pressure(ts,&tc);
	if(rt){
		input_report_key(ts->input, BTN_TOUCH, 1);
		input_report_abs(ts->input, ABS_PRESSURE,1); /*X-Y swap*/
		input_report_abs(ts->input, ABS_X, tc.x); /*X-Y swap*/
		input_report_abs(ts->input, ABS_Y, tc.y);
		input_sync(ts->input);
		printk( "%s:x==%d,y==%d,rt====%d\n",__func__,tc.x,tc.y,rt);
		ts->pendown = true;
	}
	else if(ts->pendown){
		input_report_key(ts->input, BTN_TOUCH, 0);
		input_report_abs(ts->input, ABS_PRESSURE,0); /*X-Y swap*/
		input_sync(ts->input);
		ts->pendown = false;
	}
	//printk("ts->pendown = %d\n", ts->pendown);
	if(ts->pendown)
		schedule_delayed_work(&ts->work,msecs_to_jiffies(TS_POLL_PERIOD));
	else{
		en_inter(ts->irq);
//		printk("*****touch screen finish*****\n");
		//ls1b_gpio_unmask_intr(ZT2083_IRQ_PIN);
	}
}

static irqreturn_t ts_interrupt(int irq,void *handle)
{
//	printk("\t\n*****Touch screen start*****\n");
	struct zt2083_data *ts = (struct zt2083_data *)handle;
	unsigned int gpio_in=0, gpio_sr=0, gpio_en=0;

	disable_irq_nosync(irq);
	
	schedule_delayed_work(&ts->work,msecs_to_jiffies(TS_POLL_DELAY));
	
	return IRQ_HANDLED;
}

/*
 * Initialization function
 */

static int zt2083_init_client(struct i2c_client *client)
{
	return 0;
}

static int key_open(struct inode *inode,struct file *file)
{
	return 0;
}
static int key_close(struct inode *inode,struct file *file)
{
	return 0;
}
static int key_read(struct file *filp,char __user *buff,size_t count,loff_t *offp)
{
	return 0;
}
static struct file_operations key_fops = {
	.open       = key_open,
	.release    = key_close,
	.read       = key_read,
};
static struct miscdevice key_dev = {
	.minor  = 189,
	.name   = ZT2083_DEV_NAME,
	.fops   = &key_fops,
};

static int __devinit zt2083_probe(struct i2c_client *client,
                   const struct i2c_device_id *id)
{
    int err, ret;
	unsigned int bank_idx, offset, temp;
	struct i2c_adapter *adapter = to_i2c_adapter(client->dev.parent);
	struct zt2083_data *ts;
	struct input_dev *input;

	printk(KERN_INFO "----zt2083_probe----\n");
	if (!i2c_check_functionality(adapter, I2C_FUNC_I2C)) {
		dev_err(&adapter->dev, "doesn't support full I2C\n");
		err = -EIO;
		goto exit;
		} 

    if (!(ts = kzalloc(sizeof(*ts), GFP_KERNEL))) {
		dev_err(&adapter->dev, "failed to alloc memory\n");
		err = -ENOMEM;
		goto exit;
		}   

	ts->client = client;
	i2c_set_clientdata(client, ts);

    /*send cmd init*/
	if(!zt2083_adcclient){
		printk(KERN_ERR "zt2083_detect!!!!!!\n");
		zt2083_adcclient = client;
	}   

	/* Initialize the zt2083 chip */
	err = zt2083_init_client(client);
	if (err)
		goto exit_kfree;
	
    err = misc_register(&key_dev); 
    if(err < 0 )
    {
   	    printk(ZT2083_DEV_NAME"\tmisc_register error\n");
   	    goto exit;
    }

    input = input_allocate_device();
    if (!input)
	{
   	    err = -ENOMEM;
		goto exit;
    }

	ts->input=input;
	
    input->evbit[0] = BIT(EV_KEY) | BIT(EV_ABS);
    set_bit(BTN_TOUCH, input->keybit);

    input_set_abs_params(input, ABS_X, 0, 800, 5, 0);
    input_set_abs_params(input, ABS_Y, 0, 480, 5, 0);
    input_set_abs_params(input, ABS_PRESSURE, 0, 1, 0, 0);

    input->name = "ZT2083";
    input->id.bustype = BUS_HOST;
    err = input_register_device(input);
    if (err) {
	    printk(KERN_ERR "Unable to register zt2083 ts input device\n");
        goto exit;
    }

    INIT_DELAYED_WORK(&ts->work,ts_work_handler);
	if(gpio_request(ZT2083_IRQ_PIN,"gpio")){
		printk("zt2083 intterrupt gpio request failed!\r\n");
		goto exit_kfree;
	}

	gpio_direction_input(ZT2083_IRQ_PIN);  
	ts->irq = gpio_to_irq(ZT2083_IRQ_PIN);
	
	ret = request_irq(ts->irq,ts_interrupt,IRQF_TRIGGER_FALLING ,ZT2083_DEV_NAME,ts);
    printk(ZT2083_DEV_NAME"\tinitialized\n");
	en_inter(ts->irq);
    return 0;

    exit_kfree:
	    kfree(ts);
    exit:
	    return err;
}

static int __devexit zt2083_remove(struct i2c_client *client)
{
	int err;
	struct zt2083_data *ts = i2c_get_clientdata(client);

    if(client == zt2083_adcclient)
		zt2083_adcclient = NULL;

    err = misc_deregister(&key_dev);
    if(err < 0 )
    {
   	    printk(ZT2083_DEV_NAME"\tmisc_register error\n");
   	    goto exit;
    }

	input_unregister_device(ts->input);
	i2c_set_clientdata(client, NULL);
	kfree(ts);

	return 0;

    exit:
	    return err;
}

static const struct i2c_device_id zt2083_id[] = { 
	    { "zt2083", 0 },
		    { } 
};


static struct i2c_driver zt2083_driver={
        .driver = {
            .name = "zt2083",
            .owner = THIS_MODULE,
        },
        .probe = zt2083_probe,
        .remove = __devexit_p(zt2083_remove),
		.id_table = zt2083_id,
};
static int __init zt2083_init(void)
{
	printk("\t\n*****zt2083_init*****\n");
    return i2c_add_driver(&zt2083_driver);
}

static int __exit zt2083_exit(void)
{
    int ret=0;
    i2c_del_driver(&zt2083_driver);        
    ret = misc_deregister(&key_dev);
    if(ret < 0 )
    {
        printk(ZT2083_DEV_NAME "cml: GPI misc_deregister error\n");
        return ret;
    }


    return 0;
}

late_initcall(zt2083_init);
module_exit(zt2083_exit);

MODULE_AUTHOR("ZHAOKAI <zhaokai@loongson.cn>");
MODULE_DESCRIPTION("LS1B zt2083 touchscreen driver");
MODULE_LICENSE("GPL");
