/* 
 * drivers/input/touchscreen/ft5x0x_ts.c
 *
 * FocalTech ft5x0x TouchScreen driver. 
 *
 * Copyright (c) 2011  Laerdal Medical.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 *
 *	note: only support mulititouch	Wenfs 2010-10-01
 */

#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/ft5x0x_ts.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/slab.h>

#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif

#define FT_CMD_READ_TOUCH 0xF9
#define FT_CMD_REG 0xFC
#define FT_REG_ID 0x3D
#define FT_REG_VERSION  0x3B
#define FT_REG_STATE  0x3C
#define FT_REG_PMODE  0x3A
#define FT_REG_CTL 0x6

#define DEBUG
#undef CONFIG_FT5X0X_MULTITOUCH
#undef REPORT_PRESSURE

struct ts_event {
	u16 x1;
	u16 y1;
	u16 x2;
	u16 y2;
	u16 x3;
	u16 y3;
	u16 x4;
	u16 y4;
	u16 x5;
	u16 y5;
	u16 pressure;
	u8 touch_point;
};

struct ft5x0x_ts_data {
	struct input_dev *input_dev;
	struct ts_event event;
	struct work_struct pen_event_work;
	//struct workqueue_struct *ts_workqueue;
	struct i2c_client *client;
	char phys[128];
#ifdef CONFIG_HAS_EARLYSUSPEND
struct early_suspend early_suspend;
#endif
};

static u8 compute_crc(const u8* msg, int len) {
	u8 res=0;
	int n;
	for (n=0; n < len; n++)
		res = res ^msg[n];

	return res;
}

static int ft5x0x_i2c_txdata(struct i2c_client *client, char *txdata,
		int length) {
	int ret;
	struct i2c_msg msg[] = {
		{
			.addr = client->addr,
			.flags = 0,
			.len = length,
			.buf = txdata,
		},
	};

	ret = i2c_transfer(client->adapter, msg, 1);
	if (ret < 0)
		pr_err("%s i2c write error: %d\n", __func__, ret);

	return ret;
}

static int ft5x0x_set_reg(struct i2c_client* client, u8 addr, u8 para) {
	u8 buf[4];
	int ret = -1;

	buf[0] = FT_CMD_REG;
	buf[1] = addr;
	buf[2] = para;
	buf[3] = buf[0] ^ buf[1] ^ buf[2];

	ret = ft5x0x_i2c_txdata(client, buf, 4);
	if (ret < 0) {
		pr_err("write reg failed! %#x ret: %d", buf[0], ret);
		return -1;
	}

	return 0;
}

static int ft5x0x_get_reg(struct i2c_client* client, u8 addr) {
	u8 rxBuf[4];
	int ret;
	struct i2c_msg msg[] = {
			{
				.addr = client->addr,
				.flags = I2C_M_RD,
				.len = 2,
				.buf = rxBuf,
			},
		};

	ret = i2c_smbus_write_byte_data(client, FT_CMD_REG, addr | 0x40);
	if ( ret < 0 )
	{
		dev_err(&client->dev, "Read reg cmd failed");
		return -ENODEV;
	}
	ret = i2c_transfer(client->adapter, msg, 1);
	return rxBuf[0];
}


static void ft5x0x_ts_release(struct ft5x0x_ts_data *ts) {

#ifdef CONFIG_FT5X0X_MULTITOUCH	
	input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, 0);
#else
#ifdef REPORT_PRESSURE
	input_report_abs(ts->input_dev, ABS_PRESSURE, 0);
#endif
	input_report_key(ts->input_dev, BTN_TOUCH, 0);
#endif
	input_sync(ts->input_dev);
}

static int ft5x0x_read_data(struct ft5x0x_ts_data *data) {

	struct ts_event *event = &data->event;
	u8 buf[32];
	u8 crc;
	int ret;
	struct i2c_msg msg[] = {
		{
			.addr = data->client->addr,
			.flags = I2C_M_RD,
			.len = 26,
			.buf = buf,
		},
	};

	ret = i2c_smbus_write_byte(data->client, FT_CMD_READ_TOUCH);
	if (ret < 0) {
		dev_err(&data->client->dev, "Failed to send read touch command\n");
		return ret;
	}

	ret = i2c_transfer(data->client->adapter, msg, 1);
	if (ret < 0) {
		dev_err(&data->client->dev, "Failed to read touch data\n");
		return ret;
	}
	crc = compute_crc(buf, 25);
	if (buf[25] != crc) {
		dev_err(&data->client->dev, "CRC error in touch data\n");
		return -EINVAL;
	}
	//dev_info(&data->client->dev, "packet len is %d\n", buf[2]);
	memset(event, 0, sizeof(struct ts_event));
	event->touch_point = buf[3] & 0x07;// 000 0111
	//dev_info(&data->client->dev, "Touch pt = %d\n", event->touch_point);
	if (event->touch_point == 0) {
		ft5x0x_ts_release(data);
		return 1;
	}

	switch (event->touch_point) {
	case 5:
		event->x5 = (s16) (buf[21] & 0x0F) << 8 | (s16) buf[22];
		event->y5 = (s16) (buf[23] & 0x0F) << 8 | (s16) buf[24];
	case 4:
		event->x4 = (s16) (buf[17] & 0x0F) << 8 | (s16) buf[18];
		event->y4 = (s16) (buf[19] & 0x0F) << 8 | (s16) buf[20];
	case 3:
		event->x3 = (s16) (buf[13] & 0x0F) << 8 | (s16) buf[14];
		event->y3 = (s16) (buf[15] & 0x0F) << 8 | (s16) buf[16];
	case 2:
		event->x2 = (s16) (buf[9] & 0x0F) << 8 | (s16) buf[10];
		event->y2 = (s16) (buf[11] & 0x0F) << 8 | (s16) buf[12];
	case 1:
		event->x1 = (s16) (buf[5] & 0x0F) << 8 | (s16) buf[6];
		event->y1 = (s16) (buf[7] & 0x0F) << 8 | (s16) buf[8];
		break;
	default:
		return -1;
	}

	event->pressure = 200;
	dev_dbg(&data->client->dev, "%s: 1:%d %d 2:%d %d \n", __func__, event->x1,
			event->y1, event->x2, event->y2);
	return 0;
}

static void ft5x0x_report_value(struct ft5x0x_ts_data *data) {

	struct ts_event *event = &data->event;

#ifdef CONFIG_FT5X0X_MULTITOUCH
	switch (event->touch_point) {
	case 5:
		input_report_abs(data->input_dev, ABS_MT_TOUCH_MAJOR, event->pressure);
		input_report_abs(data->input_dev, ABS_MT_POSITION_X, event->x5);
		input_report_abs(data->input_dev, ABS_MT_POSITION_Y, event->y5);
		input_report_abs(data->input_dev, ABS_MT_WIDTH_MAJOR, 1);
		input_mt_sync(data->input_dev);
		//			printk("===x2 = %d,y2 = %d ====\n",event->x2,event->y2);
	case 4:
		input_report_abs(data->input_dev, ABS_MT_TOUCH_MAJOR, event->pressure);
		input_report_abs(data->input_dev, ABS_MT_POSITION_X, event->x4);
		input_report_abs(data->input_dev, ABS_MT_POSITION_Y, event->y4);
		input_report_abs(data->input_dev, ABS_MT_WIDTH_MAJOR, 1);
		input_mt_sync(data->input_dev);
		//			printk("===x2 = %d,y2 = %d ====\n",event->x2,event->y2);
	case 3:
		input_report_abs(data->input_dev, ABS_MT_TOUCH_MAJOR, event->pressure);
		input_report_abs(data->input_dev, ABS_MT_POSITION_X, event->x3);
		input_report_abs(data->input_dev, ABS_MT_POSITION_Y, event->y3);
		input_report_abs(data->input_dev, ABS_MT_WIDTH_MAJOR, 1);
		input_mt_sync(data->input_dev);
		//			printk("===x2 = %d,y2 = %d ====\n",event->x2,event->y2);
	case 2:
		input_report_abs(data->input_dev, ABS_MT_TOUCH_MAJOR, event->pressure);
		input_report_abs(data->input_dev, ABS_MT_POSITION_X, event->x2);
		input_report_abs(data->input_dev, ABS_MT_POSITION_Y, event->y2);
		input_report_abs(data->input_dev, ABS_MT_WIDTH_MAJOR, 1);
		input_mt_sync(data->input_dev);
		//			printk("===x2 = %d,y2 = %d ====\n",event->x2,event->y2);
	case 1:
		input_report_abs(data->input_dev, ABS_MT_TOUCH_MAJOR, event->pressure);
		input_report_abs(data->input_dev, ABS_MT_POSITION_X, event->x1);
		input_report_abs(data->input_dev, ABS_MT_POSITION_Y, event->y1);
		input_report_abs(data->input_dev, ABS_MT_WIDTH_MAJOR, 1);
		input_mt_sync(data->input_dev);
		//			printk("===x1 = %d,y1 = %d ====\n",event->x1,event->y1);
	default:
		//			printk("==touch_point default =\n");
		break;
	}
#else	/* CONFIG_FT5X0X_MULTITOUCH*/
	if (event->touch_point == 1) {
		input_report_abs(data->input_dev, ABS_X, event->x1);
		input_report_abs(data->input_dev, ABS_Y, event->y1);
#ifdef REPORT_PRESSURE
		input_report_abs(data->input_dev, ABS_PRESSURE, event->pressure);
#endif
	}
	input_report_key(data->input_dev, BTN_TOUCH, 1);
#endif	/* CONFIG_FT5X0X_MULTITOUCH*/
	input_sync(data->input_dev);

	dev_dbg(&data->client->dev, "%s: 1:%d %d 2:%d %d \n", __func__, event->x1,
			event->y1, event->x2, event->y2);
} /*end ft5x0x_report_value*/

static void ft5x0x_ts_pen_irq_work(struct work_struct *work) {
	int ret = -1;
	struct ft5x0x_ts_data* ts = container_of(work, struct ft5x0x_ts_data, pen_event_work);
	ret = ft5x0x_read_data(ts);
	if (ret == 0) {
		ft5x0x_report_value(ts);
	}
}

static irqreturn_t ft5x0x_ts_interrupt(int irq, void *dev_id) {
	struct ft5x0x_ts_data *ft5x0x_ts = dev_id;

	schedule_work(&ft5x0x_ts->pen_event_work);
	return IRQ_HANDLED;
}

#ifdef CONFIG_HAS_EARLYSUSPEND
static void ft5x0x_ts_suspend(struct early_suspend *handler)
{
	//	struct ft5x0x_ts_data *ts;
	//	ts =  container_of(handler, struct ft5x0x_ts_data, early_suspend);

	printk("==ft5x0x_ts_suspend=\n");
	//	disable_irq(this_client->irq);
	//	disable_irq(IRQ_EINT(6));
	//	cancel_work_sync(&ts->pen_event_work);
	//	flush_workqueue(ts->ts_workqueue);
	// ==set mode ==, 
	//    	ft5x0x_set_reg(FT5X0X_REG_PMODE, PMODE_HIBERNATE);
}

static void ft5x0x_ts_resume(struct early_suspend *handler)
{
	printk("==ft5x0x_ts_resume=\n");
	// wake the mode
	//	__gpio_as_output(GPIO_FT5X0X_WAKE);
	//	__gpio_clear_pin(GPIO_FT5X0X_WAKE);		//set wake = 0,base on system
	//	 msleep(100);
	//	__gpio_set_pin(GPIO_FT5X0X_WAKE);			//set wake = 1,base on system
	//	msleep(100);
	//	enable_irq(this_client->irq);
	//	enable_irq(IRQ_EINT(6));
}
#endif  //CONFIG_HAS_EARLYSUSPEND
static int ft5x0x_ts_probe(struct i2c_client *client,
		const struct i2c_device_id *id) {
	struct ft5x0x_ts_data *ft5x0x_ts;
	struct input_dev *input_dev;
	int err = 0;
	int vId, fw, state, pmode;
	dev_info(&client->dev, "ft5x0x_ts probe start\n");

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		err = -ENODEV;
		dev_err(&client->dev, "ft5x0x_ts i2c_check_functionality error\n");
		goto exit_check_functionality_failed;
	}

	ft5x0x_ts = kzalloc(sizeof(struct ft5x0x_ts_data), GFP_KERNEL);
	if (!ft5x0x_ts) {
		err = -ENOMEM;
		dev_err(&client->dev, "ft5x0x_ts alloc error\n");
		goto exit_alloc_data_failed;
	}
	ft5x0x_ts->client = client;
	i2c_set_clientdata(client, ft5x0x_ts);

	vId = ft5x0x_get_reg(client, FT_REG_ID);
	if ( vId < 0 ) {
		kfree(ft5x0x_ts);
		return -ENODEV;
	}

	fw = ft5x0x_get_reg(client, FT_REG_VERSION);
	pmode = ft5x0x_get_reg(client, FT_REG_PMODE);
	ft5x0x_set_reg(client, FT_REG_CTL, 0);	// Set active
	state = ft5x0x_get_reg(client, FT_REG_STATE);
	dev_info(&client->dev, "VID %d, FW %d, STATE %d, PMODE %d\n",
			vId, fw, state, pmode);

	INIT_WORK(&ft5x0x_ts->pen_event_work, ft5x0x_ts_pen_irq_work);
#if 0
	ft5x0x_ts->ts_workqueue = create_singlethread_workqueue(dev_name(&client->dev));
	if (!ft5x0x_ts->ts_workqueue) {
		err = -ESRCH;
		printk(KERN_ERR "ft5x0x_ts create_singlethread_workqueue error\n");
		goto exit_create_singlethread;
	}
#endif
	input_dev = input_allocate_device();
	if (!input_dev) {
		dev_err(&client->dev, "ft5x0x_ts alloc input_device error\n");
		return -ENOMEM;
	}
	input_dev->name = FT5X0X_NAME;
	input_dev->phys = ft5x0x_ts->phys;
	input_dev->dev.parent = &client->dev;
	ft5x0x_ts->input_dev = input_dev;

#ifdef CONFIG_FT5X0X_MULTITOUCH
	set_bit(ABS_MT_TOUCH_MAJOR, input_dev->absbit);
	set_bit(ABS_MT_POSITION_X, input_dev->absbit);
	set_bit(ABS_MT_POSITION_Y, input_dev->absbit);
	set_bit(ABS_MT_WIDTH_MAJOR, input_dev->absbit);

	input_set_abs_params(input_dev, ABS_MT_POSITION_X, 0, SCREEN_MAX_X, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_POSITION_Y, 0, SCREEN_MAX_Y, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_TOUCH_MAJOR, 0, PRESS_MAX, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_WIDTH_MAJOR, 0, 200, 0, 0);
#else
	set_bit(ABS_X, input_dev->absbit);
	set_bit(ABS_Y, input_dev->absbit);

	set_bit(BTN_TOUCH, input_dev->keybit);

	input_set_abs_params(input_dev, ABS_X, 0, SCREEN_MAX_X, 0, 0);
	input_set_abs_params(input_dev, ABS_Y, 0, SCREEN_MAX_Y, 0, 0);
#ifdef REPORT_PRESSURE
	set_bit(ABS_PRESSURE, input_dev->absbit);
	input_set_abs_params(input_dev,	ABS_PRESSURE, 0, PRESS_MAX, 0 , 0);
#endif
#endif
	set_bit(EV_ABS, input_dev->evbit);
	set_bit(EV_KEY, input_dev->evbit);

	err = input_register_device(input_dev);
	if (err) {
		printk(KERN_ERR "ft5x0x_ts input_device_register error\n");
		goto exit_input_register_device_failed;
	}

#ifdef CONFIG_HAS_EARLYSUSPEND
	printk("==register_early_suspend =\n");
	ft5x0x_ts->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
	ft5x0x_ts->early_suspend.suspend = ft5x0x_ts_suspend;
	ft5x0x_ts->early_suspend.resume = ft5x0x_ts_resume;
	register_early_suspend(&ft5x0x_ts->early_suspend);
#endif

	err = request_irq(client->irq, ft5x0x_ts_interrupt, IRQF_TRIGGER_FALLING,
			"ft5x0x_ts", ft5x0x_ts);
	if (err < 0) {
		printk(KERN_ERR "ft5x0x_ts request_irq error\n");
		goto exit_irq_request_failed;
	}

	printk(KERN_ERR "ft5x0x_ts probe sucessed\n");
	return 0;

	exit_irq_request_failed:
#ifdef CONFIG_HAS_EARLYSUSPEND
	unregister_early_suspend(&ft5x0x_ts->early_suspend);
#endif
	input_unregister_device(ft5x0x_ts->input_dev);

exit_input_register_device_failed:
	input_free_device(input_dev);
	cancel_work_sync( &ft5x0x_ts->pen_event_work);
	i2c_set_clientdata(client, NULL);
	kfree(ft5x0x_ts);
	exit_alloc_data_failed:
	exit_check_functionality_failed:
	printk(KERN_ERR "ft5x0x_ts probe failed\n");
	return err;
}

static int __devexit ft5x0x_ts_remove(struct i2c_client *client)
{

	struct ft5x0x_ts_data *ft5x0x_ts = i2c_get_clientdata(client);
	printk(KERN_ERR "ft5x0x_ts remove\n");
	free_irq(client->irq, ft5x0x_ts);
#ifdef CONFIG_HAS_EARLYSUSPEND
	unregister_early_suspend(&ft5x0x_ts->early_suspend);
#endif
	input_unregister_device(ft5x0x_ts->input_dev);
	input_free_device(ft5x0x_ts->input_dev);
	cancel_work_sync(&ft5x0x_ts->pen_event_work);
	//destroy_workqueue(ft5x0x_ts->ts_workqueue);
	i2c_set_clientdata(client, NULL);
	kfree(ft5x0x_ts);
	return 0;
}

static const struct i2c_device_id ft5x0x_ts_id[] = { { FT5X0X_NAME, 0 }, { } };
MODULE_DEVICE_TABLE( i2c, ft5x0x_ts_id);

static struct i2c_driver ft5x0x_ts_driver = { .probe = ft5x0x_ts_probe,
		.remove = __devexit_p(ft5x0x_ts_remove), .id_table = ft5x0x_ts_id,
		.driver = { .name = FT5X0X_NAME, .owner = THIS_MODULE, }, };

static int __init ft5x0x_ts_init(void)
{
	return i2c_add_driver(&ft5x0x_ts_driver);
}

static void __exit ft5x0x_ts_exit(void)
{
	i2c_del_driver(&ft5x0x_ts_driver);
}

module_init(ft5x0x_ts_init);
module_exit(ft5x0x_ts_exit);

MODULE_AUTHOR("<hcl@datarespons.no>");
MODULE_DESCRIPTION("FocalTech ft5x0x TouchScreen driver");
MODULE_LICENSE("GPL");
