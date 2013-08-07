/*  Date: 2011/4/8 11:00:00
 *  Revision: 2.5
 */

/*
 * This software program is licensed subject to the GNU General Public License
 * (GPL).Version 2,June 1991, available at http://www.fsf.org/copyleft/gpl.html

 * (C) Copyright 2011 Bosch Sensortec GmbH
 * All Rights Reserved
 */


/* file BMA250.c
   brief This file contains all function implementations for the BMA250 in linux

*/

#include <linux/module.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/mutex.h>
#include <linux/earlysuspend.h>
#include <asm/uaccess.h>
#include <linux/poll.h>
#include <linux/miscdevice.h>
#include <mach/gpio.h>
#include <linux/irq.h>
#include <asm/atomic.h>
#include <mach/socinfo.h>

#define GSENSOR_OFFSET  7

#define MISC_DEV_NAME 			"accdev"
#define SENSOR_NAME 			"bma250"
#define GRAVITY_EARTH                   9806550
#define ABSMIN_2G                       (-GRAVITY_EARTH * 2)
#define ABSMAX_2G                       (GRAVITY_EARTH * 2)
#define SLOPE_THRESHOLD_VALUE 		10
#define SLOPE_DURATION_VALUE 		0
#define INTERRUPT_LATCH_MODE 		13
#define INTERRUPT_ENABLE 		1
#define INTERRUPT_DISABLE 		0
#define MAP_SLOPE_INTERRUPT 		2
#define SLOPE_X_INDEX 			5
#define SLOPE_Y_INDEX 			6
#define SLOPE_Z_INDEX 			7
#define BMA250_MAX_DELAY		200
#define BMA250_CHIP_ID			3
#define BMA222E_CHIP_ID			0xF8
#define BMA250_RANGE_SET		0
#define BMA250_BW_SET			4   //  2-> 31.25hz  16ms : // 4-> 125 hz. 4ms


/*
 *
 *      register definitions
 *
 */

#define BMA250_CHIP_ID_REG                      0x00
#define BMA250_VERSION_REG                      0x01
#define BMA250_X_AXIS_LSB_REG                   0x02
#define BMA250_X_AXIS_MSB_REG                   0x03
#define BMA250_Y_AXIS_LSB_REG                   0x04
#define BMA250_Y_AXIS_MSB_REG                   0x05
#define BMA250_Z_AXIS_LSB_REG                   0x06
#define BMA250_Z_AXIS_MSB_REG                   0x07
#define BMA250_TEMP_RD_REG                      0x08
#define BMA250_STATUS1_REG                      0x09
#define BMA250_STATUS2_REG                      0x0A
#define BMA250_STATUS_TAP_SLOPE_REG             0x0B
#define BMA250_STATUS_ORIENT_HIGH_REG           0x0C
#define BMA250_RANGE_SEL_REG                    0x0F
#define BMA250_BW_SEL_REG                       0x10
#define BMA250_MODE_CTRL_REG                    0x11
#define BMA250_LOW_NOISE_CTRL_REG               0x12
#define BMA250_DATA_CTRL_REG                    0x13
#define BMA250_RESET_REG                        0x14
#define BMA250_INT_ENABLE1_REG                  0x16
#define BMA250_INT_ENABLE2_REG                  0x17
#define BMA250_INT1_PAD_SEL_REG                 0x19
#define BMA250_INT_DATA_SEL_REG                 0x1A
#define BMA250_INT2_PAD_SEL_REG                 0x1B
#define BMA250_INT_SRC_REG                      0x1E
#define BMA250_INT_SET_REG                      0x20
#define BMA250_INT_CTRL_REG                     0x21
#define BMA250_LOW_DURN_REG                     0x22
#define BMA250_LOW_THRES_REG                    0x23
#define BMA250_LOW_HIGH_HYST_REG                0x24
#define BMA250_HIGH_DURN_REG                    0x25
#define BMA250_HIGH_THRES_REG                   0x26
#define BMA250_SLOPE_DURN_REG                   0x27
#define BMA250_SLOPE_THRES_REG                  0x28
#define BMA250_TAP_PARAM_REG                    0x2A
#define BMA250_TAP_THRES_REG                    0x2B
#define BMA250_ORIENT_PARAM_REG                 0x2C
#define BMA250_THETA_BLOCK_REG                  0x2D
#define BMA250_THETA_FLAT_REG                   0x2E
#define BMA250_FLAT_HOLD_TIME_REG               0x2F
#define BMA250_STATUS_LOW_POWER_REG             0x31
#define BMA250_SELF_TEST_REG                    0x32
#define BMA250_EEPROM_CTRL_REG                  0x33
#define BMA250_SERIAL_CTRL_REG                  0x34
#define BMA250_CTRL_UNLOCK_REG                  0x35
#define BMA250_OFFSET_CTRL_REG                  0x36
#define BMA250_OFFSET_PARAMS_REG                0x37
#define BMA250_OFFSET_FILT_X_REG                0x38
#define BMA250_OFFSET_FILT_Y_REG                0x39
#define BMA250_OFFSET_FILT_Z_REG                0x3A
#define BMA250_OFFSET_UNFILT_X_REG              0x3B
#define BMA250_OFFSET_UNFILT_Y_REG              0x3C
#define BMA250_OFFSET_UNFILT_Z_REG              0x3D
#define BMA250_SPARE_0_REG                      0x3E
#define BMA250_SPARE_1_REG                      0x3F




#define BMA250_ACC_X_LSB__POS           6
#define BMA250_ACC_X_LSB__LEN           2
#define BMA250_ACC_X_LSB__MSK           0xC0
#define BMA250_ACC_X_LSB__REG           BMA250_X_AXIS_LSB_REG

#define BMA250_ACC_X_MSB__POS           0
#define BMA250_ACC_X_MSB__LEN           8
#define BMA250_ACC_X_MSB__MSK           0xFF
#define BMA250_ACC_X_MSB__REG           BMA250_X_AXIS_MSB_REG

#define BMA250_ACC_Y_LSB__POS           6
#define BMA250_ACC_Y_LSB__LEN           2
#define BMA250_ACC_Y_LSB__MSK           0xC0
#define BMA250_ACC_Y_LSB__REG           BMA250_Y_AXIS_LSB_REG

#define BMA250_ACC_Y_MSB__POS           0
#define BMA250_ACC_Y_MSB__LEN           8
#define BMA250_ACC_Y_MSB__MSK           0xFF
#define BMA250_ACC_Y_MSB__REG           BMA250_Y_AXIS_MSB_REG

#define BMA250_ACC_Z_LSB__POS           6
#define BMA250_ACC_Z_LSB__LEN           2
#define BMA250_ACC_Z_LSB__MSK           0xC0
#define BMA250_ACC_Z_LSB__REG           BMA250_Z_AXIS_LSB_REG

#define BMA250_ACC_Z_MSB__POS           0
#define BMA250_ACC_Z_MSB__LEN           8
#define BMA250_ACC_Z_MSB__MSK           0xFF
#define BMA250_ACC_Z_MSB__REG           BMA250_Z_AXIS_MSB_REG

#define BMA250_RANGE_SEL__POS             0
#define BMA250_RANGE_SEL__LEN             4
#define BMA250_RANGE_SEL__MSK             0x0F
#define BMA250_RANGE_SEL__REG             BMA250_RANGE_SEL_REG

#define BMA250_BANDWIDTH__POS             0
#define BMA250_BANDWIDTH__LEN             5
#define BMA250_BANDWIDTH__MSK             0x1F
#define BMA250_BANDWIDTH__REG             BMA250_BW_SEL_REG

#define BMA250_EN_LOW_POWER__POS          6
#define BMA250_EN_LOW_POWER__LEN          1
#define BMA250_EN_LOW_POWER__MSK          0x40
#define BMA250_EN_LOW_POWER__REG          BMA250_MODE_CTRL_REG

#define BMA250_EN_SUSPEND__POS            7
#define BMA250_EN_SUSPEND__LEN            1
#define BMA250_EN_SUSPEND__MSK            0x80
#define BMA250_EN_SUSPEND__REG            BMA250_MODE_CTRL_REG

#define BMA250_GET_BITSLICE(regvar, bitname)\
			((regvar & bitname##__MSK) >> bitname##__POS)


#define BMA250_SET_BITSLICE(regvar, bitname, val)\
	((regvar & ~bitname##__MSK) | ((val<<bitname##__POS)&bitname##__MSK))


/* range and bandwidth */

#define BMA250_RANGE_2G                 0
#define BMA250_RANGE_4G                 1
#define BMA250_RANGE_8G                 2
#define BMA250_RANGE_16G                3


#define BMA250_BW_7_81HZ        0x08
#define BMA250_BW_15_63HZ       0x09
#define BMA250_BW_31_25HZ       0x0A
#define BMA250_BW_62_50HZ       0x0B
#define BMA250_BW_125HZ         0x0C
#define BMA250_BW_250HZ         0x0D
#define BMA250_BW_500HZ         0x0E
#define BMA250_BW_1000HZ        0x0F

/* mode settings */

#define BMA250_MODE_NORMAL      0
#define BMA250_MODE_LOWPOWER    1
#define BMA250_MODE_SUSPEND     2


#ifdef BMA250_ENABLE_IRQ
   #define BMA250_DETECT_PIN	28
   #define   INTERRUPT_INTERVAL		HZ/50	
#endif

#define BMA250_IOC_MAGIC 'B'
#define BMA250_READ_ACCEL_XYZ		_IOWR(BMA250_IOC_MAGIC,1,signed char)
#define BMA250_GET_OFFSET_XYZ		_IOWR(BMA250_IOC_MAGIC,2,signed char)
#ifdef CONFIG_LCT_AW550
#define BMA250_READ_HW_VER_CHIP_ID		_IOWR(BMA250_IOC_MAGIC,3,signed char)
#define BMA250_IOC_MAXNR  4
#else
#define BMA250_IOC_MAXNR  3
#endif

struct bma250acc{
	short	x,
		y,
		z;
} ;

struct bma250_data {
	struct i2c_client *bma250_client;
	atomic_t delay;
	atomic_t enable;
	unsigned char mode;
	struct input_dev *input;
	struct bma250acc value;
	struct mutex value_mutex;
	struct mutex enable_mutex;
	struct mutex mode_mutex;
#ifndef BMA250_ENABLE_IRQ
	struct delayed_work work;
#else
	struct work_struct irq_work;
	atomic_t	device_state;
	int 		irq;
	struct workqueue_struct *bma250_wq;
	struct timer_list my_timer;
	atomic_t 	read_enable;
#endif

};
static struct i2c_client *bma250_client = NULL;

static int bma250_suspend(struct device *dev);
static int bma250_resume(struct device *dev);
#ifdef BMA250_ENABLE_IRQ
static int bma250_enable_interrupt(struct i2c_client *client, int enable);
#endif

static int bma250_smbus_read_byte(struct i2c_client *client,
		unsigned char reg_addr, unsigned char *data)
{
	s32 dummy;
	dummy = i2c_smbus_read_byte_data(client, reg_addr);
	if (dummy < 0)
		return -1;
	*data = dummy & 0x000000ff;

	return 0;
}

static int bma250_smbus_write_byte(struct i2c_client *client,
		unsigned char reg_addr, unsigned char *data)
{
	s32 dummy;
	dummy = i2c_smbus_write_byte_data(client, reg_addr, *data);
	if (dummy < 0)
		return -1;
	return 0;
}

static int bma250_smbus_read_byte_block(struct i2c_client *client,
		unsigned char reg_addr, unsigned char *data, unsigned char len)
{
	s32 dummy;
	dummy = i2c_smbus_read_i2c_block_data(client, reg_addr, len, data);
	if (dummy < 0)
		return -1;
	return 0;
}


static int  bma250_set_wdt(struct i2c_client *client)
{	
	int comres = 0;
	unsigned char data1 = 0;

	comres = bma250_smbus_read_byte(client,
			BMA250_SERIAL_CTRL_REG, &data1);
	
	printk(KERN_ERR "1: bma250_set_wdt --data1 = %x\n",data1);

	data1 |= 0x06;

	comres = bma250_smbus_write_byte(client, 
			BMA250_SERIAL_CTRL_REG, &data1);

	printk(KERN_ERR "2: bma250_set_wdt --data1 = %x\n",data1);
	return comres;

}

static int bma250_set_mode(struct i2c_client *client, unsigned char Mode)
{
	int comres = 0;
	unsigned char data1 = 0;

	if (client == NULL) {
		comres = -1;
	} else{
		if (Mode < 3) {
			comres = bma250_smbus_read_byte(client,
					BMA250_EN_LOW_POWER__REG, &data1);
			switch (Mode) {
			case BMA250_MODE_NORMAL:
				data1  = BMA250_SET_BITSLICE(data1,
					BMA250_EN_LOW_POWER, 0);
				data1  = BMA250_SET_BITSLICE(data1,
					BMA250_EN_SUSPEND, 0);
				break;
			case BMA250_MODE_LOWPOWER:
				data1  = BMA250_SET_BITSLICE(data1,
					BMA250_EN_LOW_POWER, 1);
				data1  = BMA250_SET_BITSLICE(data1,
					BMA250_EN_SUSPEND, 0);
				break;
			case BMA250_MODE_SUSPEND:
				data1  = BMA250_SET_BITSLICE(data1,
					BMA250_EN_LOW_POWER, 0);
				data1  = BMA250_SET_BITSLICE(data1,
					BMA250_EN_SUSPEND, 1);
				break;
			default:
				break;
			}

			comres += bma250_smbus_write_byte(client,
					BMA250_EN_LOW_POWER__REG, &data1);
		} else{
			comres = -1;
		}
	}

	return comres;
}
#ifdef BMA250_DEBUG
static int bma250_get_mode(struct i2c_client *client, unsigned char *Mode)
{
	int comres = 0;

	if (client == NULL) {
		comres = -1;
	} else{
		comres = bma250_smbus_read_byte(client,
				BMA250_EN_LOW_POWER__REG, Mode);
		*Mode  = (*Mode) >> 6;
	}

	return comres;
}
#endif
static int bma250_set_range(struct i2c_client *client, unsigned char Range)
{
	int comres = 0;
	unsigned char data1;

	if (client == NULL) {
		comres = -1;
	} else{
		if (Range < 4) {
			comres = bma250_smbus_read_byte(client,
					BMA250_RANGE_SEL_REG, &data1);
			switch (Range) {
			case 0:
				data1  = BMA250_SET_BITSLICE(data1,
						BMA250_RANGE_SEL, 0);
				break;
			case 1:
				data1  = BMA250_SET_BITSLICE(data1,
						BMA250_RANGE_SEL, 5);
				break;
			case 2:
				data1  = BMA250_SET_BITSLICE(data1,
						BMA250_RANGE_SEL, 8);
				break;
			case 3:
				data1  = BMA250_SET_BITSLICE(data1,
						BMA250_RANGE_SEL, 12);
				break;
			default:
					break;
			}
			comres += bma250_smbus_write_byte(client,
					BMA250_RANGE_SEL_REG, &data1);
		} else{
			comres = -1;
		}
	}

	return comres;
}
#ifdef BMA250_DEBUG
static int bma250_get_range(struct i2c_client *client, unsigned char *Range)
{
	int comres = 0;
	unsigned char data;

	if (client == NULL) {
		comres = -1;
	} else{
		comres = bma250_smbus_read_byte(client, BMA250_RANGE_SEL__REG,
				&data);
		data = BMA250_GET_BITSLICE(data, BMA250_RANGE_SEL);
		*Range = data;
	}

	return comres;
}

static int bma250_get_threshold(struct i2c_client *client, unsigned char *Threshold)
{
	int comres = 0;
	unsigned char data;

	if (client == NULL) {
		comres = -1;
	} else{
		comres = bma250_smbus_read_byte(client, BMA250_SLOPE_THRES_REG,
				&data);
		*Threshold = data;
	}

	return comres;
}

static int bma250_set_threshold(struct i2c_client *client, unsigned char Threshold)
{
	int comres = 0;

	if (client == NULL) {
		comres = -1;
	} else{
			comres = bma250_smbus_write_byte(client,
					BMA250_SLOPE_THRES_REG, &Threshold);
	}
	
	return comres;
}

static int bma250_get_duration(struct i2c_client *client, unsigned char *Duration)
{
	int comres = 0;
	unsigned char data;

	if (client == NULL) {
		comres = -1;
	} else{
		comres = bma250_smbus_read_byte(client, BMA250_SLOPE_DURN_REG,
				&data);
		*Duration = data;
	}

	return comres;
}

static int bma250_set_duration(struct i2c_client *client, unsigned char Duration)
{
	int comres = 0;

	if (client == NULL) {
		comres = -1;
	} else{
		if (Duration < 4) {
			comres += bma250_smbus_write_byte(client,
					BMA250_SLOPE_DURN_REG, &Duration);
		} else{
			comres = -1;
		}
	}

	return comres;
}
#endif

static int bma250_set_bandwidth(struct i2c_client *client, unsigned char BW)
{
	int comres = 0;
	unsigned char data;
	int Bandwidth = 0;

	if (client == NULL) {
		comres = -1;
	} else{
		if (BW < 8) {
			switch (BW) {
			case 0:
				Bandwidth = BMA250_BW_7_81HZ;
				break;
			case 1:
				Bandwidth = BMA250_BW_15_63HZ;
				break;
			case 2:
				Bandwidth = BMA250_BW_31_25HZ;
				break;
			case 3:
				Bandwidth = BMA250_BW_62_50HZ;
				break;
			case 4:
				Bandwidth = BMA250_BW_125HZ;
				break;
			case 5:
				Bandwidth = BMA250_BW_250HZ;
				break;
			case 6:
				Bandwidth = BMA250_BW_500HZ;
				break;
			case 7:
				Bandwidth = BMA250_BW_1000HZ;
				break;
			default:
					break;
			}
			comres = bma250_smbus_read_byte(client,
					BMA250_BANDWIDTH__REG, &data);
			data = BMA250_SET_BITSLICE(data, BMA250_BANDWIDTH,
					Bandwidth);
			comres += bma250_smbus_write_byte(client,
					BMA250_BANDWIDTH__REG, &data);
		} else{
			comres = -1;
		}
	}

	return comres;
}
#ifdef BMA250_DEBUG
static int bma250_get_bandwidth(struct i2c_client *client, unsigned char *BW)
{
	int comres = 0;
	unsigned char data;

	if (client == NULL) {
		comres = -1;
	} else{
		comres = bma250_smbus_read_byte(client, BMA250_BANDWIDTH__REG,
				&data);
		data = BMA250_GET_BITSLICE(data, BMA250_BANDWIDTH);
		if (data <= 8) {
			*BW = 0;
		} else{
			if (data >= 0x0F)
				*BW = 7;
			else
				*BW = data - 8;

		}
	}

	return comres;
}
#endif
static int bma250_read_accel_xyz(struct i2c_client *client,
							struct bma250acc *acc)
{
	int comres;
	unsigned char data[6];
	if (client == NULL) {
		comres = -1;
	} else{
		comres = bma250_smbus_read_byte_block(client,
				BMA250_ACC_X_LSB__REG, data, 6);

		acc->x = BMA250_GET_BITSLICE(data[0], BMA250_ACC_X_LSB)
			|(BMA250_GET_BITSLICE(data[1],
				BMA250_ACC_X_MSB)<<BMA250_ACC_X_LSB__LEN);
		acc->x = acc->x << (sizeof(short)*8-(BMA250_ACC_X_LSB__LEN
					+ BMA250_ACC_X_MSB__LEN));
		acc->x = acc->x >> (sizeof(short)*8-(BMA250_ACC_X_LSB__LEN
					+ BMA250_ACC_X_MSB__LEN));
		acc->y = BMA250_GET_BITSLICE(data[2], BMA250_ACC_Y_LSB)
			| (BMA250_GET_BITSLICE(data[3],
				BMA250_ACC_Y_MSB)<<BMA250_ACC_Y_LSB__LEN);
		acc->y = acc->y << (sizeof(short)*8-(BMA250_ACC_Y_LSB__LEN
					+ BMA250_ACC_Y_MSB__LEN));
		acc->y = acc->y >> (sizeof(short)*8-(BMA250_ACC_Y_LSB__LEN
					+ BMA250_ACC_Y_MSB__LEN));

		acc->z = BMA250_GET_BITSLICE(data[4], BMA250_ACC_Z_LSB)
			| (BMA250_GET_BITSLICE(data[5],
				BMA250_ACC_Z_MSB)<<BMA250_ACC_Z_LSB__LEN);
		acc->z = acc->z << (sizeof(short)*8-(BMA250_ACC_Z_LSB__LEN
					+ BMA250_ACC_Z_MSB__LEN));
		acc->z = acc->z >> (sizeof(short)*8-(BMA250_ACC_Z_LSB__LEN
					+ BMA250_ACC_Z_MSB__LEN));
	}

	return comres;
}

/*	read command for BMA250 device file	*/
static ssize_t bma250_read(struct file *file, char __user *buf, size_t count, loff_t *offset)
{
	struct bma250acc acc;
	int ret;
	if( bma250_client == NULL )
	{
		printk(KERN_ERR "I2C driver not install\n");
		return -1;
	}

	bma250_read_accel_xyz(bma250_client,&acc);

	if( count != sizeof(acc) )
	{
		return -1;
	}
	ret = copy_to_user(buf,&acc, sizeof(acc));
	if( ret != 0 )
	{
#if defined(BMA250_DEBUG) || defined(BMA250_DEBUG_MODULE)
	printk(KERN_INFO "BMA250: copy_to_user result: %d\n", ret);
#endif
	}
	return sizeof(acc);
}
#ifdef BMA250_ENABLE_IRQ
static void timer_enable_irq(unsigned long para)
{
	struct bma250_data *data = (struct bma250_data *)para;
	if(data != NULL)
		enable_irq(data->irq);
}

static irqreturn_t bma250_irq_handler(int irq, void * _id)
{

	struct bma250_data *data;
	if(bma250_client != NULL){
		 data = i2c_get_clientdata(bma250_client);
	        if(data != NULL){
			disable_irq_nosync(data->irq);
			queue_work(data->bma250_wq, &data->irq_work);
			mod_timer(&(data->my_timer), jiffies + INTERRUPT_INTERVAL);			
	        }
	}
#if defined(BMA250_DEBUG)
        printk("BMA250: irq handler\n");
#endif
	return IRQ_HANDLED;
}
#endif

#define GSENSOR_OFFSET  7

static void bma250_work_func(struct work_struct *work)
{
#ifndef BMA250_ENABLE_IRQ
	struct bma250_data *bma250 = container_of((struct delayed_work *)work,
			struct bma250_data, work);
	static struct bma250acc acc;
//	static struct bma250acc offset;
	unsigned long delay = msecs_to_jiffies(atomic_read(&bma250->delay));
#else
	struct bma250_data *bma250 = container_of(work,
			struct bma250_data, irq_work);
	static struct bma250acc acc;
	int xoffset, yoffset, zoffset;	
#endif

	bma250_read_accel_xyz(bma250->bma250_client, &acc);
//	pr_info("[%s] acc: %3d %3d %3d\n",__FUNCTION__,acc.x,acc.y,acc.z);
#ifdef BMA250_DEBUG
	input_report_abs(bma250->input, ABS_X, acc.x);
	input_report_abs(bma250->input, ABS_Y, acc.y);
	input_report_abs(bma250->input, ABS_Z, acc.z);
	input_sync(bma250->input);
#endif
	mutex_lock(&bma250->value_mutex);
#if defined(BMA250_ENABLE_IRQ)
	xoffset = (acc.x -bma250->value.x) > 0 ? (acc.x -bma250->value.x) : (bma250->value.x - acc.x);
	yoffset = (acc.y-bma250->value.y) > 0 ? (acc.y -bma250->value.y) : (bma250->value.y - acc.y);
	zoffset = (acc.z -bma250->value.z) > 0 ? (acc.z -bma250->value.z) : (bma250->value.z - acc.z);
	if(xoffset > GSENSOR_OFFSET || yoffset > GSENSOR_OFFSET  || zoffset > GSENSOR_OFFSET ){
		memcpy(&bma250->value, &acc, sizeof(acc));
		//bma250->value.x = xyz.x;
		//bma250->value.y = xyz.y;
		//bma250->value.z = xyz.z;
		atomic_set(&(bma250->read_enable), 1);
	}	
#else
	//offset.x=(bma250->value.x-acc.x)>0?(bma250->value.x-acc.x):(acc.x-bma250->value.x);
	//offset.y=(bma250->value.y-acc.y)>0?(bma250->value.y-acc.y):(acc.x-bma250->value.y);
	//offset.z=(bma250->value.z-acc.z)>0?(bma250->value.z-acc.z):(acc.z-bma250->value.z);
	//if(offset.x>GSENSOR_OFFSET||offset.y>GSENSOR_OFFSET||offset.z>GSENSOR_OFFSET)
		bma250->value = acc;
#endif
	mutex_unlock(&bma250->value_mutex);
#ifndef BMA250_ENABLE_IRQ
	schedule_delayed_work(&bma250->work, delay);
#endif
}
static void bma250_set_enable(struct device *dev, int enable)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct bma250_data *bma250 = i2c_get_clientdata(client);
	int pre_enable = atomic_read(&bma250->enable);

	mutex_lock(&bma250->enable_mutex);
	if (enable) {
		if (pre_enable ==0) {
			bma250_set_mode(client, 
							BMA250_MODE_NORMAL);
#ifndef BMA250_ENABLE_IRQ
			schedule_delayed_work(&bma250->work,
				msecs_to_jiffies(atomic_read(&bma250->delay)));
#else
			enable_irq(bma250->irq);
			bma250_enable_interrupt(client,1);
#endif
			atomic_set(&bma250->enable, 1);
		}
		
	} else {
		if (pre_enable ==1) {
			bma250_set_mode(client, 
							BMA250_MODE_SUSPEND);
#ifndef BMA250_ENABLE_IRQ
			cancel_delayed_work_sync(&bma250->work);
#else
			cancel_work_sync(&bma250->irq_work);
			disable_irq(bma250->irq);
			bma250_enable_interrupt(client,0);
#endif
			atomic_set(&bma250->enable, 0);
		} 
	}
	mutex_unlock(&bma250->enable_mutex);
	
}
#ifdef BMA250_DEBUG
static ssize_t bma250_range_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	unsigned char data;
	struct i2c_client *client = to_i2c_client(dev);
	struct bma250_data *bma250 = i2c_get_clientdata(client);

	if (bma250_get_range(bma250->bma250_client, &data) < 0)
		return sprintf(buf, "Read error\n");

	return sprintf(buf, "%d\n", data);
}

static ssize_t bma250_range_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	unsigned long data;
	int error;
	struct i2c_client *client = to_i2c_client(dev);
	struct bma250_data *bma250 = i2c_get_clientdata(client);

	error = strict_strtoul(buf, 10, &data);
	if (error)
		return error;
	if (bma250_set_range(bma250->bma250_client, (unsigned char) data) < 0)
		return -EINVAL;

	return count;
}

static ssize_t bma250_bandwidth_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	unsigned char data;
	struct i2c_client *client = to_i2c_client(dev);
	struct bma250_data *bma250 = i2c_get_clientdata(client);

	if (bma250_get_bandwidth(bma250->bma250_client, &data) < 0)
		return sprintf(buf, "Read error\n");

	return sprintf(buf, "%d\n", data);

}

static ssize_t bma250_bandwidth_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	unsigned long data;
	int error;
	struct i2c_client *client = to_i2c_client(dev);
	struct bma250_data *bma250 = i2c_get_clientdata(client);

	error = strict_strtoul(buf, 10, &data);
	if (error)
		return error;
	if (bma250_set_bandwidth(bma250->bma250_client,
						 (unsigned char) data) < 0)
		return -EINVAL;

	return count;
}

static ssize_t bma250_mode_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	unsigned char data;
	struct i2c_client *client = to_i2c_client(dev);
	struct bma250_data *bma250 = i2c_get_clientdata(client);

	if (bma250_get_mode(bma250->bma250_client, &data) < 0)
		return sprintf(buf, "Read error\n");

	return sprintf(buf, "%d\n", data);
}

static ssize_t bma250_mode_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	unsigned long data;
	int error;
	struct i2c_client *client = to_i2c_client(dev);
	struct bma250_data *bma250 = i2c_get_clientdata(client);

	error = strict_strtoul(buf, 10, &data);
	if (error)
		return error;
	if (bma250_set_mode(bma250->bma250_client, (unsigned char) data) < 0)
		return -EINVAL;

	return count;
}


static ssize_t bma250_value_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct input_dev *input = to_input_dev(dev);
	struct bma250_data *bma250 = input_get_drvdata(input);
	struct bma250acc acc_value;

	mutex_lock(&bma250->value_mutex);
	acc_value = bma250->value;
	mutex_unlock(&bma250->value_mutex);

	return sprintf(buf, "%d %d %d\n", acc_value.x, acc_value.y,
			acc_value.z);
}

static ssize_t bma250_delay_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct bma250_data *bma250 = i2c_get_clientdata(client);

	return sprintf(buf, "%d\n", atomic_read(&bma250->delay));

}

static ssize_t bma250_delay_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	unsigned long data;
	int error;
	struct i2c_client *client = to_i2c_client(dev);
	struct bma250_data *bma250 = i2c_get_clientdata(client);

	error = strict_strtoul(buf, 10, &data);
	if (error)
		return error;
	if (data > BMA250_MAX_DELAY)
		data = BMA250_MAX_DELAY;
	atomic_set(&bma250->delay, (unsigned int) data);

	return count;
}


static ssize_t bma250_enable_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct bma250_data *bma250 = i2c_get_clientdata(client);

	return sprintf(buf, "%d\n", atomic_read(&bma250->enable));

}



static ssize_t bma250_enable_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	unsigned long data;
	int error;

	error = strict_strtoul(buf, 10, &data);
	if (error)
		return error;
	if ((data == 0)||(data==1)) {
		bma250_set_enable(dev,data);
	}

	return count;
}

static ssize_t bma250_threshold_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	unsigned char data;
	struct i2c_client *client = to_i2c_client(dev);
	struct bma250_data *bma250 = i2c_get_clientdata(client);

	if (bma250_get_threshold(bma250->bma250_client, &data) < 0)
		return sprintf(buf, "Read error\n");

	return sprintf(buf, "%d\n", data);
}

static ssize_t bma250_threshold_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	unsigned long data;
	int error;
	struct i2c_client *client = to_i2c_client(dev);
	struct bma250_data *bma250 = i2c_get_clientdata(client);

	error = strict_strtoul(buf, 10, &data);
	if (error)
		return error;
	if (bma250_set_threshold(bma250->bma250_client, (unsigned char) data) < 0)
		return -EINVAL;

	return count;
}

static ssize_t bma250_duration_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	unsigned char data;
	struct i2c_client *client = to_i2c_client(dev);
	struct bma250_data *bma250 = i2c_get_clientdata(client);

	if (bma250_get_duration(bma250->bma250_client, &data) < 0)
		return sprintf(buf, "Read error\n");

	return sprintf(buf, "%d\n", data);
}

static ssize_t bma250_duration_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	unsigned long data;
	int error;
	struct i2c_client *client = to_i2c_client(dev);
	struct bma250_data *bma250 = i2c_get_clientdata(client);

	error = strict_strtoul(buf, 10, &data);
	if (error)
		return error;
	if (bma250_set_duration(bma250->bma250_client, (unsigned char) data) < 0)
		return -EINVAL;

	return count;
}

static DEVICE_ATTR(range, S_IRUGO|S_IWUSR|S_IWGRP|S_IWOTH,
		bma250_range_show, bma250_range_store);
static DEVICE_ATTR(bandwidth, S_IRUGO|S_IWUSR|S_IWGRP|S_IWOTH,
		bma250_bandwidth_show, bma250_bandwidth_store);
static DEVICE_ATTR(mode, S_IRUGO|S_IWUSR|S_IWGRP|S_IWOTH,
		bma250_mode_show, bma250_mode_store);
static DEVICE_ATTR(value, S_IRUGO,
		bma250_value_show, NULL);
static DEVICE_ATTR(delay, S_IRUGO|S_IWUSR|S_IWGRP|S_IWOTH,
		bma250_delay_show, bma250_delay_store);
static DEVICE_ATTR(enable, S_IRUGO|S_IWUSR|S_IWGRP|S_IWOTH,
		bma250_enable_show, bma250_enable_store);
static DEVICE_ATTR(threshold, S_IRUGO|S_IWUSR|S_IWGRP|S_IWOTH,
		bma250_threshold_show, bma250_threshold_store);
static DEVICE_ATTR(duration, S_IRUGO|S_IWUSR|S_IWGRP|S_IWOTH,
		bma250_duration_show, bma250_duration_store);

static struct attribute *bma250_attributes[] = {
	&dev_attr_range.attr,
	&dev_attr_bandwidth.attr,
	&dev_attr_mode.attr,
	&dev_attr_value.attr,
	&dev_attr_delay.attr,
	&dev_attr_enable.attr,
	&dev_attr_threshold.attr,
	&dev_attr_duration.attr,
	NULL
};

static struct attribute_group bma250_attribute_group = {
	.attrs = bma250_attributes
};

static int bma250_input_init(struct bma250_data *bma250)
{
	struct input_dev *dev;
	int err;

	dev = input_allocate_device();
	if (!dev)
		return -ENOMEM;
	dev->name = SENSOR_NAME;
	dev->id.bustype = BUS_I2C;

	input_set_capability(dev, EV_ABS, ABS_MISC);
	input_set_abs_params(dev, ABS_X, ABSMIN_2G, ABSMAX_2G, 0, 0);
	input_set_abs_params(dev, ABS_Y, ABSMIN_2G, ABSMAX_2G, 0, 0);
	input_set_abs_params(dev, ABS_Z, ABSMIN_2G, ABSMAX_2G, 0, 0);
	input_set_drvdata(dev, bma250);

	err = input_register_device(dev);
	if (err < 0) {
		input_free_device(dev);
		return err;
	}
	bma250->input = dev;

	return 0;
}
static void bma250_input_delete(struct bma250_data *bma250)
{
	struct input_dev *dev = bma250->input;

	input_unregister_device(dev);
	input_free_device(dev);
}
#endif
static int bma250_open(struct inode *inode, struct file *file)
{
#if defined(BMA250_ENABLE_IRQ)
	struct bma250_data *data;
	struct irq_desc *desc = NULL;
	if( bma250_client == NULL)
	{
		printk(KERN_INFO "I2C driver not install\n");
		return -1;
	}
	data = i2c_get_clientdata(bma250_client);
	if(data != NULL && 1 == atomic_add_return(1, &(data->device_state))){
		printk(KERN_INFO "%s, %s\n",__FUNCTION__, "enable irq");
		desc = irq_to_desc(data->irq);
		while(desc->depth > 0){//depth = 0--- irq enabled
			enable_irq(data->irq);
		}
		atomic_set(&data->enable, 1);
	}
#endif//BMA250_ENABLE_IRQ
#ifdef BMA250_DEBUG
	pr_info("[%s]\n",__FUNCTION__);
#endif
	return 0;
}
static int bma250_release(struct inode *inode, struct file *file)
{
#if defined(BMA250_ENABLE_IRQ)
	struct bma250_data *data;
	struct irq_desc *desc = NULL;
	data = i2c_get_clientdata(bma250_client);
	if(data != NULL && 0 == atomic_sub_return(1, &(data->device_state))){
		printk(KERN_INFO "%s, %s\n",__FUNCTION__, "disable irq");
			desc = irq_to_desc(data->irq);
	     		while(desc->depth < 1){//depth == 1---- irq disabled
				disable_irq(data->irq);
			}
		atomic_set(&data->enable, 0);
	}
#endif//BMA250_ENABLE_IRQ
#ifdef BMA250_DEBUG
	pr_info("[%s]\n",__FUNCTION__);
#endif
	return 0;
}
static long bma250_ioctl(struct file *file,
	unsigned int cmd, unsigned long arg)
{
	#ifdef CONFIG_LCT_AW550
	signed char rwbuf[3] ={0};
	#endif
	
	int err = 0;
	struct bma250_data *gsensor;


#if defined(BMA250_DEBUG) 
	pr_info( "%s\n",__FUNCTION__);
#endif

	/* check cmd */
	if(_IOC_TYPE(cmd) != BMA250_IOC_MAGIC)
	{
#if defined(BMA250_DEBUG) 
		pr_err("cmd magic type error\n");
#endif
		return -ENOTTY;
	}
	if(_IOC_NR(cmd) > BMA250_IOC_MAXNR)
	{
#if defined(BMA250_DEBUG) 
		pr_err("cmd number error\n");
#endif
		return -ENOTTY;
	}

	if(_IOC_DIR(cmd) & _IOC_READ)
		err = !access_ok(VERIFY_WRITE,(void __user*)arg, _IOC_SIZE(cmd));
	else if(_IOC_DIR(cmd) & _IOC_WRITE)
		err = !access_ok(VERIFY_READ, (void __user*)arg, _IOC_SIZE(cmd));
	if(err)
	{
#if defined(BMA250_DEBUG) 
		pr_err("cmd access_ok error\n");
#endif
		return -EFAULT;
	}

	if( bma250_client == NULL)
	{
#if defined(BMA250_DEBUG) 
		pr_err("I2C driver not install\n");
#endif
		return -EFAULT;
	}
	switch(cmd)
	{
		#ifdef CONFIG_LCT_AW550
		case BMA250_READ_HW_VER_CHIP_ID:
			   if (SOCINFO_VERSION_MINOR(socinfo_get_platform_version()) == 2) 
			   {
			   	rwbuf[0] = 'b';
			   	rwbuf[1] = 6;
				if(copy_to_user((char*)arg, rwbuf, 3)){					
					return-EFAULT;
				}
			   }
			   else
			   {
			   	rwbuf[0] = 'b';
			   	rwbuf[1] = 4;
				if(copy_to_user((char*)arg, rwbuf, 3)){					
					return-EFAULT;
				}

			   }
			return 0;
		#endif	
		case BMA250_READ_ACCEL_XYZ:
			gsensor=i2c_get_clientdata(bma250_client);
			#if defined(BMA250_ENABLE_IRQ)			
//pr_info("[%s] acc: %d,%d,%d\n",__FUNCTION__,gsensor->value.x,gsensor->value.y,gsensor->value.z);
		if(atomic_sub_and_test(1, &gsensor->read_enable)){
			mutex_lock(&gsensor->value_mutex);
	
			if(copy_to_user((struct bma250acc*)arg,(struct bma250acc*)&gsensor->value,6)!=0)
			{
#if defined(BMA250_DEBUG) 
				pr_err( "copy_to error\n");
#endif
				mutex_unlock(&gsensor->value_mutex);
				return -EFAULT;
			}
			mutex_unlock(&gsensor->value_mutex);
		}else	
		{
			return -EFAULT;
		}
			return err;
		#else
			return err;
		#endif
		default:
			return 0;
	}
	
}
static unsigned int bma250_poll(struct file *fp, poll_table * wait)
{
	#if defined(BMA250_ENABLE_IRQ)	
	struct bma250_data *data;
	data= i2c_get_clientdata(bma250_client);
	if(atomic_read(&(data->read_enable)) > 0){
		return POLLIN | POLLRDNORM;
	}else{
		return 0;
	}
	#else
		return 0;
	#endif
}
static struct file_operations bma250_fops = {
	.owner	= THIS_MODULE,
	.open	= bma250_open,
	.read = bma250_read,
	.release	= bma250_release,
	.unlocked_ioctl = bma250_ioctl,
	.poll		= bma250_poll,
};
static struct miscdevice bma250_dev = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = MISC_DEV_NAME,
	.fops   = &bma250_fops,
};

#ifdef BMA250_ENABLE_IRQ
int  bma250_enable_pins(void)
{
	int ret;
	struct msm_gpio bma250_gpio_table[] = {
		{ GPIO_CFG(BMA250_DETECT_PIN,  0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), "sensor_int" },
	};

	ret = msm_gpios_request_enable(bma250_gpio_table, ARRAY_SIZE(bma250_gpio_table));
	if (ret <0)
		pr_err("failed to request gpios for bma250\n");
	return ret;
}
static int bma250_enable_interrupt(struct i2c_client *client, int enable)
{
	int comres = 0;
	unsigned char data;
	if(enable)
	{
		data = SLOPE_THRESHOLD_VALUE;
		comres |= bma250_smbus_write_byte(client,
				BMA250_SLOPE_THRES_REG, &data);

		data = SLOPE_DURATION_VALUE;
		comres |= bma250_smbus_write_byte(client,
				BMA250_SLOPE_DURN_REG, &data);
		
		comres = bma250_smbus_read_byte(client,
				BMA250_INT_ENABLE1_REG, &data);
		data |= 0x07;
		comres |= bma250_smbus_write_byte(client,
				BMA250_INT_ENABLE1_REG, &data);

		comres = bma250_smbus_read_byte(client,
				BMA250_INT1_PAD_SEL_REG, &data);
		data |= 0x04;
		comres |= bma250_smbus_write_byte(client,
				BMA250_INT1_PAD_SEL_REG, &data);
	}
	else
	{
		comres = bma250_smbus_read_byte(client,
				BMA250_INT_ENABLE1_REG, &data);
		data &= ~0x07;
		comres |= bma250_smbus_write_byte(client,
				BMA250_INT_ENABLE1_REG, &data);

		comres = bma250_smbus_read_byte(client,
				BMA250_INT1_PAD_SEL_REG, &data);
		data &= ~0x04;
		comres |= bma250_smbus_write_byte(client,
				BMA250_INT1_PAD_SEL_REG, &data);
	}
	return comres;
}
#endif

static int bma250_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{
	int err = 0;
	int tempvalue;
	struct bma250_data *data;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		printk(KERN_INFO "i2c_check_functionality error\n");
		goto exit;
	}
	data = kzalloc(sizeof(struct bma250_data), GFP_KERNEL);
	if (!data) {
		err = -ENOMEM;
		pr_err("[%s] memory not enough!\n",__FUNCTION__);
		goto exit;
	}
	/* read chip id */
	tempvalue = 0;
	tempvalue = i2c_smbus_read_word_data(client, BMA250_CHIP_ID_REG);
	if ((tempvalue&0x00FF) == BMA250_CHIP_ID) {
		printk(KERN_INFO "Bosch Sensortec Device detected!\n" \
				"BMA250 registered I2C driver!\n");
	} else if ((tempvalue&0x00FF) == BMA222E_CHIP_ID) {
		printk(KERN_INFO "Bosch Sensortec Device detected!\n" \
				"BMA222E registered I2C driver!\n");
        } 

          else{
		printk(KERN_INFO "Bosch Sensortec Device not found, \
				i2c error %d \n", tempvalue);
		err = -1;
		goto kfree_exit;
	}
	i2c_set_clientdata(client, data);
	data->bma250_client = client;
	bma250_client = client;
	mutex_init(&data->value_mutex);
	mutex_init(&data->mode_mutex);
	mutex_init(&data->enable_mutex);
	bma250_set_bandwidth(client, BMA250_BW_SET);
	bma250_set_range(client, BMA250_RANGE_SET);

	//zrr@2112.03.20
       bma250_set_wdt(client);
	

#ifndef BMA250_ENABLE_IRQ
	INIT_DELAYED_WORK(&data->work, bma250_work_func);
	atomic_set(&data->delay, BMA250_MAX_DELAY);
#else
	if(bma250_enable_pins()<0)
	{
		printk(KERN_ERR "failed to configure the gpio pin!\n");
	}
		
	atomic_set(&(data->device_state), 0);
	init_timer(&(data->my_timer));
	data->my_timer.expires= jiffies + INTERRUPT_INTERVAL;
	data->my_timer.function = timer_enable_irq;
	data->my_timer.data = (unsigned long)data;

/*--------------------init  bma250 irq--------------------------------*/
        data->bma250_wq = create_singlethread_workqueue("atmel_wq");
        if (!data->bma250_wq) {
                printk(KERN_ERR"%s: create workqueue failed\n", __func__);
                err = -ENOMEM;
                goto kfree_exit;
        }
	INIT_WORK(&data->irq_work, bma250_work_func);
	data->irq = gpio_to_irq(BMA250_DETECT_PIN);
	err = request_irq(data->irq, bma250_irq_handler, IRQF_TRIGGER_HIGH, "bma250", data);
	if (err)
	{
		printk(KERN_ERR "could not request irq\n");
		goto kfree_exit;
	}
	bma250_enable_interrupt(client,1);
#endif
	atomic_set(&data->enable, 0);
#ifdef BMA250_DEBUG
	err = bma250_input_init(data);
	if (err < 0)
		goto kfree_exit;


	err = sysfs_create_group(&data->input->dev.kobj,
						 &bma250_attribute_group);

	if (err < 0)
		goto error_sysfs;
#endif



	err = misc_register(&bma250_dev);
	if (err) {
		pr_err("[%s]bma250 register failed, err=%d\n", __FUNCTION__,err);
		return err;
	}
#ifndef BMA250_ENABLE_IRQ
bma250_set_mode(client, BMA250_MODE_NORMAL);
//atomic_set(&data->delay, 40);
schedule_delayed_work(&data->work,
	msecs_to_jiffies(atomic_read(&data->delay)));
atomic_set(&data->enable, 1);
#endif

	return 0;
#ifdef BMA250_DEBUG
error_sysfs:
	bma250_input_delete(data);
#endif
kfree_exit:
	kfree(data);
exit:
	return err;
}

#ifdef CONFIG_TOUCHSCREEN_MCS6024
int bma250_pause_for_tp_update(void)
{
		struct bma250_data *data =i2c_get_clientdata(bma250_client);
		int err = 0;
	
		mutex_lock(&data->enable_mutex);
		if (atomic_read(&data->enable)==1) {
			err=bma250_set_mode(data->bma250_client, BMA250_MODE_SUSPEND);
#ifndef BMA250_ENABLE_IRQ
			cancel_delayed_work_sync(&data->work);
#else
			cancel_work_sync(&data->irq_work);
#endif
		}
		mutex_unlock(&data->enable_mutex);
		return err;

}
EXPORT_SYMBOL(bma250_pause_for_tp_update);

int bma250_resume_for_tp_update(void)
{
		struct bma250_data *data =i2c_get_clientdata(bma250_client);
		int err = 0;
		mutex_lock(&data->enable_mutex);
		if (atomic_read(&data->enable)==1) {
			err=bma250_set_mode(data->bma250_client, BMA250_MODE_NORMAL);
#ifndef BMA250_ENABLE_IRQ
			schedule_delayed_work(&data->work,
				msecs_to_jiffies(atomic_read(&data->delay)));
#else
			schedule_work(&data->irq_work);
#endif
		}
		mutex_unlock(&data->enable_mutex);
		return err;

}
EXPORT_SYMBOL(bma250_resume_for_tp_update);

#endif /* CONFIG_TOUCHSCREEN_MCS6024 */ 

static int bma250_suspend(struct device *dev)
{
	struct bma250_data *data =i2c_get_clientdata(bma250_client);
	int err = 0;

	mutex_lock(&data->enable_mutex);
	if (atomic_read(&data->enable)==1) {
		err=bma250_set_mode(data->bma250_client, BMA250_MODE_SUSPEND);
#ifndef BMA250_ENABLE_IRQ
		cancel_delayed_work_sync(&data->work);
#else
		cancel_work_sync(&data->irq_work);
#endif
	}
	mutex_unlock(&data->enable_mutex);
	return err;
}


static int bma250_resume(struct device *dev)
{
	struct bma250_data *data =i2c_get_clientdata(bma250_client);
	int err = 0;
	mutex_lock(&data->enable_mutex);
	if (atomic_read(&data->enable)==1) {
		err=bma250_set_mode(data->bma250_client, BMA250_MODE_NORMAL);
#ifndef BMA250_ENABLE_IRQ
		schedule_delayed_work(&data->work,
			msecs_to_jiffies(atomic_read(&data->delay)));
#else
		schedule_work(&data->irq_work);
#endif
	}
	mutex_unlock(&data->enable_mutex);
	return err;
}

static int bma250_remove(struct i2c_client *client)
{
	struct bma250_data *data = i2c_get_clientdata(client);

	bma250_set_enable(&client->dev, 0);
#ifdef BMA250_DEBUG
	sysfs_remove_group(&data->input->dev.kobj, &bma250_attribute_group);
	bma250_input_delete(data);
#endif
	kfree(data);
	return 0;
}


static const struct i2c_device_id bma250_id[] = {
	{ SENSOR_NAME, 0 },
	{ }
};

MODULE_DEVICE_TABLE(i2c, bma250_id);
static struct dev_pm_ops bma250_pmops = {
		.resume 		= bma250_resume,
		.suspend		= bma250_suspend,
};
#define BMA250_PMOPS &bma250_pmops
static struct i2c_driver bma250_driver = {
	.driver = {
		.owner	= THIS_MODULE,
		.name	= SENSOR_NAME,
		.pm = BMA250_PMOPS,
	},
	.id_table	= bma250_id,
	.probe		= bma250_probe,
	.remove		= bma250_remove,

};

static int __init BMA250_init(void)
{
	pr_info("BMA250_init\n");
	return i2c_add_driver(&bma250_driver);
}

static void __exit BMA250_exit(void)
{
	i2c_del_driver(&bma250_driver);
}

MODULE_AUTHOR("Albert Zhang <xu.zhang@bosch-sensortec.com>");
MODULE_DESCRIPTION("BMA250 driver");
MODULE_LICENSE("GPL");

module_init(BMA250_init);
module_exit(BMA250_exit);
