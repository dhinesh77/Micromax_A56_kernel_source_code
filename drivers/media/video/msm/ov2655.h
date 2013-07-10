/* Copyright (c) 2010-2011, Code Aurora Forum. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */


/*
[SENSOR]
Sensor Model:
Camera Module:
Lens Model:
Driver IC:      = OV2655
PV Size         = 640 x 480
Cap Size        = 2048 x 1536
Output Format   = YUYV
MCLK Speed      = 24M
PV DVP_PCLK     =
Cap DVP_PCLK    =
PV Frame Rate   = 30fps
Cap Frame Rate  = 7.5fps
I2C Slave ID    =
I2C Mode        = 16Addr, 16Data
*/

#ifndef CAMSENSOR_OV2655
#define CAMSENSOR_OV2655
#define OV2655_WriteSettingTbl(pTbl)     OV2655_WriteRegsTbl(pTbl,sizeof(pTbl)/sizeof(pTbl[0]))

enum ov2655_test_mode_t {
	TEST_OFF,
	TEST_1,
	TEST_2,
	TEST_3
};

enum ov2655_resolution_t {
  QTR_SIZE,
  FULL_SIZE,
  HFR_60FPS,
  HFR_90FPS,
  HFR_120FPS,
  INVALID_SIZE
};
/******************************************************************************
OV2655_WREG, *POV2655_WREG Definition
******************************************************************************/
typedef struct 
{
unsigned int addr;          /*Reg Addr :16Bit*/
unsigned int data;          /*Reg Data :16Bit or 8Bit*/
unsigned int data_format;   /*Reg Data Format:16Bit = 0,8Bit = 1*/
unsigned int delay_mask;
} OV2655_WREG, *POV2655_WREG;

/******************************************************************************
Initial Setting Table
******************************************************************************/
OV2655_WREG ov2655_init_tbl[] =
{
  {0x308c,0x80,1,0},
  {0x308d,0x0e,1,0},
  {0x360b,0x00,1,0},
  {0x30b0,0xff,1,0},
  {0x30b1,0xff,1,0},
  {0x30b2,0x04,1,0},
  {0x300e,0x34,1,0},
  {0x300f,0xa6,1,0},
  {0x3010,0x81,1,0},
  {0x3082,0x01,1,0},
  {0x30f4,0x01,1,0},
  {0x3090,0x43,1,0},
  {0x3091,0xc0,1,0},
  {0x30ac,0x42,1,0},
  {0x30d1,0x08,1,0},
  {0x30a8,0x54,1,0},
  {0x3015,0x02,1,0},
  {0x3093,0x00,1,0},
  {0x307e,0xe5,1,0},
  {0x3079,0x00,1,0},
  {0x30aa,0x52,1,0},
  {0x3017,0x40,1,0},
  {0x30f3,0x83,1,0},
  {0x306a,0x0c,1,0},
  {0x306d,0x00,1,0},
  {0x336a,0x3c,1,0},
  {0x3076,0x6a,1,0},
  {0x30d9,0x95,1,0},
  {0x3016,0x52,1,0},
  {0x3601,0x30,1,0},
  {0x304e,0x88,1,0},
  {0x30f1,0x82,1,0},
  {0x306f,0x14,1,0},
  {0x302a,0x02,1,0},
  {0x302b,0x84,1,0},
  {0x3012,0x10,1,0},
  {0x3011,0x01,1,0},//15fps
  //;AEC/AGC
  {0x3013,0xf7,1,0},
  {0x301c,0x13,1,0},
  {0x301d,0x17,1,0},
  {0x3070,0x5c,1,0},
  {0x3072,0x4d,1,0},
  //;D5060
  {0x30af,0x00,1,0},
  {0x3048,0x1f,1,0},
  {0x3049,0x4e,1,0},
  {0x304a,0x20,1,0},
  {0x304f,0x20,1,0},
  {0x304b,0x02,1,0},
  {0x304c,0x00,1,0},
  {0x304d,0x02,1,0},
  {0x304f,0x20,1,0},
  {0x30a3,0x10,1,0},
  {0x3013,0xf7,1,0},
  {0x3014,0xa4,1,0},
  {0x3071,0x00,1,0},
  {0x3070,0xb9,1,0},
  {0x3073,0x00,1,0},
  {0x3072,0x4d,1,0},
  {0x301c,0x03,1,0},
  {0x301d,0x06,1,0},
  {0x304d,0x42,1,0},
  {0x304a,0x40,1,0},
  {0x304f,0x40,1,0},
  {0x3095,0x07,1,0},
  {0x3096,0x16,1,0},
  {0x3097,0x1d,1,0},
  //;Window Setup
  {0x3020,0x01,1,0},
  {0x3021,0x18,1,0},
  {0x3022,0x00,1,0},
  {0x3023,0x06,1,0},
  {0x3024,0x06,1,0},
  {0x3025,0x58,1,0},
  {0x3026,0x02,1,0},
  {0x3027,0x61,1,0},
  {0x3088,0x02,1,0},
  {0x3089,0x80,1,0},
  {0x308a,0x01,1,0},
  {0x308b,0xe0,1,0},
  {0x3316,0x64,1,0},
  {0x3317,0x25,1,0},
  {0x3318,0x80,1,0},
  {0x3319,0x08,1,0},
  {0x331a,0x28,1,0},
  {0x331b,0x1e,1,0},
  {0x331c,0x00,1,0},
  {0x331d,0x38,1,0},
  {0x3100,0x00,1,0},  
  //awb
  {0x3320,0xfa,1,0},
  {0x3321,0x11,1,0},
  {0x3322,0x92,1,0},
  {0x3323,0x01,1,0},
  {0x3324,0x97,1,0},
  {0x3325,0x02,1,0},
  {0x3326,0xff,1,0},
  {0x3327,0x10,1,0},
  {0x3328,0x11,1,0},
  {0x3329,0x0f,1,0},
  {0x332a,0x58,1,0},
  {0x332b,0x55,1,0},
  {0x332c,0x90,1,0},
  {0x332d,0xc0,1,0},
  {0x332e,0x46,1,0},
  {0x332f,0x2f,1,0},
  {0x3330,0x2f,1,0},
  {0x3331,0x44,1,0},
  {0x3332,0xff,1,0},
  {0x3333,0x00,1,0},
  {0x3334,0xf0,1,0},
  {0x3335,0xf0,1,0},
  {0x3336,0xf0,1,0},
  {0x3337,0x40,1,0},
  {0x3338,0x40,1,0},
  {0x3339,0x40,1,0},
  {0x333a,0x00,1,0},
  {0x333b,0x00,1,0},   
  //cmx
  {0x3380,0x06,1,0},
  {0x3381,0x90,1,0},
  {0x3382,0x18,1,0},
  {0x3383,0x31,1,0},
  {0x3384,0x84,1,0},
  {0x3385,0xb5,1,0},
  {0x3386,0xba,1,0},
  {0x3387,0xd2,1,0},
  {0x3388,0x17,1,0},
  {0x3389,0x9c,1,0},
  {0x338a,0x00,1,0},  
  //gamma 
  {0x334f,0x20,1,0},
  {0x3340,0x06,1,0},
  {0x3341,0x14,1,0},
  {0x3342,0x2b,1,0},
  {0x3343,0x42,1,0},
  {0x3344,0x55,1,0},
  {0x3345,0x65,1,0},
  {0x3346,0x70,1,0},
  {0x3347,0x7c,1,0},
  {0x3348,0x86,1,0},
  {0x3349,0x96,1,0}, 
  {0x334a,0xa3,1,0},  
  {0x334b,0xaf,1,0},  
  {0x334c,0xc4,1,0},  
  {0x334d,0xd7,1,0}, 
  {0x334e,0xe8,1,0},  
  //;Lens correction
  //R
  {0x3350,0x35,1,0},
  {0x3351,0x29,1,0},
  {0x3352,0x08,1,0},
  {0x3353,0x24,1,0},
  {0x3354,0x00,1,0},
  {0x3355,0x85,1,0},
  //G
  {0x3356,0x34,1,0},
  {0x3357,0x29,1,0},
  {0x3358,0x0f,1,0},
  {0x3359,0x1d,1,0},
  {0x335a,0x00,1,0},
  {0x335b,0x85,1,0},
  //B
  {0x335c,0x36,1,0},
  {0x335d,0x2b,1,0},
  {0x335e,0x08,1,0},
  {0x335f,0x1b,1,0},
  {0x3360,0x00,1,0},
  {0x3361,0x85,1,0},
  //lenc gain
  {0x3363,0x03,1,0},
  {0x3364,0x01,1,0},
  {0x3365,0x02,1,0},
  {0x3366,0x00,1,0},
  {0x3362,0x90,1,0},//lenc for binning  
  //uv adjust
  {0x338b,0x08,1,0},
  {0x338c,0x10,1,0},
  {0x338d,0x5f,1,0},  
  //Sharpness/De-noise
  {0x3370,0xd0,1,0},
  {0x3371,0x00,1,0},
  {0x3372,0x00,1,0},
  {0x3373,0x30,1,0},
  {0x3374,0x10,1,0},
  {0x3375,0x10,1,0},
  {0x3376,0x08,1,0},
  {0x3377,0x02,1,0},
  {0x3378,0x04,1,0},
  {0x3379,0x40,1,0},  
  //aec 
  {0x3018,0x70,1,0},
  {0x3019,0x60,1,0},
  {0x301a,0x85,1,0},
  //;BLC
  {0x3069,0x86,1,0},
  {0x307c,0x13,1,0},
  {0x3087,0x02,1,0},
  //;blacksun
  //;Avdd 2.55~3.0V
  {0x3090,0x3b,1,0}, 
  {0x30a8,0x54,1,0},
  {0x30aa,0x82,1,0},
  {0x30a3,0x91,1,0},
  {0x30a1,0x41,1,0},
  //;Other functions
  {0x3300,0xfc,1,0},
  {0x3302,0x11,1,0},
  {0x3400,0x00,1,0},
  {0x3606,0x20,1,0},
  {0x3601,0x30,1,0},
  {0x300e,0x34,1,0},
  {0x30f3,0x83,1,0},
  {0x304e,0x88,1,0},
  
};

/******************************************************************************
Preview Setting Table 30Fps
******************************************************************************/
OV2655_WREG ov2655_preview_tbl_30fps[] =
{
  //pclk=18M 
  //framerate:15fps
  //YUVVGA(640x480)
  {0x300e,0x3a,1,0},
  {0x3010,0x81,1,0},
  {0x3012,0x10,1,0},
  {0x3014,0xac,1,0},
  {0x3015,0x11,1,0}, 
  {0x3016,0x82,1,0},
  {0x3023,0x06,1,0},
  {0x3026,0x02,1,0},
  {0x3027,0x5e,1,0},
  {0x302a,0x02,1,0},
  {0x302b,0xe4,1,0},
  {0x330c,0x00,1,0},
  {0x3301,0xff,1,0},
  {0x3069,0x80,1,0},
  {0x306f,0x14,1,0},
  {0x3088,0x03,1,0},
  {0x3089,0x20,1,0},
  {0x308a,0x02,1,0},
  {0x308b,0x58,1,0},
  {0x308e,0x00,1,0},
  {0x30a1,0x41,1,0},
  {0x30a3,0x80,1,0},
  {0x30d9,0x95,1,0},
  {0x3302,0x11,1,0},
  {0x3317,0x25,1,0},
  {0x3318,0x80,1,0},
  {0x3319,0x08,1,0},
  {0x331d,0x38,1,0},
  {0x3373,0x40,1,0},
  {0x3376,0x05,1,0},
  {0x3362,0x90,1,0},
  //svga->vga
  {0x3302,0x11,1,0},
  {0x3088,0x02,1,0},
  {0x3089,0x80,1,0},
  {0x308a,0x01,1,0},
  {0x308b,0xe0,1,0},
  {0x331a,0x28,1,0},
  {0x331b,0x1e,1,0},
  {0x331c,0x00,1,0},
  {0x3302,0x11,1,0},
  //mipi
  {0x363b,0x01,1,0},
  {0x309e,0x08,1,0},
  {0x3606,0x00,1,0},
  {0x3630,0x35,1,0},
  {0x3086,0x0f,1,0},
  {0x3086,0x00,1,0},
  {0x304e,0x04,1,0},
  {0x363b,0x01,1,0},
  {0x309e,0x08,1,0},
  {0x3606,0x00,1,0},
  {0x3084,0x01,1,0},
  {0x3010,0x81,1,0},
  {0x3011,0x00,1,0},
  {0x3634,0x26,1,0},
  {0x3086,0x0f,1,0},
  {0x3086,0x00,1,0},
  //avoid black screen slash
  {0x3000,0x15,1,0},
  {0x3002,0x02,1,0},
  {0x3003,0x6a,1,0}, 

};

/******************************************************************************
Preview Setting Table 60Fps
******************************************************************************/
OV2655_WREG ov2655_preview_tbl_60fps[] =
{

};

/******************************************************************************
Preview Setting Table 90Fps
******************************************************************************/
OV2655_WREG ov2655_preview_tbl_90fps[] =
{

};

/******************************************************************************
Capture Setting Table
******************************************************************************/
OV2655_WREG ov2655_capture_tbl[]=
{
  //pclk=24M
  //framerate:7.5ps
  {0x300e,0x34,1,0},
  {0x3011,0x01,1,0},
  {0x3010,0x81,1,0},
  {0x3012,0x00,1,0},
  {0x3015,0x02,1,0},
  {0x3016,0xc2,1,0},
  {0x3023,0x0c,1,0},
  {0x3026,0x04,1,0},
  {0x3027,0xbc,1,0},
  {0x302a,0x04,1,0},
  {0x302b,0xd4,1,0},
  {0x3069,0x80,1,0},
  {0x306f,0x54,1,0},
  {0x3088,0x06,1,0},
  {0x3089,0x40,1,0},
  {0x308a,0x04,1,0},
  {0x308b,0xb0,1,0},
  {0x308e,0x64,1,0},
  {0x30a1,0x41,1,0},
  {0x30a3,0x80,1,0},
  {0x30d9,0x95,1,0},
  {0x3302,0x01,1,0},
  {0x3317,0x4b,1,0},
  {0x3318,0x00,1,0},
  {0x3319,0x4c,1,0},
  {0x331d,0x6c,1,0},
  {0x3362,0x80,1,0},
  {0x3373,0x40,1,0},
  {0x3376,0x03,1,0},
};
/******************************************************************************
Contrast Setting
******************************************************************************/
OV2655_WREG ov2655_contrast_lv0_tbl[] =
{
//Contrast -4
  {0x3391,0x04,1,0x04},
  {0x3390,0x45,1,0x04},
  {0x3398,0x14,1,0},
  {0x3399,0x14,1,0},
};
OV2655_WREG ov2655_contrast_lv1_tbl[] =
{
//Contrast -3
  {0x3391,0x04,1,0x04},
  {0x3390,0x45,1,0x04},
  {0x3398,0x14,1,0},
  {0x3399,0x14,1,0},
};
OV2655_WREG ov2655_contrast_lv2_tbl[] =
{
//Contrast -2
  {0x3391,0x04,1,0x04},
  {0x3390,0x45,1,0x04},
  {0x3398,0x18,1,0},
  {0x3399,0x18,1,0},
};
OV2655_WREG ov2655_contrast_lv3_tbl[] =
{
//Contrast -1
  {0x3391,0x04,1,0x04},
  {0x3390,0x45,1,0x04},
  {0x3398,0x1c,1,0},
  {0x3399,0x1c,1,0},
};
OV2655_WREG ov2655_contrast_default_lv4_tbl[] =
{
//Contrast (Default)
  {0x3391,0x04,1,0x04},
  {0x3390,0x41,1,0x04},
  {0x3398,0x20,1,0},
  {0x3399,0x20,1,0},
};
OV2655_WREG ov2655_contrast_lv5_tbl[] =
{
//Contrast +1
  {0x3391,0x04,1,0x04},
  {0x3390,0x45,1,0x04},
  {0x3398,0x24,1,0},
  {0x3399,0x24,1,0},
};
OV2655_WREG ov2655_contrast_lv6_tbl[] =
{
//Contrast +2
  {0x3391,0x04,1,0x04},
  {0x3390,0x45,1,0x04},
  {0x3398,0x28,1,0},
  {0x3399,0x28,1,0},
};
OV2655_WREG ov2655_contrast_lv7_tbl[] =
{
//Contrast +3
  {0x3391,0x04,1,0x04},
  {0x3390,0x45,1,0x04},
  {0x3398,0x2c,1,0},
  {0x3399,0x2c,1,0},
};
OV2655_WREG ov2655_contrast_lv8_tbl[] =
{
//Contrast +4
  {0x3391,0x04,1,0x04},
  {0x3390,0x45,1,0x04},
  {0x3398,0x30,1,0},
  {0x3399,0x30,1,0},
};

/******************************************************************************
Sharpness Setting
******************************************************************************/
OV2655_WREG ov2655_sharpness_lv0_tbl[] =
{
//Sharpness 0
  {0x3306,0x00,1,0x08},
  {0x3376,0x01,1,0},
  {0x3377,0x00,1,0},
  {0x3378,0x10,1,0},
  {0x3379,0x80,1,0},
};
OV2655_WREG ov2655_sharpness_lv1_tbl[] =
{
//Sharpness 1
  {0x3306,0x00,1,0x08},
  {0x3376,0x02,1,0},
  {0x3377,0x00,1,0},
  {0x3378,0x08,1,0},
  {0x3379,0x80,1,0},
};
OV2655_WREG ov2655_sharpness_default_lv2_tbl[] =
{
//Sharpness_Auto (Default)
  {0x3306,0x00,1,0x08},
  {0x3376,0x05,1,0},//0x04
  {0x3377,0x00,1,0},
  {0x3378,0x04,1,0},
  {0x3379,0x80,1,0},
};
OV2655_WREG ov2655_sharpness_lv3_tbl[] =
{
//Sharpness 3
  {0x3306,0x00,1,0x08},
  {0x3376,0x06,1,0},
  {0x3377,0x00,1,0},
  {0x3378,0x04,1,0},
  {0x3379,0x80,1,0},
};
OV2655_WREG ov2655_sharpness_lv4_tbl[] =
{
//Sharpness 4
  {0x3306,0x00,1,0x08},
  {0x3376,0x08,1,0},
  {0x3377,0x00,1,0},
  {0x3378,0x04,1,0},
  {0x3379,0x80,1,0},
};
OV2655_WREG ov2655_sharpness_lv5_tbl[] =
{
//Sharpness 5
  {0x3306,0x00,1,0x08},
  {0x3376,0x0a,1,0},
  {0x3377,0x00,1,0},
  {0x3378,0x04,1,0},
  {0x3379,0x80,1,0},
};
OV2655_WREG ov2655_sharpness_lv6_tbl[] =
{
//Sharpness 5
  {0x3306,0x00,1,0x08},
  {0x3376,0x0c,1,0},
  {0x3377,0x00,1,0},
  {0x3378,0x04,1,0},
  {0x3379,0x80,1,0},
};
OV2655_WREG ov2655_sharpness_lv7_tbl[] =
{
//Sharpness 7
  {0x3306,0x00,1,0x08},
  {0x3376,0x0e,1,0},
  {0x3377,0x00,1,0},
  {0x3378,0x04,1,0},
  {0x3379,0x80,1,0},
};
OV2655_WREG ov2655_sharpness_lv8_tbl[] =
{
//Sharpness 8
  {0x3306,0x00,1,0x08},
  {0x3376,0x10,1,0},
  {0x3377,0x00,1,0},
  {0x3378,0x04,1,0},
  {0x3379,0x80,1,0},
};

/******************************************************************************
Saturation Setting
******************************************************************************/
OV2655_WREG ov2655_saturation_lv0_tbl[] =
{
//Saturation x0.25
  {0x3301,0x80,1,0x80},
  {0x3391,0x02,1,0x02},
  {0x3394,0x10,1,0},
  {0x3395,0x10,1,0},
};
OV2655_WREG ov2655_saturation_lv1_tbl[] =
{
//Saturation x0.5
  {0x3301,0x80,1,0x80},
  {0x3391,0x02,1,0x02},
  {0x3394,0x18,1,0},
  {0x3395,0x18,1,0},
};
OV2655_WREG ov2655_saturation_lv2_tbl[] =
{
//Saturation x0.75
  {0x3301,0x80,1,0x80},
  {0x3391,0x02,1,0x02},
  {0x3394,0x20,1,0},
  {0x3395,0x20,1,0},
};
OV2655_WREG ov2655_saturation_lv3_tbl[] =
{
//Saturation x0.75
  {0x3301,0x80,1,0x80},
  {0x3391,0x02,1,0x02},
  {0x3394,0x30,1,0},
  {0x3395,0x30,1,0},
};
OV2655_WREG ov2655_saturation_default_lv4_tbl[] =
{
//Saturation x1 (Default)
  {0x3301,0x80,1,0x80},
  {0x3391,0x02,1,0x02},
  {0x3394,0x40,1,0},
  {0x3395,0x40,1,0},
};
OV2655_WREG ov2655_saturation_lv5_tbl[] =
{
//Saturation x1.25
  {0x3301,0x80,1,0x80},
  {0x3391,0x02,1,0x02},
  {0x3394,0x50,1,0},
  {0x3395,0x50,1,0},
};
OV2655_WREG ov2655_saturation_lv6_tbl[] =
{
//Saturation x1.5
  {0x3301,0x80,1,0x80},
  {0x3391,0x02,1,0x02},
  {0x3394,0x58,1,0},
  {0x3395,0x58,1,0},
};
OV2655_WREG ov2655_saturation_lv7_tbl[] =
{
//Saturation x1.25
  {0x3301,0x80,1,0x80},
  {0x3391,0x02,1,0x02},
  {0x3394,0x60,1,0},
  {0x3395,0x60,1,0},
};
OV2655_WREG ov2655_saturation_lv8_tbl[] =
{
//Saturation x1.5
  {0x3301,0x80,1,0x80},
  {0x3391,0x02,1,0x02},
  {0x3394,0x68,1,0},
  {0x3395,0x68,1,0},
};

/******************************************************************************
Brightness Setting
******************************************************************************/
OV2655_WREG ov2655_brightness_lv0_tbl[] =
{
//Brightness -4
  {0x3391,0x04,1,0x04},
  {0x3390,0x49,1,0x08},
  {0x339a,0x28,1,0},
};
OV2655_WREG ov2655_brightness_lv1_tbl[] =
{
//Brightness -3
  {0x3391,0x04,1,0x04},
  {0x3390,0x49,1,0x08},
  {0x339a,0x20,1,0},
};
OV2655_WREG ov2655_brightness_lv2_tbl[] =
{
//Brightness -2
  {0x3391,0x04,1,0x04},
  {0x3390,0x49,1,0x08},
  {0x339a,0x18,1,0},
};
OV2655_WREG ov2655_brightness_lv3_tbl[] =
{
//Brightness -1
  {0x3391,0x04,1,0x04},
  {0x3390,0x49,1,0x08},
  {0x339a,0x10,1,0},
};
OV2655_WREG ov2655_brightness_default_lv4_tbl[] =
{
//Brightness 0 (Default)
  {0x3391,0x04,1,0x04},
  {0x3390,0x41,1,0x08},
  {0x339a,0x00,1,0},
};
OV2655_WREG ov2655_brightness_lv5_tbl[] =
{
//Brightness +1
  {0x3391,0x04,1,0x04},
  {0x3390,0x41,1,0x08},
  {0x339a,0x10,1,0},
};
OV2655_WREG ov2655_brightness_lv6_tbl[] =
{
//Brightness +2
  {0x3391,0x04,1,0x04},
  {0x3390,0x41,1,0x08},
  {0x339a,0x20,1,0},
};
OV2655_WREG ov2655_brightness_lv7_tbl[] =
{
//Brightness +3
  {0x3391,0x04,1,0x04},
  {0x3390,0x41,1,0x08},
  {0x339a,0x28,1,0},
};
OV2655_WREG ov2655_brightness_lv8_tbl[] =
{
//Brightness +4
  {0x3391,0x04,1,0x04},
  {0x3390,0x41,1,0x08},
  {0x339a,0x30,1,0},
};

/******************************************************************************
Exposure Compensation Setting
******************************************************************************/
OV2655_WREG ov2655_exposure_compensation_lv0_tbl[]=
{
//Exposure Compensation +1.7EV
  {0x3018,0x98,1,0},
  {0x3019,0x88,1,0},
  {0x301a,0xd4,1,0},
};
OV2655_WREG ov2655_exposure_compensation_lv1_tbl[]=
{
//Exposure Compensation +1.0EV
  {0x3018,0x88,1,0},
  {0x3019,0x78,1,0},
  {0x301a,0xd4,1,0},
};
OV2655_WREG ov2655_exposure_compensation_lv2_default_tbl[]=
{
//Exposure Compensation default
  {0x3018,0x70,1,0},//0x78
  {0x3019,0x60,1,0},//0x68
  {0x301a,0x85,1,0},//0xa5
};
OV2655_WREG ov2655_exposure_compensation_lv3_tbl[]=
{
//Exposure Compensation -1.0EV
  {0x3018,0x6a,1,0},
  {0x3019,0x5a,1,0},
  {0x301a,0xd4,1,0},
};
OV2655_WREG ov2655_exposure_compensation_lv4_tbl[]=
{
//Exposure Compensation -1.7EV
  {0x3018,0x5a,1,0},
  {0x3019,0x4a,1,0},
  {0x301a,0xc2,1,0},
};

/******************************************************************************
ISO TYPE Setting
******************************************************************************/
OV2655_WREG ov2655_iso_type_auto[]=
{
//ISO Auto
};
OV2655_WREG ov2655_iso_type_100[]=
{
//ISO 100
};
OV2655_WREG ov2655_iso_type_200[]=
{
//ISO 200
};
OV2655_WREG ov2655_iso_type_400[]=
{
//ISO 400
};
OV2655_WREG ov2655_iso_type_800[]=
{
//ISO 800
};
OV2655_WREG ov2655_iso_type_1600[]=
{
//ISO 1600
};

/******************************************************************************
Auto Expourse Weight Setting
******************************************************************************/
OV2655_WREG ov2655_ae_average_tbl[] =
{
//Whole Image Average
  {0x3030,0x55,1,0},
  {0x3031,0x55,1,0},
  {0x3032,0x55,1,0},
  {0x3033,0x55,1,0},
};
OV2655_WREG ov2655_ae_centerweight_tbl[] =
{
//Whole Image Center More weight
  {0x3030,0x00,1,0},
  {0x3031,0x3c,1,0},
  {0x3032,0x00,1,0},
  {0x3033,0x00,1,0},
};

/******************************************************************************
Light Mode Setting
******************************************************************************/
OV2655_WREG ov2655_wb_Auto[]=
{
//CAMERA_WB_AUTO                //1
  {0x3306,0x00,1,0x02},
};
OV2655_WREG ov2655_wb_custom[]=
{
//CAMERA_WB_CUSTOM             //2
  {0x3306,0x02,1,0x02},
  {0x3337,0x44,1,0},
  {0x3338,0x40,1,0},
  {0x3339,0x70,1,0},
};
OV2655_WREG ov2655_wb_inc[]=
{
//CAMERA_WB_INCANDESCENT       //3
  {0x3306,0x02,1,0x02},
  {0x3337,0x52,1,0},
  {0x3338,0x40,1,0},
  {0x3339,0x58,1,0},
};
OV2655_WREG ov2655_wb_fluorescent[]=
{
//CAMERA_WB_FLUORESCENT       //4
  {0x3306,0x02,1,0x02},
  {0x3337,0x44,1,0},
  {0x3338,0x40,1,0},
  {0x3339,0x70,1,0},
};
OV2655_WREG ov2655_wb_daylight[]=
{
//CAMERA_WB_DAYLIGHT          //5
  {0x3306,0x02,1,0x02},
  {0x3337,0x5e,1,0},
  {0x3338,0x40,1,0},
  {0x3339,0x46,1,0},
};
OV2655_WREG ov2655_wb_cloudy[]=
{
//CAMERA_WB_CLOUDY_DAYLIGHT   //6
//  {0x3306,0x02,1,0x02},
//  {0x3337,0x68,1,0},
//  {0x3338,0x40,1,0},
//  {0x3339,0x4e,1,0},
  {0x3306,0x02,1,0x02},
  {0x3337,0x78,1,0},
  {0x3338,0x50,1,0},
  {0x3339,0x48,1,0},
};
OV2655_WREG ov2655_wb_twilight[]=
{
//CAMERA_WB_TWILIGHT          //7
};
OV2655_WREG ov2655_wb_shade[]=
{
//CAMERA_WB_SHADE             //8
};

/******************************************************************************
EFFECT Setting
******************************************************************************/
OV2655_WREG ov2655_effect_normal_tbl[] =
{
//CAMERA_EFFECT_OFF           0
  {0x3391,0x00,1,0x78},
};
OV2655_WREG ov2655_effect_mono_tbl[] =
{
//CAMERA_EFFECT_MONO          1
  {0x3391,0x20,1,0x78},
};
OV2655_WREG ov2655_effect_negative_tbl[] =
{
//CAMERA_EFFECT_NEGATIVE      2
  {0x3391,0x40,1,0x78},
};
OV2655_WREG ov2655_effect_solarize_tbl[] =
{
//CAMERA_EFFECT_SOLARIZE      3
};
OV2655_WREG ov2655_effect_sepia_tbl[] =
{
//CAMERA_EFFECT_SEPIA         4
  {0x3391,0x18,1,0x78},
  {0x3396,0x40,1,0},
  {0x3397,0xa6,1,0},
};
OV2655_WREG ov2655_effect_posterize_tbl[] =
{
//CAMERA_EFFECT_POSTERIZE     5
};
OV2655_WREG ov2655_effect_whiteboard_tbl[] =
{
//CAMERA_EFFECT_WHITEBOARD    6
};
OV2655_WREG ov2655_effect_blackboard_tbl[] =
{
//CAMERA_EFFECT_BLACKBOARD    7
};
OV2655_WREG ov2655_effect_aqua_tbl[] =
{
//CAMERA_EFFECT_AQUA          8
};
OV2655_WREG ov2655_effect_bw_tbl[] =
{
//CAMERA_EFFECT_BW            10
};
OV2655_WREG ov2655_effect_bluish_tbl[] =
{
//CAMERA_EFFECT_BLUISH        12
  {0x3391,0x18,1,0x78},
  {0x3396,0xa0,1,0},
  {0x3397,0x40,1,0},
};
OV2655_WREG ov2655_effect_reddish_tbl[] =
{
//CAMERA_EFFECT_REDDISH       13
  {0x3391,0x18,1,0x78},
  {0x3396,0x80,1,0},
  {0x3397,0xc0,1,0},
};
OV2655_WREG ov2655_effect_greenish_tbl[] =
{
//CAMERA_EFFECT_GREENISH      14
  {0x3391,0x18,1,0x78},
  {0x3396,0x60,1,0},
  {0x3397,0x60,1,0},
};

/******************************************************************************
AntiBanding Setting
******************************************************************************/
OV2655_WREG ov2655_antibanding_auto_tbl[] =
{
//Auto-XCLK24MHz
 // {0x3014,0xc0,1,0xc0},
    {0x3014,0x80,1,0xc0},
};
OV2655_WREG ov2655_antibanding_50z_tbl[] =
{
//Band 50Hz
   {0x3014,0x80,1,0xc0},
};
OV2655_WREG ov2655_antibanding_60z_tbl[] =
{
//Band 60Hz
 // {0x3014,0x00,1,0xc0},
    {0x3014,0x80,1,0xc0},
};

/******************************************************************************
Lens_shading Setting
******************************************************************************/
OV2655_WREG ov2655_lens_shading_on_tbl[] =
{
//Lens_shading On
  {0x3300,0x08,1,0x08},
};

OV2655_WREG ov2655_lens_shading_off_tbl[] =
{
//Lens_shading Off
  {0x3300,0x00,1,0x08},
};
/******************************************************************************
Auto Focus Setting
******************************************************************************/
OV2655_WREG ov2655_afinit_tbl[] =
{

};
#endif /* CAMSENSOR_OV2655 */