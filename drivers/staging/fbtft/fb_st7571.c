/*
 * FB driver for the ST7571 LCD Controller
 *
 * Copyright (C) 2013 Frank Chung
 * 
 * This driver based on fbtft drivers solution created by Noralf Thonnes
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/gpio.h>
#include <linux/spi/spi.h>
#include <linux/delay.h>

#include "fbtft.h"

#define DRVNAME         "fb_st7571"
#define WIDTH	        128
#define HEIGHT	        128
#define PAGES	        (HEIGHT / 8)
#define TXBUFLEN        (128 * 16)
#define BPP             8
#define FPS             30
#define DEFAULT_GAMMA   "0x00,0x06,0x22"

#define COLUMNS_PER_PAGE    128

#define BIAS_RATIO_BASE     0x50

static bool reverse = false;

static void lcd_set_address(struct fbtft_par *par, unsigned char page, unsigned char column)
{
    gpio_set_value(par->gpio.dc, 1);
    write_reg(par, 0xB0 | page);
    write_reg(par, ((column >> 4) & 0x0F) | 0x10);
    write_reg(par, column & 0x0F);
}

static int init_display(struct fbtft_par *par)
{
    // /* Set CS active high */
    // par->spi->mode |= SPI_CS_HIGH;
    // ret = spi_setup(par->spi);
    // if (ret) {
    // 	dev_err(par->info->device, "Could not set SPI_CS_HIGH\n");
    // 	return ret;
    // }

    fbtft_par_dbg(DEBUG_INIT_DISPLAY, par, "%s()\n", __func__);

    par->fbtftops.reset(par);

    write_reg(par, 0x2F);    /* power control, turn on all internal power supply */

    write_reg(par, 0xAE);    /* display OFF */

    write_reg(par, 0x38);    /* set mode, 2-byte instruction */
    write_reg(par, 0xBF);    /* mode: 85Hz, booster efficiency level 4 */

    /**
     * COM scan direction (row),    0xC0: normal, 0xC8: reverse
     * SEG scan direction (column), 0xA0: normal, 0xA1: reverse
     */ 
    write_reg(par, 0xC8);
    write_reg(par, 0xA0);

    write_reg(par, 0x40);   /* set display start line, 2-byte instruction */
    write_reg(par, 0x00);

    write_reg(par, 0x44);   /* set COM0, 2-byte instruction */
    write_reg(par, 0x00);

    write_reg(par, 0xAB);   /* turn on internal oscillator */
    write_reg(par, 0x27);   /* set regulator resister */

    write_reg(par, 0x81);
    write_reg(par, 0x22);    /* set contrast, 2-byte instruction, 0x00 ~ 0x3F */
    
    write_reg(par, 0x56);    /* set LCD bias, 0x50~0x57: 1/5 ~ 1/12 */

    write_reg(par, 0x7B);    /* enter extension mode 3 */
    write_reg(par, 0x10);    /* set color mode, 0x10: gray mode, 0x11: black/white mode */
    write_reg(par, 0x00);    /* leave extension mode 3 */

    write_reg(par, 0xAF);    /* display ON */

    return 0;
}

static void set_addr_win(struct fbtft_par *par, int xs, int ys, int xe, int ye)
{
    fbtft_par_dbg(DEBUG_SET_ADDR_WIN, par,
        "%s(xs=%d, ys=%d, xe=%d, ye=%d)\n", __func__, xs, ys, xe, ye);

    // TODO : implement set_addr_win

}

static int set_var(struct fbtft_par *par)
{
    fbtft_par_dbg(DEBUG_INIT_DISPLAY, par, "%s()\n", __func__);

    switch (par->info->var.rotate) {
	case 0:
		write_reg(par, 0xC8);
        write_reg(par, 0xA0);
		break;
	case 180:
		write_reg(par, 0xC0);
        write_reg(par, 0xA1);  
		break;
    default:
        dev_err(par->info->device, "unspported rotate(%d)\n", 
                par->info->var.rotate);
	}

    return 0;
}

static u32 bgr_to_gray(u16 bgr)
{
    u32 r, g, b, gray;

    b = (bgr & 0xf800) >> 11;
    g = (bgr & 0x07e0) >> 5;
    r = bgr & 0x001f;
    gray = (r * 19595 + g * 38469 + b * 7472) >> 16;

    return gray;
}

static int write_vmem(struct fbtft_par *par, size_t offset, size_t len)
{
#define S_BUFLEN    2 * COLUMNS_PER_PAGE * sizeof(u8) + 1

    u16 *vmem16 = (u16 *)(par->info->screen_buffer);
    u8 *buf = NULL;
    u8 *hbuf = NULL;
    u32 gray;
    int x, y, i;
    int ret = 0;

    fbtft_par_dbg(DEBUG_WRITE_VMEM, par, "%s()\n", __func__);

    buf = devm_kzalloc(par->info->device, S_BUFLEN, GFP_KERNEL);
    if (!buf)
    {
        ret = -1;
        goto _alloc_fail;
    }
    hbuf = buf;

    for (y = 0; y < PAGES; y++) {
        memset(hbuf, 0, S_BUFLEN);
        for (x = 0; x < COLUMNS_PER_PAGE; x++) {
            for (i = 0; i < 8; i++) {
                //*buf |= (vmem16[(y * 8 + i) * COLUMNS_PER_PAGE + x] ? 1 : 0) << i;
                // if(vmem16[(y * 8 + i) * COLUMNS_PER_PAGE + x])
                //     *buf |= BIT(i);
                gray = bgr_to_gray(vmem16[(y * 8 + i) * COLUMNS_PER_PAGE + x]);

                if (!reverse)
                {
                    if(gray < 12)
                    {
                        buf[2 * x] |= 1 << i;
                        buf[2 * x + 1] |= 1 << i;
                    }
                    else if(gray > 12 && gray < 25)
                        buf[2 * x + 1] |= 1 << i;
                    else if (gray >= 25 && gray < 37)
                        buf[2 * x] |= 1 << i;
                    else
                        ;
                }
                else
                {
                    if(gray < 12)
                        ;
                    else if(gray > 12 && gray < 25)
                        buf[2 * x] |= 1 << i;
                    else if (gray >= 25 && gray < 37)
                        buf[2 * x + 1] |= 1 << i;
                    else
                    {
                        buf[2 * x] |= 1 << i;
                        buf[2 * x + 1] |= 1 << i;
                    }
                }
            }
        }

        lcd_set_address(par, (u8)y, 0);
        gpio_set_value(par->gpio.dc, 1);
        par->fbtftops.write(par, hbuf, COLUMNS_PER_PAGE * 2);
    }

    devm_kfree(par->info->device, hbuf);
_alloc_fail:
    return ret;
}

/**
 * Gamma string format:
 *   REVERSE, BIAS, CONTRAST
 */
static int set_gamma(struct fbtft_par *par, unsigned long *curves)
{
    u8 bias_ratio, contrast;

    fbtft_par_dbg(DEBUG_INIT_DISPLAY, par, "%s(reverse=%d, bias=%d, contrast=%d)\n",
                  __func__, (u8)curves[0], (u8)curves[1], (u8)curves[2]);

    reverse = (u8)curves[0] ? true : false;
    bias_ratio = ((u8)curves[1] > 7) ? 7 : (u8)curves[1];
    contrast = ((u8)curves[2] > 63) ? 63 : (u8)curves[2];

    write_reg(par, BIAS_RATIO_BASE | bias_ratio);
    write_reg(par, 0x81);
    write_reg(par, contrast);

    return 0;
}

static struct fbtft_display display = {
    .regwidth = 8,
    .width = WIDTH,
    .height = HEIGHT,
    .gamma_num = 1,
    .gamma_len = 3,
    .gamma = DEFAULT_GAMMA,
    .fbtftops = {
        .init_display = init_display,
        .set_addr_win = set_addr_win,
        .set_var = set_var,
        .write_vmem = write_vmem,
        .set_gamma = set_gamma,
    },
    .backlight = 1,
    .fps = FPS,
};

FBTFT_REGISTER_DRIVER(DRVNAME, "sitronix,st7571", &display);

MODULE_ALIAS("spi:" DRVNAME);
MODULE_ALIAS("platform:" DRVNAME);
MODULE_ALIAS("spi:jlx128128g");
MODULE_ALIAS("platform:jlx128128g");

MODULE_DESCRIPTION("FB driver for the ST7571 LCD Controller");
MODULE_AUTHOR("Frank");
MODULE_LICENSE("GPL");
