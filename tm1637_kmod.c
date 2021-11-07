/*-
 * SPDX-License-Identifier: BSD-2-Clause-FreeBSD
 *
 * Copyright (C) 2020 Alexander Mishin
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 */

//#define FDT

#include <sys/types.h>
#include <sys/module.h>
#include <sys/lock.h>
#include <sys/mutex.h>
#include <sys/systm.h>  /* uprintf */
#include <sys/sysctl.h>
#include <sys/conf.h>   /* cdevsw struct */
#include <sys/param.h>  /* defines used in kernel.h */
#include <sys/kernel.h> /* types used in module initialization */
#include <sys/bus.h>
#include <sys/uio.h>    /* uio struct */
#include <sys/malloc.h>
#include <sys/gpio.h>

#include <dev/iicbus/iiconf.h>
#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>
#include <dev/gpio/gpiobusvar.h>
#include <dev/fdt/fdt_pinctrl.h>

#include "include/dev/tm1637/tm1637.h"

#define TM1637_DESC		"TM1637 %u digits 7 segments display with %s"
#define TM1637_CDEV_NAME	"tm1637"
#define TM1637_SCL_PROPERTY	"scl-gpios"
#define TM1637_SDA_PROPERTY	"sda-gpios"
#define TM1637_SCL_IDX		0
#define TM1637_SDA_IDX		1
#define TM1637_MIN_PINS		2
#define TM1637_BUSFREQ		200000

#define TM1637_ADDRESS_AUTO	0x40
#define TM1637_ADDRESS_FIXED	0x44
#define TM1637_ADDRESS_START	0xc0
#define TM1637_DISPLAY_OFF	0x80
#define TM1637_DISPLAY_CTRL	0x88

#define TM1637_8TH_COLON	0
#define TM1637_8TH_DOT		1

#define TM1637_LOCK_INIT(sc)	\
    mtx_init(&(sc)->lock, "tm1637 mtx", NULL, MTX_DEF)
#define TM1637_LOCK_DESTROY(sc)	\
    mtx_destroy(&(sc)->lock)
#define TM1637_LOCK(sc)		\
    mtx_lock(&(sc)->lock)
#define TM1637_UNLOCK(sc)	\
    mtx_unlock(&(sc)->lock)

#define IICBB_DEBUG
#ifdef IICBB_DEBUG
static int i2c_debug = 0;

SYSCTL_DECL(_hw_i2c);
SYSCTL_INT(_hw_i2c, OID_AUTO, iicbb_debug, CTLFLAG_RWTUN,
    &i2c_debug, 0, "Enable i2c bit-banging driver debug");

#define I2C_DEBUG(x)	do {		\
		if (i2c_debug) (x);	\
	} while (0)
#else
#define I2C_DEBUG(x)
#endif

#define	GPIOBB_GETSDA(dev)	(gpiobb_getsda(dev))
#define	GPIOBB_SETSDA(dev, x)	(gpiobb_setsda(dev, x))
#define	GPIOBB_GETSCL(dev)	(gpiobb_getscl(dev))
#define	GPIOBB_SETSCL(dev, x)	(gpiobb_setscl(dev, x))

/* Based on the SMBus specification. */
#define	DEFAULT_SCL_LOW_TIMEOUT	(25 * 1000)

static const u_char char_code[10] = {
    CHR_0, CHR_1, CHR_2, CHR_3, CHR_4,
    CHR_5, CHR_6, CHR_7, CHR_8, CHR_9
};

/* Buffer struct */
MALLOC_DEFINE(M_TM1637BUF, "tm1637buffer", "Buffer for tm1637 display");
MALLOC_DEFINE(M_TM1637COD, "tm1637codes", "Masks array for tm1637 display");

struct tm1637_buf_t {
    uint8_t				 length;
    uint8_t				*codes;
    uint8_t				 text_length;
    unsigned char			*text;
};

struct tm1637_dispinfo {
    const uint8_t			 number;
    const uint8_t			*position;
    const uint8_t			 buffer_length;
    const uint8_t			 type;
};

struct tm1637_softc {
    device_t				 dev;
    phandle_t				 node;
    gpio_pin_t				 sclpin;
    gpio_pin_t				 sdapin;
    uint8_t				 brightness;
    bool				 on;
    bool				 needupdate;
    bool				 inuse;
    struct tm1637_buf_t			 buffer;
    u_int				 udelay; /* signal toggle delay in usec */
    u_int				 scl_low_timeout;
    struct mtx				 lock;
    const struct tm1637_dispinfo	*info;
    struct cdev				*cdev;
};

static int tm1637_probe(device_t);
static int tm1637_attach(device_t);
static int tm1637_detach(device_t);

static void gpiobb_setsda(const device_t, const bool);
static void gpiobb_setscl(const device_t, const bool);
static int gpiobb_getsda(const device_t);
static int gpiobb_getscl(const device_t);
static int gpiobb_getack(const device_t);

static int gpiobb_start(const device_t);
static int gpiobb_stop(const device_t);

static void tm1637_set_speed(struct tm1637_softc *, const u_int);
static void tm1637_sysctl_register(struct tm1637_softc *);

static int digit_convert(u_char *, const unsigned char);
static int buffer_convert(struct tm1637_softc *);
static int is_raw_command(struct tm1637_softc *);

static int tm1637_send_command(struct tm1637_softc *, const uint8_t);
static int tm1637_send_data1(struct tm1637_softc *, const uint8_t);
static int tm1637_send_data(struct tm1637_softc *, size_t, const uint8_t);

static int tm1637_write(struct cdev*, struct uio*, int ioflag);
static int tm1637_ioctl(struct cdev*, u_long cmd, caddr_t data, int fflag, struct thread*);
static void tm1637_display_on(struct tm1637_softc*);
static void tm1637_display_off(struct tm1637_softc*);
static void tm1637_set_brightness(struct tm1637_softc*, const uint8_t);

/* Sequences of digit positions for a display type */
static const uint8_t dig4pos[] = { 0, 1, 2, 3 };
static const uint8_t dig6pos[] = { 2, 1, 0, 5, 4, 3 };

static const struct tm1637_dispinfo tm1637_disp_infos[] = {
    {  4, dig4pos, 6,  TM1637_8TH_COLON },
    {  4, dig4pos, 10, TM1637_8TH_DOT },
    {  6, dig6pos, 14, TM1637_8TH_DOT },
};

static const struct ofw_compat_data tm1637_compat_data[] = {
    {"tm1637-4-colon",		(uintptr_t)&tm1637_disp_infos[0]},
    {"tm1637-4-dots",		(uintptr_t)&tm1637_disp_infos[1]},
    {"tm1637-6-dots",		(uintptr_t)&tm1637_disp_infos[2]},
    {"tm1637",			(uintptr_t)&tm1637_disp_infos[0]},
    {NULL,			(uintptr_t)NULL}
};

OFWBUS_PNP_INFO(tm1637_compat_data);
SIMPLEBUS_PNP_INFO(tm1637_compat_data);

static device_method_t tm1637_methods[] = {
    /* Device interface */
    DEVMETHOD(device_probe,	tm1637_probe),
    DEVMETHOD(device_attach,	tm1637_attach),
    DEVMETHOD(device_detach,	tm1637_detach),

    DEVMETHOD_END
};

static devclass_t tm1637_devclass;

DEFINE_CLASS_0(tm1637, tm1637_driver, tm1637_methods, sizeof(struct tm1637_softc));

#ifdef FDT
DRIVER_MODULE(tm1637, simplebus, tm1637_driver, tm1637_devclass, NULL, NULL);
#endif

DRIVER_MODULE(tm1637, gpiobus, tm1637_driver, tm1637_devclass, NULL, NULL);
MODULE_VERSION(tm1637, 1);
MODULE_DEPEND(tm1637, gpiobus, 1, 1, 1);

#ifdef FDT

static int
tm1637_fdt_get_params(struct tm1637_softc *sc)
{
    pcell_t param_cell;
    ssize_t param_found;
    uint32_t param;

    param_found = OF_getencprop(sc->node, "default-brightness-level", &param_cell, sizeof(param_cell));

    if (param_found > 0) {
	param = (uint32_t)param_cell;

	if (param > BRIGHT_BRIGHTEST) {
	    device_printf(sc->dev,
			"could not acquire correct brightness level from DTS\n");
	    return (EINVAL);
	}

	sc->brightness = (uint8_t)param;
	if (bootverbose)
	    device_printf(sc->dev,
			"Acquired brightness level: %u %s\n", sc->brightness, "from DTS");
    }
    else {
	sc->brightness = BRIGHT_TYPICAL;
	if (bootverbose)
	    device_printf(sc->dev,
			"Acquired brightness level: %u %s\n", sc->brightness, "by default");
    }

    return (0);
}

static int
tm1637_setup_fdt_pins(struct tm1637_softc *sc)
{
    int err;

    /*
     * Historically, we used the first two array elements of the gpios
     * property.  The modern bindings specify separate scl-gpios and
     * sda-gpios properties.  We cope with whichever is present.
     */
    if (OF_hasprop(sc->node, "gpios")) {
	if ((err = gpio_pin_get_by_ofw_idx(sc->dev, sc->node, TM1637_SCL_IDX, &sc->sclpin)) != 0)
	{
	    device_printf(sc->dev, "invalid gpios property for index:%d\n", TM1637_SCL_IDX);
	    return (err);
	}
	if ((err = gpio_pin_get_by_ofw_idx(sc->dev, sc->node, TM1637_SDA_IDX, &sc->sdapin)) != 0)
	{
	    device_printf(sc->dev, "invalid gpios property for index:%d\n", TM1637_SDA_IDX);
	    return (err);
	}
    } else {
	if ((err = gpio_pin_get_by_ofw_property(sc->dev, sc->node, TM1637_SCL_PROPERTY, &sc->sclpin)) != 0)
	{
	    device_printf(sc->dev, "missing %s property\n", TM1637_SCL_PROPERTY);
	    return (err);
	}
	if ((err = gpio_pin_get_by_ofw_property(sc->dev, sc->node, TM1637_SDA_PROPERTY, &sc->sdapin)) != 0)
	{
	    device_printf(sc->dev, "missing %s property\n", TM1637_SDA_PROPERTY);
	    return (err);
	}
    }

    /* Get pin configuration from pinctrl-0 and ignore errors */
    err = fdt_pinctrl_configure(sc->dev, 0);
#ifdef DEBUG
    if (err != 0)
	uprintf("No valid pinctrl-0 configuration. Error: %d\n", err);
    else
	uprintf("pinctrl-0 configration is OK\n");
#endif

    return (0);
}
#endif /* FDT */

/*
 * Sysctl parameter: brightness
 */
static int
tm1637_brightness_sysctl(SYSCTL_HANDLER_ARGS)
{
    struct tm1637_softc *sc = arg1;
    uint8_t brightness = sc->brightness;
    int error;

    error = SYSCTL_OUT(req, &brightness, sizeof(brightness));
    if (error != 0 || req->newptr == NULL)
	return (error);

    error = SYSCTL_IN(req, &brightness, sizeof(brightness));
    if (error != 0)
	return (error);

    if (brightness > 7)
	return (EINVAL);

    tm1637_set_brightness(sc, brightness);

    return (0);
}

/*
 * Sysctl parameter: on
 */
static int
tm1637_set_on_sysctl(SYSCTL_HANDLER_ARGS)
{
    struct tm1637_softc *sc = arg1;
    uint8_t _on = sc->on;
    int error = 0;

    error = SYSCTL_OUT(req, &_on, sizeof(_on));
    if (error != 0 || req->newptr == NULL)
	return (error);

    error = SYSCTL_IN(req, &_on, sizeof(_on));
    if (error != 0)
	return (error);

    switch(_on)
    {
	case 0:
	    tm1637_display_off(sc);
	    break;
	case 1:
	    tm1637_display_on(sc);
	    break;
	default:
	    error = EINVAL;
    }

    return (error);
}

static int
digit_convert(u_char *tm1637_digit, const unsigned char c)
{
    switch (c) {
    case '#': // Skip the digit but clear a dot or a colon
	*tm1637_digit &= 0x7f;
	break;
    case ' ':
	*tm1637_digit = CHR_SPACE;
	break;
    case '-':
	*tm1637_digit = CHR_GYPHEN;
	break;
    default:
	if ((c >= '0') && (c <= '9'))
	    *tm1637_digit = char_code[c&0x0f];
	else
	    return (-1);
    }

    return (0);
}

/* Convert the date written to device */
static int
buffer_convert(struct tm1637_softc *sc)
{
    const struct tm1637_dispinfo *info = sc->info;
    const uint8_t *position = info->position;
    const struct tm1637_buf_t *buf = &sc->buffer;
    const unsigned char *text = buf->text;
    uint8_t *codes = buf->codes;
    int8_t n = info->number;
    int8_t i = buf->length;
    int8_t p;

    if (info->type == TM1637_8TH_DOT) {
	/* tm1637 with decimals
	 * Reverse order for right aligned result */
	while (n-- > 0) {
	    unsigned char c;

	    p = position[n];

	    /* When the text buffer is cleared before the digits run out,
	     * a leading space will be displayed.
	     */
	    if (i <= 0) {
		codes[p] = CHR_SPACE;
		continue;
	    }

	    c = text[--i];
	    if (c == '.') {
		/* If a dot check for buffer is not empty,
		 * get a number followed by the dot (backward, of course),
		 * and set its eighth bit to light the dot segment
		 */
		if ((i <= 0) ||
		    (digit_convert(&codes[p], text[--i]) < 0))
			return (-1);
		/* Set a dot segment */
		codes[p] |= 0x80;
	    }
	    else
		if (digit_convert(&codes[p], c) < 0)
		    return (-1);

	}
    }
    else
    if(buf->length <= info->number) {
	/* tm1637 4 digits with colon. Format: integer
	 * Reverse order for right aligned result */
	while (n-- > 0) {
	    p = position[n];

	    /* When the text buffer is cleared before the digits run out,
	     * a leading space will be displayed.
	     */
	    if (i <= 0)
		codes[p] = CHR_SPACE;
	    else
		if (digit_convert(&codes[p], text[--i]) < 0)
		    return (-1);
	}
    }
    else
    if (buf->length == 5) {
	unsigned char clockpoint;
	/* tm1637 4 digits with colon. Format: clock
	 * Reverse order for right aligned result */
	while (n-- > 0) {
	    p = position[n];

	    /* Get a clockpoint and go for a digit before */
	    if (i == 3)
		clockpoint = text[--i];

	    if (digit_convert(&codes[p], text[--i]) < 0)
		return (-1);
	}

	if (clockpoint == ':') {
	    /* Set a dot segment */
	    p = position[TM1637_COLON_POSITION - 1];
	    codes[p] |= 0x80;
	}
	else
	if (clockpoint == ' ') {
	    /* Clear a dot segment */
	    p = position[TM1637_COLON_POSITION - 1];
	    codes[p] &= 0x7f;
	}
	else
	    return (-1);
    }

    /* The text buffer must be empty now */
    if (i > 0)
	return (-1);

    return (0);
}

static int
is_raw_command(struct tm1637_softc *sc)
{
    struct tm1637_buf_t *buf = &sc->buffer;
    size_t start, stop, tmp;
    uint8_t c;

    switch (buf->text[0]) {
    /* Send one byte at a fixed position */
    case TM1637_ADDRESS_FIXED:
	if ((buf->length < 3) ||
	   ((start = buf->text[1]^TM1637_ADDRESS_START) >= sc->info->number))
	    return (-1);

	buf->codes[start] = buf->text[2];
	tm1637_send_data1(sc, start);
	break;
    /* Send up to 4 bytes at an autoincremented position */
    case TM1637_ADDRESS_AUTO:
	if ((buf->length < 3) ||
	   ((start = buf->text[1]^TM1637_ADDRESS_START) >= sc->info->number))
	    return (-1);

	tmp = start;
	stop = MIN(buf->length-2, sc->info->number);
	for (size_t i=2; tmp<stop; tmp++)
	    buf->codes[tmp] = buf->text[i++];

	tm1637_send_data(sc, start, stop);
	break;
    /* Send one byte command to turn display off */
    case TM1637_DISPLAY_OFF:
	tm1637_display_off(sc);
	break;
    default:
	/* Send one byte command to light display with the bright level 0..7 */
	if ((c = (buf->text[0]^TM1637_DISPLAY_CTRL)) <= BRIGHT_BRIGHTEST) {
	    sc->brightness = c;
	    tm1637_display_on(sc);
	    break;
	}
	return (-1);
    }

    return (0);
}

static void
gpiobb_setsda(const device_t dev, const bool sda)
{
    struct tm1637_softc *sc = device_get_softc(dev);

#ifndef I2C_LIKE
    gpio_pin_setflags(sc->sdapin, GPIO_PIN_OUTPUT|GPIO_PIN_OPENDRAIN);
    gpio_pin_set_active(sc->sdapin, sda);
#else
    if (val) {
	gpio_pin_setflags(sc->sdapin, GPIO_PIN_INPUT);
    } else {
	gpio_pin_setflags(sc->sdapin, GPIO_PIN_OUTPUT|GPIO_PIN_OPENDRAIN);
	gpio_pin_set_active(sc->sdapin, 0);
    }
#endif
}

static void
gpiobb_setscl(const device_t dev, const bool scl)
{
    struct tm1637_softc *sc = device_get_softc(dev);

#ifndef I2C_LIKE
    gpio_pin_setflags(sc->sclpin, GPIO_PIN_OUTPUT|GPIO_PIN_OPENDRAIN);
    gpio_pin_set_active(sc->sclpin, scl);
#else
    if (val) {
	gpio_pin_setflags(sc->sclpin, GPIO_PIN_INPUT);
    } else {
	gpio_pin_setflags(sc->sclpin, GPIO_PIN_OUTPUT|GPIO_PIN_OPENDRAIN);
	gpio_pin_set_active(sc->sclpin, 0);
    }
#endif
}

static int
gpiobb_getsda(const device_t dev)
{
    struct tm1637_softc *sc = device_get_softc(dev);
    bool val;

    gpio_pin_setflags(sc->sdapin, GPIO_PIN_INPUT);
    gpio_pin_is_active(sc->sdapin, &val);
    return (val);
}

static int
gpiobb_getscl(const device_t dev)
{
    struct tm1637_softc *sc = device_get_softc(dev);
    bool val;

    gpio_pin_setflags(sc->sclpin, GPIO_PIN_INPUT);
    gpio_pin_is_active(sc->sclpin, &val);
    return (val);
}

static int
gpiobb_waitforscl(const device_t dev)
{
    struct tm1637_softc *sc = device_get_softc(dev);
    sbintime_t fast_timeout;
    sbintime_t now, timeout;

    /* Spin for up to 1 ms, then switch to pause. */
    now = sbinuptime();
    fast_timeout = now + SBT_1MS;
    timeout = now + sc->scl_low_timeout * SBT_1US;

    do {
	if (GPIOBB_GETSCL(dev))
	    return (0);
	now = sbinuptime();
    } while (now < fast_timeout);

    do {
	I2C_DEBUG(printf("."));
	pause_sbt("gpiobb-scl-low", SBT_1MS, C_PREL(8), 0);
	if (GPIOBB_GETSCL(dev))
	    return (0);
	now = sbinuptime();
    } while (now < timeout);

    I2C_DEBUG(printf("*"));
    return (IIC_ETIMEOUT);
}

/* Start the high phase of the clock. */
static int
gpiobb_clockin(const device_t dev, const bool sda)
{
    /*
     * Precondition: SCL is low.
     * Action:
     * - set SDA to the value;
     * - release SCL and wait until it's high.
     * The caller is responsible for keeping SCL high for udelay.
     *
     * There should be a data set-up time, 250 ns minimum, between setting
     * SDA and raising SCL.  It's expected that the I/O access latency will
     * naturally provide that delay.
     */
    GPIOBB_SETSDA(dev, sda);
    GPIOBB_SETSCL(dev, 1);

    return (gpiobb_waitforscl(dev));
}

/*
 * End the high phase of the clock and wait out the low phase
 * as nothing interesting happens during it anyway.
 */
static void
gpiobb_clockout(const device_t dev)
{
    struct tm1637_softc *sc = device_get_softc(dev);

    /*
     * Precondition: SCL is high.
     * Action:
     * - pull SCL low and hold for udelay.
     */
    GPIOBB_SETSCL(dev, 0);
    DELAY(sc->udelay);
}

static int
gpiobb_sendbit(const device_t dev, const bool bit)
{
#ifdef I2C_LIKE
    struct tm1637_softc *sc = device_get_softc(dev);
#endif
    int err;

    err = gpiobb_clockin(dev, bit);
    if (err != 0)
	return (err);
#ifdef I2C_LIKE
    DELAY(sc->udelay);
#endif
    gpiobb_clockout(dev);

    return (0);
}

/*
 * Start command or data transmission
 */
static int
gpiobb_start(const device_t dev)
{
    struct tm1637_softc *sc = device_get_softc(dev);

    I2C_DEBUG(printf("<<"));

    /* SCL must be high on the idle bus. */
    if (gpiobb_waitforscl(dev) != 0) {
	I2C_DEBUG(printf("C!\n"));
	return (IIC_EBUSERR);
    }

    /*
     * SDA must be high after the earlier stop condition or the end
     * of Ack/NoAck pulse.
     */
    if (!GPIOBB_GETSDA(dev)) {
	I2C_DEBUG(printf("D!\n"));
	return (IIC_EBUSERR);
    }

    /* Start: SDA high->low. */
    GPIOBB_SETSDA(dev, 0);

    /* Wait the second half of the SCL high phase. */
    DELAY((sc->udelay + 1) / 2);

    /* Pull SCL low to keep the bus reserved. */
    gpiobb_clockout(dev);

    return (0);
}

/*
 * Stop command or data transmission
 */
static int
gpiobb_stop(const device_t dev)
{
    struct tm1637_softc *sc = device_get_softc(dev);
    int err = 0;

    /*
     * Stop: SDA goes from low to high in the middle of the SCL high phase.
     */
    err = gpiobb_clockin(dev, 0);
    if (err != 0)
	return (err);
    DELAY((sc->udelay + 1) / 2);
    GPIOBB_SETSDA(dev, 1);
    DELAY((sc->udelay + 1) / 2);

    I2C_DEBUG(printf("%s>>", err != 0 ? "!" : ""));
    I2C_DEBUG(printf("\n"));

    return (err);
}

static int
gpiobb_getack(const device_t dev)
{
    struct tm1637_softc *sc = device_get_softc(dev);
    int noack, err;
    int t;

    /* Release SDA so that the slave can drive it. */
    err = gpiobb_clockin(dev, 1);
    if (err != 0) {
	I2C_DEBUG(printf("! "));
	return (err);
    }

    /* Sample SDA until ACK (low) or udelay runs out. */
    for (t = 0; t < sc->udelay; t++) {
	noack = GPIOBB_GETSDA(dev);
	if (!noack)
	    break;
	DELAY(1);
    }

#ifdef I2C_LIKE
    DELAY(sc->udelay - t);
#endif
    gpiobb_clockout(dev);

    I2C_DEBUG(printf("%c ", noack ? '-' : '+'));

    return (noack ? IIC_ENOACK : 0);
}

/*
 * Send a byte of information w/ ack checking
 */
static int
tm1637_gpio_sendbyte(const device_t dev, const uint8_t data)
{
    int err;
    uint8_t i;

    // LSB first
    for(i=0; i<=7; i++) {
	err = gpiobb_sendbit(dev, (data & (1 << i)) != 0);
	if (err != 0) {
	    I2C_DEBUG(printf("w!"));
	    return (err);
	}
    }
    I2C_DEBUG(printf("w%02x", data));
    return (gpiobb_getack(dev));
}

/*
 * Sends one byte command with a retry if no acnowledge occurs
 * Returns zero on success
 */
static int
tm1637_send_command(struct tm1637_softc *sc, const uint8_t cmd)
{
    const device_t dev = sc->dev;
    int err;

    TM1637_LOCK(sc);

    gpiobb_start(dev);
    err = tm1637_gpio_sendbyte(dev, cmd);
    gpiobb_stop(dev);

    TM1637_UNLOCK(sc);

    return err;
}

/*
 * Sends an address and one byte data
 * Returns zero on success
 */
static int
tm1637_send_data1(struct tm1637_softc *sc, const uint8_t pos)
{
	const device_t dev = sc->dev;
	const uint8_t addr = TM1637_ADDRESS_START + pos;
	const uint8_t data = sc->buffer.codes[pos];

	if (sc->on) {
		TM1637_LOCK(sc);

		gpiobb_start(dev);
		tm1637_gpio_sendbyte(dev, TM1637_ADDRESS_FIXED);
		gpiobb_stop(dev);

		gpiobb_start(dev);
		tm1637_gpio_sendbyte(dev, addr);
		tm1637_gpio_sendbyte(dev, data);
		gpiobb_stop(dev);

		TM1637_UNLOCK(sc);
	}

	return (0);
}

/*
 * Send number of bytes fron first to last address
 * Returns zero on success
 */
static int
tm1637_send_data(struct tm1637_softc *sc, size_t pos, const uint8_t stop)
{
	const device_t dev = sc->dev;
	const uint8_t addr = TM1637_ADDRESS_START + pos;
	const uint8_t *codes = sc->buffer.codes;

	if (sc->on) {
		TM1637_LOCK(sc);

		gpiobb_start(dev);
		tm1637_gpio_sendbyte(dev, TM1637_ADDRESS_AUTO);
		gpiobb_stop(dev);

		gpiobb_start(dev);
		tm1637_gpio_sendbyte(dev, addr);
		while(pos < stop)
			tm1637_gpio_sendbyte(dev, codes[pos++]);
		gpiobb_stop(dev);

		TM1637_UNLOCK(sc);
	}

	return (0);
}

/*
 * Display or clear a clockpoint
 */
static void
tm1637_display_clockpoint(struct tm1637_softc *sc, const bool clockpoint)
{
    const uint8_t p = sc->info->position[TM1637_COLON_POSITION - 1];

    if (clockpoint)
	sc->buffer.codes[p] |= 0x80;
    else
	sc->buffer.codes[p] &= 0x7f;

    tm1637_send_data1(sc, p);
}

/*
 * Send to display all 4 digits (6 bytes will be sended)
 */
static void
tm1637_update_display(struct tm1637_softc *sc)
{
    if(sc->on) {
	tm1637_send_data(sc, 0, sc->info->number);
	/* Clear the flag if it is set */
	if(sc->needupdate)
	    sc->needupdate = false;
    }
    else
	sc->needupdate = true; /* Do nothing but set flag */
}

/*
 * Writes all blanks to a display
 */
static void
tm1637_clear_display(struct tm1637_softc *sc)
{
    uint8_t position = sc->info->number;

    /* Display all blanks */
    while(position > 0)
	sc->buffer.codes[--position] = CHR_SPACE;

    /* If display is off right now then mark it for update when it is on */
    if(sc->on)
	tm1637_send_data(sc, 0, sc->info->number);
    else
	sc->needupdate = true;
}

/*
 * Sets a display on with a brightness value from softc
 */
static void
tm1637_display_on(struct tm1637_softc *sc)
{
    if(!sc->on) {
	sc->on = true;
	tm1637_send_data(sc, 0, sc->info->number);
    }
    /* Light the display anyway */
    tm1637_send_command(sc, TM1637_DISPLAY_CTRL|sc->brightness);
}

/*
 * Sets a display on with a brightness value as parameter
 */
static void
tm1637_set_brightness(struct tm1637_softc *sc, const uint8_t brightness)
{
    /* If brightness level is correct */
    if (brightness <= BRIGHT_BRIGHTEST) {
	/* Store a value as a current brightness now */
	sc->brightness = brightness;
	/* Do not send a command unless a display is already on */
	if(sc->on)
	    tm1637_send_command(sc, TM1637_DISPLAY_CTRL|brightness);
    }
}

/*
 * Set a display off
 */
static void
tm1637_display_off(struct tm1637_softc *sc)
{
    /* Do nothing is always off */
    if(sc->on) {
	sc->on = false;
	tm1637_send_command(sc, TM1637_DISPLAY_OFF);
    }
}

/*
 * Sets time to the display
 */
static void
tm1637_set_clock(struct tm1637_softc *sc, struct tm1637_clock_t clock)
{
    const uint8_t *position = sc->info->position;
    uint8_t *codes = sc->buffer.codes;
    uint8_t p;
    uint8_t t;

    t = clock.tm_hour / 10;
    p = position[0];
    codes[p] = char_code[t&0x0f];
    t = clock.tm_hour % 10;
    p = position[1];
    codes[p] = char_code[t];
    t = clock.tm_min / 10;
    p = position[2];
    codes[p] = char_code[t&0x0f];
    t = clock.tm_min % 10;
    p = position[3];
    codes[p] = char_code[t];

    /* Clockpoint */
    if (clock.tm_colon) {
	p = position[TM1637_COLON_POSITION - 1];
	codes[p] |= 0x80;
    }

    tm1637_update_display(sc);
}

static int
tm1637_setup_hinted_pins(struct tm1637_softc *sc)
{
    device_t busdev;
    const char *busname, *devname;
    int err, numpins, sclnum, sdanum, unit;

    devname = device_get_name(sc->dev);
    unit = device_get_unit(sc->dev);
    busdev = device_get_parent(sc->dev);

    /*
     * If there is not an "at" hint naming our actual parent, then we
     * weren't instantiated as a child of gpiobus via hints, and we thus
     * can't access ivars that only exist for such children.
     */
    if (resource_string_value(devname, unit, "at", &busname) != 0 ||
	(strcmp(busname, device_get_nameunit(busdev)) != 0 &&
	 strcmp(busname, device_get_name(busdev)) != 0)) {
	    return (ENOENT);
    }

    /* Make sure there were hints for at least two pins. */
    numpins = gpiobus_get_npins(sc->dev);
    if (numpins < TM1637_MIN_PINS) {

#ifdef FDT
	/*
	 * Be silent when there are no hints on FDT systems; the FDT
	 * data will provide the pin config (we'll whine if it doesn't).
	 */
	if (numpins == 0) {
	    return (ENOENT);
	}
#endif
	device_printf(sc->dev, 
	    "invalid pins hint; it must contain at least %d pins\n",
	    TM1637_MIN_PINS);
	return (EINVAL);
    }

    /*
     * Our parent bus has already parsed the pins hint and it will use that
     * info when we call gpio_pin_get_by_child_index().  But we have to
     * handle the scl/sda index hints that tell us which of the two pins is
     * the clock and which is the data.  They're optional, but if present
     * they must be a valid index (0 <= index < numpins).
     */
    if ((err = resource_int_value(devname, unit, "scl", &sclnum)) != 0)
	sclnum = TM1637_SCL_IDX;
    else if (sclnum < 0 || sclnum >= numpins) {
	device_printf(sc->dev, "invalid scl hint %d\n", sclnum);
	return (EINVAL);
    }
    if ((err = resource_int_value(devname, unit, "sda", &sdanum)) != 0)
	sdanum = TM1637_SDA_IDX;
    else if (sdanum < 0 || sdanum >= numpins) {
	device_printf(sc->dev, "invalid sda hint %d\n", sdanum);
	return (EINVAL);
    }

    /* Allocate gpiobus_pin structs for the pins we found above. */
    if ((err = gpio_pin_get_by_child_index(sc->dev, sclnum, &sc->sclpin)) != 0)
	return (err);

    if ((err = gpio_pin_get_by_child_index(sc->dev, sdanum, &sc->sdapin)) != 0)
	return (err);

    return (0);
}

/* Function prototypes */
static d_open_t      tm1637_open;
static d_close_t     tm1637_close;
static d_write_t     tm1637_write;
static d_ioctl_t     tm1637_ioctl;

/* Character device entry points */
static struct cdevsw tm1637_cdevsw = {
    .d_version = D_VERSION,
    .d_open = tm1637_open,
    .d_close = tm1637_close,
    .d_write = tm1637_write,
    .d_ioctl = tm1637_ioctl,
    .d_name = TM1637_CDEV_NAME,
};

static int
tm1637_ioctl(struct cdev *cdev, u_long cmd, caddr_t data, int fflag, struct thread *td)
{
    struct tm1637_softc *sc = cdev->si_drv1; // Stored here on tm1637_attach()
    int error = 0;

    switch (cmd) {
    case TM1637IOC_CLEAR:
	tm1637_clear_display(sc);
	break;
    case TM1637IOC_OFF:
	tm1637_display_off(sc);
	break;
    case TM1637IOC_ON:
	tm1637_display_on(sc);
	break;
    case TM1637IOC_SET_BRIGHTNESS:
	tm1637_set_brightness(sc, *(uint8_t*)data);
	break;
    case TM1637IOC_SET_CLOCKPOINT:
	if(sc->on != 0)
	    tm1637_display_clockpoint(sc, *(bool*)data);
	break;
    case TM1637IOC_SET_CLOCK:
	tm1637_set_clock(sc, *(struct tm1637_clock_t*)data);
	break;
    default:
	error = ENOTTY;
	break;
    }

    return (error);
}

static int
tm1637_open(struct cdev *cdev, int oflags __unused, int devtype __unused,
    struct thread *td __unused)
{

#ifdef DEBUG
    uprintf("Opening device \"%s\".\n", tm1637_cdevsw.d_name);
#endif

    return (0);
}

static int
tm1637_close(struct cdev *cdev __unused, int fflag __unused, int devtype __unused,
    struct thread *td __unused)
{

#ifdef DEBUG
    uprintf("Closing device \"%s\".\n", tm1637_cdevsw.d_name);
#endif

    return (0);
}

static int
tm1637_write(struct cdev *cdev, struct uio *uio, int ioflag __unused)
{
    int error;

    struct tm1637_softc *sc = cdev->si_drv1;

    size_t amount;
    off_t uio_offset_saved;

    amount = MIN(uio->uio_resid, (sc->buffer.text_length));
    uio_offset_saved = uio->uio_offset;
    error = uiomove(sc->buffer.text, amount, uio);
    uio->uio_offset = uio_offset_saved;
    sc->buffer.length = amount;

    if (error)
	return (error);

    if ((is_raw_command(sc)) == 0)
	return (0);

    if (buffer_convert(sc) == 0) {
	tm1637_send_data(sc, 0, sc->info->number);
	return (0);
    }

    return (EINVAL);
}

static const struct tm1637_dispinfo *
tm1637_find_dispinfo(const device_t dev)
{
    const struct ofw_compat_data *cdata;
    const char *disptype;

#ifdef FDT
    const struct tm1637_dispinfo *info;

    if (ofw_bus_status_okay(dev)) {
	info = (struct tm1637_dispinfo*)
		ofw_bus_search_compatible(dev, tm1637_compat_data)->ocd_data;
	if (info != NULL)
	    return (info);
    }
#endif

    /* For hinted devices, we must be told the compatible string. */
    disptype = NULL;
    if (resource_string_value(device_get_name(dev), device_get_unit(dev),
		"compatible", &disptype) == 0) {

	/*
	 * Loop through the ofw compat data comparing the hinted chip type to
	 * the compat strings.
	 */
	for (cdata = tm1637_compat_data; cdata->ocd_str != NULL; ++cdata) {
	    if (strcmp(disptype, cdata->ocd_str) == 0)
		break;
	}

	return ((const struct tm1637_dispinfo *)cdata->ocd_data);
    }

    return (NULL);
}

static int
tm1637_probe(device_t dev)
{
    struct tm1637_softc *sc = device_get_softc(dev);
    char desc[50];

    sc->info = tm1637_find_dispinfo(dev);
    if (sc->info != NULL) {
	snprintf(desc, sizeof(desc), TM1637_DESC, sc->info->number,
	    sc->info->type?"decimals":"colon");
	device_set_desc_copy(dev, desc);
#ifdef FDT
	return (BUS_PROBE_DEFAULT);
#else
	return (BUS_PROBE_NOWILDCARD);
#endif
    }

    return (ENXIO);
}

static int
tm1637_detach(device_t dev)
{
    struct tm1637_softc *sc = device_get_softc(dev);

    /* Turn a display off */
    tm1637_display_off(sc);

    /* Remove a device from devfs */
    if (sc->cdev != NULL)
	destroy_dev(sc->cdev);

    /* Release sda and scl pins of the device */
    if (sc->sclpin != NULL)
	gpio_pin_release(sc->sclpin);
    if (sc->sdapin != NULL)
	gpio_pin_release(sc->sdapin);

    /* Free buffers dynamic length arrays */
    free(sc->buffer.text, M_TM1637BUF);
    free(sc->buffer.codes, M_TM1637COD);

    TM1637_LOCK_DESTROY(sc);

    return (0);
}

static int
tm1637_attach(device_t dev)
{
    struct tm1637_softc *sc = device_get_softc(dev);
    int unit;
    int err;

    sc->dev = dev;
    sc->node = ofw_bus_get_node(dev);
    sc->buffer.text_length = sc->info->buffer_length;
    sc->scl_low_timeout = DEFAULT_SCL_LOW_TIMEOUT;

    /* Acquire gpio pins from hints first */
    err = tm1637_setup_hinted_pins(sc);

#ifdef FDT
    /* If no pins found from hints
     * acquire them from FDT
     */
    if (err != 0)
	err = tm1637_setup_fdt_pins(sc);
#endif

    /* If still no luck print a message */
    if (err != 0) {
	device_printf(sc->dev, "no pins configured\n");
	return (ENXIO);
    }

#ifdef FDT
    /* Get other properties from FDT */
    if ((err = tm1637_fdt_get_params(sc)) != 0)
	return (err);
#endif

    /* Say what we came up with for pin config. */
    device_printf(dev, "SCL pin: %s:%d, SDA pin: %s:%d\n",
		device_get_nameunit(GPIO_GET_BUS(sc->sclpin->dev)), sc->sclpin->pin,
		device_get_nameunit(GPIO_GET_BUS(sc->sdapin->dev)), sc->sdapin->pin );

    sc->buffer.codes = malloc(sc->info->number, M_TM1637COD, M_WAITOK | M_ZERO);
    sc->buffer.text = malloc(sc->info->buffer_length, M_TM1637BUF, M_WAITOK | M_ZERO);

    /* Create the tm1637 cdev. */
    unit = device_get_unit (dev);
    err = make_dev_p(MAKEDEV_CHECKNAME | MAKEDEV_WAITOK,
	&sc->cdev,
	&tm1637_cdevsw,
	0,
	UID_ROOT,
	GID_WHEEL,
	0600,
	"%s/%d", TM1637_CDEV_NAME, unit);

    if (err != 0) {
	device_printf(dev, "Unable to create '%s/%d' cdev\n", TM1637_CDEV_NAME, unit);
	tm1637_detach(dev);
	return (err);
    }

    sc->cdev->si_drv1 = sc;
    TM1637_LOCK_INIT(sc);

    tm1637_set_speed(sc, TM1637_BUSFREQ);
    tm1637_sysctl_register(sc);
    tm1637_clear_display(sc);
    tm1637_display_on(sc);

    return (0);
}

/* Create sysctl variables and set their handlers */
static void
tm1637_sysctl_register(struct tm1637_softc *sc)
{
    struct sysctl_ctx_list	*ctx;
    struct sysctl_oid		*tree_node;
    struct sysctl_oid_list	*tree;

    ctx = device_get_sysctl_ctx(sc->dev);
    tree_node = device_get_sysctl_tree(sc->dev);
    tree = SYSCTL_CHILDREN(tree_node);

    /* Create sysctl variables and set their handlers */
    SYSCTL_ADD_UINT(ctx, tree, OID_AUTO,
	"delay", CTLFLAG_RD, &sc->udelay,
	0, "Signal change delay controlled by bus frequency, microseconds");

    SYSCTL_ADD_U8(ctx, tree, OID_AUTO,
	"digits", CTLFLAG_RD, SYSCTL_NULL_U8_PTR,
	sc->info->number, "Number of display digits");

    SYSCTL_ADD_PROC(ctx, tree, OID_AUTO,
	"brightness", CTLTYPE_U8 | CTLFLAG_RW | CTLFLAG_MPSAFE | CTLFLAG_ANYBODY, sc, 0,
	&tm1637_brightness_sysctl, "CU", "brightness 0..7. 0 is a darkest one");

    SYSCTL_ADD_PROC(ctx, tree, OID_AUTO,
	"on", CTLTYPE_U8 | CTLFLAG_RW | CTLFLAG_MPSAFE | CTLFLAG_ANYBODY, sc, 0,
	&tm1637_set_on_sysctl, "CU", "display is on or off");
}

/* Set bus speed in Hz */
static void
tm1637_set_speed(struct tm1637_softc *sc, const u_int busfreq)
{
    const u_int period = 1000000 / 2 / busfreq;	/* Hz -> uS */
    sc->udelay = MAX(period, 1);
}
