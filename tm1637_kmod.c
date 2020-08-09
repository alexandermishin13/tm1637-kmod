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

#define FDT

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

#include <dev/gpio/gpiobusvar.h>
#include <dev/fdt/fdt_pinctrl.h>

#include "include/tm1637_kmod.h"

#define TM1637_CDEV_NAME	"tm1637"
#define TM1637_SCL_PROPERTY	"scl-gpios"
#define TM1637_SDA_PROPERTY	"sda-gpios"
#define TM1637_SCL_IDX		0
#define TM1637_SDA_IDX		1
#define TM1637_MIN_PINS		2
#define TM1637_BUSFREQ		450000

/* Based on the SMBus specification. */
#define	DEFAULT_SCL_LOW_TIMEOUT	(25 * 1000)

#define SIGN_MINUS		0x40
#define SIGN_EMPTY		0x00

#define TM1637_LOCK_INIT(sc)	\
    mtx_init(&(sc)->lock, "tm1637 mtx", NULL, MTX_DEF)
#define TM1637_LOCK_DESTROY(sc)	\
    mtx_destroy(&(sc)->lock)
#define TM1637_LOCK(sc)		\
    mtx_lock(&(sc)->lock)
#define TM1637_UNLOCK(sc)	\
    mtx_unlock(&(sc)->lock)

#define	GPIOBB_GETSDA(sc)	(gpiobb_getsda(sc))
#define	GPIOBB_SETSDA(sc, x)	(gpiobb_setsda(sc, x))
#define	GPIOBB_GETSCL(sc)	(gpiobb_getscl(sc))
#define	GPIOBB_SETSCL(sc, x)	(gpiobb_setscl(sc, x))

/*
#define GPIOBB_SET(sc, ctrl, val) do {			\
	tm1637_setscl(sc, ctrl);			\
	gpio_pin_set_active(sc->sdapin, val);	\
	DELAY(sc->udelay);				\
	} while (0)
*/

static const u_char char_code[] = { 0x3f, 0x06, 0x5b, 0x4f, 0x66, 0x6d, 0x7d, 0x07, 0x7f, 0x6f };

struct tm1637_softc {
    device_t		 dev;
    phandle_t		 node;
    gpio_pin_t		 sclpin;
    gpio_pin_t		 sdapin;
    uint8_t		 brightness;
    uint8_t		 on;
    uint8_t		 raw_mode;
    bool		 needupdate;
    bool		 inuse;
    u_char		 tm1637_digits[TM1637_MAX_COLOM];
    u_char		 tm1637_digits_prev[TM1637_MAX_COLOM];
    u_int		 udelay; /* signal toggle delay in usec */
    u_int		 scl_low_timeout;
    struct mtx		 lock;
    struct cdev		*cdev;
    struct s_message	*tm1637_buf;
    struct s_message	*tm1637_msg;
};

static int tm1637_probe(device_t);
static int tm1637_attach(device_t);
static int tm1637_detach(device_t);

static void gpiobb_setsda(struct tm1637_softc*, bool);
static void gpiobb_setscl(struct tm1637_softc*, bool);
static bool gpiobb_getsda(struct tm1637_softc*);
static bool gpiobb_getscl(struct tm1637_softc*);
static int gpiobb_getack(struct tm1637_softc*);

static void tm1637_set_speed(struct tm1637_softc*, u_int);
static void tm1637_setscl(struct tm1637_softc*, bool);
//static int tm1637_gpio_start(struct tm1637_softc*);
//static int tm1637_gpio_stop(struct tm1637_softc*);

static int tm1637_read(struct cdev*, struct uio*, int ioflag);
static int tm1637_write(struct cdev*, struct uio*, int ioflag);
static int tm1637_ioctl(struct cdev*, u_long cmd, caddr_t data, int fflag, struct thread*);
static void tm1637_display_on(struct tm1637_softc*);
static void tm1637_display_off(struct tm1637_softc*);
static void tm1637_set_brightness(struct tm1637_softc*, uint8_t);

#ifdef FDT
#include <dev/ofw/ofw_bus.h>
//#include <dev/ofw/ofw_bus_subr.h>

static struct ofw_compat_data compat_data[] = {
    {"tm1637",  true},
    {NULL,     false}
};

OFWBUS_PNP_INFO(compat_data);
SIMPLEBUS_PNP_INFO(compat_data);

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

/*
 * Sysctl parameter: raw_mode
 */
static int
tm1637_raw_mode_sysctl(SYSCTL_HANDLER_ARGS)
{
    struct tm1637_softc *sc = arg1;
    uint8_t _raw_mode = sc->raw_mode;
    int error = 0;

    error = SYSCTL_OUT(req, &_raw_mode, sizeof(_raw_mode));
    if (error != 0 || req->newptr == NULL)
	return (error);

    error = SYSCTL_IN(req, &_raw_mode, sizeof(_raw_mode));
    if (error != 0)
	return (error);

    switch(_raw_mode)
    {
	case 0:
	case 1:
	    sc->raw_mode = _raw_mode;
	    break;
	default:
	    error = EINVAL;
    }

    return (error);
}

static void
gpiobb_setsda(struct tm1637_softc *sc, bool val)
{
	//int err;

	/*
	 * Some controllers cannot set an output value while a pin is in input
	 * mode; in that case we set the pin again after changing mode.
	 */
	//err = gpio_pin_set_active(sc->sdapin, val);
	gpio_pin_setflags(sc->sdapin, GPIO_PIN_OUTPUT | GPIO_PIN_OPENDRAIN);
	//if (err != 0)
		gpio_pin_set_active(sc->sdapin, val);
}

static void
gpiobb_setscl(struct tm1637_softc *sc, bool val)
{
	gpio_pin_setflags(sc->sclpin, GPIO_PIN_OUTPUT | GPIO_PIN_OPENDRAIN);
	gpio_pin_set_active(sc->sclpin, val);
}

static bool
gpiobb_getscl(struct tm1637_softc *sc)
{
	bool val;

	gpio_pin_setflags(sc->sclpin, GPIO_PIN_INPUT);
	gpio_pin_is_active(sc->sclpin, &val);
	return (val);
}

static bool
gpiobb_getsda(struct tm1637_softc *sc)
{
	bool val;

	gpio_pin_setflags(sc->sdapin, GPIO_PIN_INPUT);
	gpio_pin_is_active(sc->sdapin, &val);
	return (val);
}

static int
gpiobb_waitforscl(struct tm1637_softc *sc)
{
	sbintime_t fast_timeout;
	sbintime_t now, timeout;

	/* Spin for up to 1 ms, then switch to pause. */
	now = sbinuptime();
	fast_timeout = now + SBT_1MS;
	timeout = now + sc->scl_low_timeout * SBT_1US;
	do {
		if (GPIOBB_GETSCL(sc))
			return (0);
		now = sbinuptime();
	} while (now < fast_timeout);
	do {
		pause_sbt("gpiobb-scl-low", SBT_1MS, C_PREL(8), 0);
		if (GPIOBB_GETSCL(sc))
			return (0);
		now = sbinuptime();
	} while (now < timeout);

	return (EIO);
}

/* Start the high phase of the clock. */
static int
gpiobb_clockin(struct tm1637_softc *sc, bool sda)
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
	GPIOBB_SETSDA(sc, sda);
	GPIOBB_SETSCL(sc, true);
	return (gpiobb_waitforscl(sc));
}

/*
 * End the high phase of the clock and wait out the low phase
 * as nothing interesting happens during it anyway.
 */
static void
gpiobb_clockout(struct tm1637_softc *sc)
{
	/*
	 * Precondition: SCL is high.
	 * Action:
	 * - pull SCL low and hold for udelay.
	 */
	GPIOBB_SETSCL(sc, false);
	DELAY(sc->udelay);
}

static int
gpiobb_sendbit(struct tm1637_softc *sc, bool bit)
{
	int err;

	err = gpiobb_clockin(sc, bit);
	if (err != 0)
		return (err);
	DELAY(sc->udelay);
	gpiobb_clockout(sc);
	return (0);
}

/*
 * Start command or data transmission
 */
static int
gpiobb_start(struct tm1637_softc *sc)
{
//	int error;

	/* SCL must be high on the idle bus. */
	if (gpiobb_waitforscl(sc) != 0)
		return (EIO);

	/*
	 * SDA must be high after the earlier stop condition or the end
	 * of Ack/NoAck pulse.
	 */
	if (!GPIOBB_GETSDA(sc))
		return (EIO);

	/* Start: SDA high->low. */
	GPIOBB_SETSDA(sc, 0);

	/* Wait the second half of the SCL high phase. */
	DELAY((sc->udelay + 1) / 2);

	/* Pull SCL low to keep the bus reserved. */
	gpiobb_clockout(sc);

	/* send address */
//	error = iicbb_sendbyte(dev, slave);

	/* check for ack */
//	if (error == 0)
//		error = iicbb_getack(dev);
//	if (error != 0)
//		(void)iicbb_stop(dev);
//	return (error);
	return (0);
}

/*
 * Stop command or data transmission
 */
static int
gpiobb_stop(struct tm1637_softc *sc)
{
	int err = 0;

	/*
	 * Stop: SDA goes from low to high in the middle of the SCL high phase.
	 */
	err = gpiobb_clockin(sc, false);
	if (err != 0)
		return (err);
	DELAY((sc->udelay + 1) / 2);
	GPIOBB_SETSDA(sc, true);
	DELAY((sc->udelay + 1) / 2);
	return (err);
}

static void
tm1637_setscl(struct tm1637_softc *sc, bool val)
{
    sbintime_t now, end;
    int fast_timeout;
    bool scl_val;

    gpio_pin_set_active(sc->sclpin, val);
    DELAY(sc->udelay);

    /* Pulling low cannot fail. */
    if (!val)
	return;

    /* Use DELAY for up to 1 ms, then switch to pause. */
    end = sbinuptime() + sc->scl_low_timeout * SBT_1US;
    fast_timeout = MIN(sc->scl_low_timeout, 1000);
    while (fast_timeout > 0) {
	gpio_pin_is_active(sc->sdapin, &scl_val);
	if (scl_val)
	    return;
	gpio_pin_set_active(sc->sclpin, true);	/* redundant ? */
	DELAY(sc->udelay);
	fast_timeout -= sc->udelay;
    }

    gpio_pin_is_active(sc->sclpin, &scl_val);
    while (!scl_val) {
	now = sbinuptime();
	if (now >= end)
	    break;
	pause_sbt("tm1637-scl-low", SBT_1MS, C_PREL(8), 0);
    }
}

/*
static int
tm1637_gpio_start(struct tm1637_softc *sc)
{
    bool scl_val;

    GPIOBB_SET(sc, true, true);
*/

    /* SCL must be high now. */
/*
    gpio_pin_setflags(sc->sclpin, GPIO_PIN_INPUT);
    gpio_pin_is_active(sc->sclpin, &scl_val);
    gpio_pin_setflags(sc->sclpin, GPIO_PIN_OUTPUT);
    if (!scl_val)
	return (EIO);

    GPIOBB_SET(sc, true, false);
    GPIOBB_SET(sc, false, false);

    return (0);
}
*/

/*
 * Send a byte of information w/ ack checking
 */
static int
tm1637_gpio_sendbyte(struct tm1637_softc *sc, u_char data)
{
    int err, i;

    // LSB first
    for(i=0; i<=7; i++)
    {
	err = gpiobb_sendbit(sc, (data & (1 << i)) != 0);
	if (err != 0)
	    return (err);
    }

    if (gpiobb_getack(sc))
	return EIO;

    return 0;
}

static int
gpiobb_getack(struct tm1637_softc *sc)
{
    int noack, err;
    int t;

    /* Release SDA so that the slave can drive it. */
    err = gpiobb_clockin(sc, true);
    if (err != 0) {
	return (err);
    }

    /* Sample SDA until ACK (low) or udelay runs out. */
    for (t = 0; t < sc->udelay; t++) {
	noack = GPIOBB_GETSDA(sc);
	if (!noack)
	    break;
	DELAY(1);
    }

    DELAY(sc->udelay - t);
    gpiobb_clockout(sc);

    return (noack ? EIO : 0);
}

/*
 * Sends one byte command with a retry if no acnowledge occurs
 * Returns zero on success
 */
static int
tm1637_send_command(struct tm1637_softc *sc, u_char cmd)
{
    int err;

    gpiobb_start(sc);
    //tm1637_gpio_start(sc);
    err = tm1637_gpio_sendbyte(sc, cmd);
    gpiobb_stop(sc);

    if (err)
    {
	device_printf(sc->dev, "No ack when sent command 0x%02x, resending\n", cmd);

	gpiobb_start(sc);
	//tm1637_gpio_start(sc);
	err = tm1637_gpio_sendbyte(sc, cmd);
	gpiobb_stop(sc);

	if (err)
	    device_printf(sc->dev, "No ack when resent command 0x%02x, canceled\n", cmd);
    }

    return err;
}

/*
 * Sends an address and one byte data to it with a retry if no acknowleges occurs
 * Returns zero on success
 */
static int
tm1637_send_data1(struct tm1637_softc *sc, size_t pos)
{
    int err;
    u_char addr = TM1637_START_ADDRESS + pos;
    u_char data = sc->tm1637_digits[pos];

    gpiobb_start(sc);
    //tm1637_gpio_start(sc);
    // Send address
    err = tm1637_gpio_sendbyte(sc, addr);
    if (err)
    {
	// A stop bit needed before a retry
//	gpiobb_stop(sc);

	device_printf(sc->dev, "No ack when sent address 0x%02x, resending\n", addr);

//	tm1637_gpio_start(sc);
	err = tm1637_gpio_sendbyte(sc, addr);
	if (err)
	{
	    // Give up this time and a stop bit
	    gpiobb_stop(sc);

	    device_printf(sc->dev, "No ack when resent address 0x%02x, canceled\n", addr);

	    return err;
	}
    }

    // Send data to the address
    err = tm1637_gpio_sendbyte(sc, data);
    if (err)
    {
	// Resend data to the same address with no stop bit
	err = tm1637_gpio_sendbyte(sc, data);
	gpiobb_stop(sc);
	device_printf(sc->dev, "No ack when sent data 0x%02x twice, canceled\n", data);

	return err;
    }
    gpiobb_stop(sc);

#ifdef DEBUG
    uprintf("display: ");
    uprintf("%i[    %02x]\n",
            TM1637_COLON_POSITION - 1,
            sc->tm1637_digits[TM1637_COLON_POSITION - 1]);
#endif

    return 0;
}

/*
 * Send number of bytes fron first to last address
 * Changed pos parameter can be used for sending rest of data on error
 * 
 */
static int
tm1637_send_data(struct tm1637_softc *sc, size_t *pos, size_t last)
{
    int err;
    u_char addr = TM1637_START_ADDRESS + *pos;

    gpiobb_start(sc);
    //tm1637_gpio_start(sc);
    // Send address
    err = tm1637_gpio_sendbyte(sc, addr);
    if (err)
    {
	// A stop bit needed before a retry
	gpiobb_stop(sc);

	device_printf(sc->dev, "No ack when sent address 0x%02x, resending\n", addr);

	gpiobb_start(sc);
	//tm1637_gpio_start(sc);
	err = tm1637_gpio_sendbyte(sc, addr);
	if (err)
	{
	    // Give up this time and a stop bit
	    gpiobb_stop(sc);

	    device_printf(sc->dev, "No ack when resent address 0x%02x, canceled\n", addr);

	    return err;
	}
    }

#ifdef DEBUG
    uprintf("display: ");
#endif

    // Send change digits
    while(*pos<=last)
    {

#ifdef DEBUG
	uprintf("%i[    %02x]", *pos, sc->tm1637_digits[*pos]);
#endif
	// Send colom segments
	err = tm1637_gpio_sendbyte(sc, sc->tm1637_digits[*pos]);
	// Early exit, keep 'pos' for unsuccessful colom accessible from outside
	if (err)
	    break;

	++*pos;
    }
    gpiobb_stop(sc);

#ifdef DEBUG
    uprintf("\n");
#endif

    return err;
}

/*
 * Restore previous row of digits
 */
static void
tm1637_restore_digits(struct tm1637_softc *sc)
{
    size_t position;

#ifdef DEBUG
    uprintf("restore: ");
#endif
    for(position=0; position<TM1637_MAX_COLOM; position++)
    {
	sc->tm1637_digits[position] = sc->tm1637_digits_prev[position];
#ifdef DEBUG
	uprintf("%i[%02x->%02x]", position, sc->tm1637_digits[position], sc->tm1637_digits_prev[position]);
#endif
    }
#ifdef DEBUG
    uprintf("\n");
#endif
}

/*
 * Display part of the full row of digits
 */
static void
tm1637_display_digits(struct tm1637_softc *sc, size_t first, size_t last)
{
    int err;

    // Prepare to an autoincremented address data transfer
    err = tm1637_send_command(sc, TM1637_ADDRESS_AUTO);
    if (err)
	sc->needupdate = true;
    else
    {
	// If no error send the data
	err = tm1637_send_data(sc, &first, last);
	if (err)
	{
	    // Resend rest of data ("first" points to an unsuccessful colom)
	    err = tm1637_send_data(sc, &first, last);
	    if (err)
		sc->needupdate = true;
	}
    }
}

/*
 * Display or clear a clockpoint
 */
static void
tm1637_display_clockpoint(struct tm1637_softc *sc, bool clockpoint)
{
    int err;

    if (clockpoint)
	sc->tm1637_digits[TM1637_COLON_POSITION - 1] |= 0x80;
    else
	sc->tm1637_digits[TM1637_COLON_POSITION - 1] &= 0x7f;

    // Prepare to an one byte data transfer
    err = tm1637_send_command(sc, TM1637_ADDRESS_FIXED);
    if (err)
	sc->needupdate = true;
    else
    {
	// If no error send the on byte data
	err = tm1637_send_data1(sc, TM1637_COLON_POSITION - 1);
	if (err)
	    sc->needupdate = true;
    }
}

/*
 * Send to display all 4 digits (6 bytes will be sended)
 */
static void
tm1637_update_display(struct tm1637_softc *sc)
{
    size_t first = TM1637_MAX_COLOM, position, last = 0;

    if(sc->needupdate)
    {
	// If display is on update it all and clear the flag
	if(sc->on)
	{
	    first = 0;
	    last = TM1637_MAX_COLOM - 1;
#ifdef DEBUG
	    uprintf("Needs to be updated all\n");
#endif
	    tm1637_display_digits(sc, first, last);
	    sc->needupdate = false;
	}
    }
    else
    {
	// Finding a range of changed digits
#ifdef DEBUG
	uprintf("changed: ");
#endif
	for(position=0; position<TM1637_MAX_COLOM; position++)
	{
	    if(sc->tm1637_digits_prev[position] != sc->tm1637_digits[position])
	    {
#ifdef DEBUG
		uprintf("%i[%02x->%02x]", position, sc->tm1637_digits_prev[position], sc->tm1637_digits[position]);
#endif
		sc->tm1637_digits_prev[position] = sc->tm1637_digits[position];

		// Last changed digit
		last = position;
		// First changed digit
		if (first == TM1637_MAX_COLOM)
		    first = position;
	    }
	}

#ifdef DEBUG
	uprintf("\n");
#endif

	// Display an optimized part of row of digits
	// or mark it for update if it is off
	if(first < TM1637_MAX_COLOM)
	{
	    if(sc->on)
		tm1637_display_digits(sc, first, last);
	    else
		sc->needupdate = true;
	}
    }
}

/*
 * Writes all blanks to a display
 */
static void
tm1637_clear_display(struct tm1637_softc *sc)
{
    size_t position = TM1637_MAX_COLOM;

    // Display all blanks
    while(position--)
    {
	sc->tm1637_digits[position] = SIGN_EMPTY;
	sc->tm1637_digits_prev[position] = SIGN_EMPTY;
    }

    // If display is off right now then mark it for update when it is on
    if(sc->on)
	tm1637_display_digits(sc, 0, TM1637_MAX_COLOM - 1);
    else
	sc->needupdate = true;
}

/*
 * Sets a display on with a brightness value from softc
 */
static void
tm1637_display_on(struct tm1637_softc *sc)
{
    int err;

    // Do nothing is always on
    if(sc->on == 0)
    {
	if(sc->needupdate)
	{
	    tm1637_display_digits(sc, 0, TM1637_MAX_COLOM - 1);
	    sc->needupdate = false;
	}
	sc->on = 1;

	err = tm1637_send_command(sc, 0x88|sc->brightness);

#ifdef DEBUG
	uprintf("Display turned on\n");
#endif

    }
}

/*
 * Sets a display on with a brightness value as parameter
 */
static void
tm1637_set_brightness(struct tm1637_softc *sc, uint8_t brightness)
{
    int err;

#ifdef DEBUG
    uprintf("Brightness level ");
#endif

    // If brightness is really changed
    if ((brightness != sc->brightness) &&
        (brightness <= TM1637_BRIGHTEST))
    {
	sc->brightness = brightness;
	// Only change variable if a display is not on
	if(sc->on != 0)
	{
	    err = tm1637_send_command(sc, 0x88|sc->brightness);

#ifdef DEBUG
	    uprintf("is %d now\n", sc->brightness);
#endif

	}

#ifdef DEBUG
	else
	    uprintf("setting to %d is delayed until the display is on\n", sc->brightness);
#endif

    }

#ifdef DEBUG
    else
	uprintf("setting is ignored\n");
#endif

}

/*
 * Set a display off
 */
static void
tm1637_display_off(struct tm1637_softc *sc)
{
    int err;

    // Do nothing is always off
    if(sc->on != 0)
    {
	sc->on = 0;
	err = tm1637_send_command(sc, 0x80);

#ifdef DEBUG
	uprintf("Display turned off\n");
#endif

    }
}

/*
 * Sets time to the display
 */
static void
tm1637_set_clock(struct tm1637_softc *sc, struct tm1637_clock_t clock)
{
    int t;

    // Four clock digits
    t = clock.tm_hour / 10;
    sc->tm1637_digits[0] = char_code[t&0x0f];
    t = clock.tm_hour % 10;
    sc->tm1637_digits[1] = char_code[t];
    t = clock.tm_min / 10;
    sc->tm1637_digits[2] = char_code[t&0x0f];
    t = clock.tm_min % 10;
    sc->tm1637_digits[3] = char_code[t];

    // Clockpoint
    if (clock.tm_colon)
	sc->tm1637_digits[TM1637_COLON_POSITION - 1] |= 0x80;

    tm1637_update_display(sc);
}

static int
tm1637_process_segs(struct tm1637_softc *sc)
{
    int err = 0;

    if(sc->tm1637_buf->len > TM1637_MAX_COLOM)
        return EINVAL;

#ifdef DEBUG
    uprintf("processed: [");
#endif

    // Process only chars from starting offset to len-1
    int position = sc->tm1637_buf->offset;
    while(position < sc->tm1637_buf->len)
    {
	u_char c = sc->tm1637_buf->text[position];
	sc->tm1637_digits[position] = c;
	position++;

#ifdef DEBUG
	uprintf("%02x", c);
#endif

    }

#ifdef DEBUG
    uprintf("]\n");
    uprintf("  offset: %d\n", sc->tm1637_buf->offset);
    uprintf("  length: %d\n", sc->tm1637_buf->len);
#endif

    return err;
}

static int
tm1637_write_segs(struct tm1637_softc *sc, struct uio *uio)
{
    int error;
    size_t amount;

    // Check a file current position which an user set by seek()
    if (uio->uio_offset >= 0)
    {
	if(uio->uio_offset >= TM1637_MAX_COLOM)
	    return EINVAL;
    }
    else
    {
	if(uio->uio_offset >= -TM1637_MAX_COLOM)
	    uio->uio_offset = TM1637_MAX_COLOM + uio->uio_offset;
	else
	    return EINVAL;
    }

/*
    if (uio->uio_offset < 0 && uio->uio_offset >= TM1637_MAX_COLOM)
	return (error);
*/

    // Copy the string in from user memory to kernel memory
    amount = MIN(uio->uio_resid, (TM1637_BUFFERSIZE - uio->uio_offset));

#ifdef DEBUG
    uprintf("dev_write\n");
    uprintf("  uio_offset: %lld\n", uio->uio_offset);
#endif

    // Set a first char position
    sc->tm1637_buf->offset = uio->uio_offset;

    error = uiomove(sc->tm1637_buf->text + uio->uio_offset, amount, uio);

    // Set the length
    sc->tm1637_buf->len = uio->uio_offset;

#ifdef DEBUG
    uprintf("  tm1637_buf->text: [");
    for(size_t i = 0; i < sc->tm1637_buf->len; i++)
    {
	if (sc->tm1637_buf->text[i] == '\n')
	    uprintf("\\n");
	else
	    uprintf("%c", sc->tm1637_buf->text[i]);
    }
    uprintf("]\n");
    uprintf("  tm1637_buf->offset: %d\n", sc->tm1637_buf->offset);
    uprintf("  tm1637_buf->len: %d\n", sc->tm1637_buf->len);
#endif

    return (error);
}

static int
tm1637_process_chars(struct tm1637_softc *sc)
{
    size_t buf_index = 0, position = 0;
    //int err = EJUSTRETURN;
    int err = 0;

#ifdef DEBUG
    uprintf("processed: [");
#endif

    // Number of digits and colon if any
    while(buf_index <= sc->tm1637_buf->len)
    {
	u_char c = sc->tm1637_buf->text[buf_index];

#ifdef DEBUG
	if (c == '\n')
	    uprintf("\\n");
	else
	    uprintf("%c", c);
#endif

	if(c>='0' && c<='9') // encode if digit
	    sc->tm1637_digits[position++] = char_code[c&0x0f];
	else if(c=='-') // '-' sign
	    sc->tm1637_digits[position++] = SIGN_MINUS;
	else if(c=='#') // No changes if wildcard
	    position++;
	else if(c=='\n') // End of a chunk, checking its format
	{
	    // 'buf_index' contains a length of the message
	    if (buf_index == TM1637_MAX_COLOM + 1)
	    {
		u_char clockpoint = sc->tm1637_buf->text[TM1637_COLON_POSITION];
		if (clockpoint != ':' && clockpoint != ' ')
		    err = EINVAL;
	    }
	    else if (buf_index <= TM1637_MAX_COLOM)
	    {
		if ((buf_index > TM1637_COLON_POSITION) && (sc->tm1637_buf->text[TM1637_COLON_POSITION] == ':'))
		    err = EINVAL;
		else
		{
		    // Add trailing spaces starting from last position
		    for (;position <= TM1637_MAX_COLOM; position++)
			sc->tm1637_digits[position] = 0x00;
		}
	    }
	    else
		err = EINVAL;

	    break; // Stop the processing
	}
	else if(buf_index==TM1637_COLON_POSITION)
	{
	    if (c == ':')
		sc->tm1637_digits[TM1637_COLON_POSITION - 1] |= 0x80;
	    else if (c == ' ')
		sc->tm1637_digits[TM1637_COLON_POSITION - 1] &= 0x7f;
	    else
	    {
		err = EINVAL; // 2-nd character must be a digit or a clockpoint
		break;
	    }
	}
	else if(c==' ') // space !!!after if(buf_index==TM1637_COLON_POSITION)
	    sc->tm1637_digits[position++] = SIGN_EMPTY;
	else
	{
	    err = EINVAL;
	    break;
	}

	buf_index++;
    }

#ifdef DEBUG
    uprintf("]\n");
    uprintf("  length: %d\n", buf_index);
#endif

    return err;
}

static int
tm1637_write_chars(struct tm1637_softc *sc, struct uio *uio)
{
    int error;
    size_t amount;
    off_t uio_offset_saved;

#ifdef DEBUG
    uprintf("dev_write\n");
    uprintf("  uio_offset: %lld\n", uio->uio_offset);
#endif

    amount = MIN(uio->uio_resid, TM1637_BUFFERSIZE);
    uio_offset_saved = uio->uio_offset;
    error = uiomove(sc->tm1637_buf->text, amount, uio);
    uio->uio_offset = uio_offset_saved;

#ifdef DEBUG
    uprintf("  tm1637_buf->text: [");
    for(size_t i = 0; i < amount; i++)
    {
	if (sc->tm1637_buf->text[i] == '\n')
	    uprintf("\\n");
	else
	    uprintf("%c", sc->tm1637_buf->text[i]);
    }
    uprintf("]\n");
    uprintf("  tm1637_buf->len: %d\n", amount);
#endif

    if (error == 0)
	sc->tm1637_buf->len = amount;

    return (error);
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
static d_read_t      tm1637_read;
static d_write_t     tm1637_write;
static d_ioctl_t     tm1637_ioctl;

/* Character device entry points */
static struct cdevsw tm1637_cdevsw = {
    .d_version = D_VERSION,
    .d_open = tm1637_open,
    .d_close = tm1637_close,
    .d_read = tm1637_read,
    .d_write = tm1637_write,
    .d_ioctl = tm1637_ioctl,
    .d_name = TM1637_CDEV_NAME,
};

static int
tm1637_ioctl(struct cdev *cdev, u_long cmd, caddr_t data, int fflag, struct thread *td)
{
    int error = 0;

    struct tm1637_softc *sc = cdev->si_drv1; // Stored here on tm1637_attach()

    switch (cmd)
    {
	case TM1637IOC_CLEAR:
#ifdef DEBUG
	    uprintf("ioctl(display_clear)\n");
#endif
	    tm1637_clear_display(sc);
	    break;
	case TM1637IOC_OFF:
#ifdef DEBUG
	    uprintf("ioctl(display_off)\n");
#endif
	    tm1637_display_off(sc);
	    break;
	case TM1637IOC_ON:
#ifdef DEBUG
	    uprintf("ioctl(display_on)\n");
#endif
	    tm1637_display_on(sc);
	    break;
	case TM1637IOC_SET_BRIGHTNESS:
#ifdef DEBUG
	    uprintf("ioctl(set_brightness, %i)\n", *(uint8_t*)data);
#endif
	    tm1637_set_brightness(sc, *(uint8_t*)data);
	    break;
	case TM1637IOC_SET_CLOCKPOINT:
	    if(sc->on != 0)
	    {
#ifdef DEBUG
		uprintf("ioctl(set_clockpoint, %i)\n", *(uint8_t*)data);
#endif
		tm1637_display_clockpoint(sc, *(bool*)data);
	    }
	    break;
	case TM1637IOC_SET_RAWMODE:
	    if (*(uint8_t*)data <= 1)
	    {
#ifdef DEBUG
		uprintf("ioctl(set_rawmode, %i)\n", *(uint8_t*)data);
#endif
		sc->raw_mode = *(uint8_t*)data;
	    }
	    break;
	case TM1637IOC_GET_RAWMODE:
#ifdef DEBUG
	    uprintf("ioctl(get_rawmode) -> %i\n", sc->raw_mode);
#endif
	    *(uint8_t *)data = sc->raw_mode;
	    break;
	case TM1637IOC_SET_CLOCK:
#ifdef DEBUG
		uprintf("ioctl(set_clock, %02i%c%02i)\n",
			*(struct tm1637_clock_t*)data->tm_hour,
			*(struct tm1637_clock_t*)data->tm_colon)?':':' ',
			*(struct tm1637_clock_t*)data->tm_min);
#endif
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
    struct tm1637_softc *sc = cdev->si_drv1; // Stored here on tm1637_attach()

    TM1637_LOCK(sc);
    if (sc->inuse) {
        TM1637_UNLOCK(sc);
        return (EBUSY);
    }
    /* move to init */
    sc->inuse = true;
    TM1637_UNLOCK(sc);

#ifdef DEBUG
    uprintf("Opened device \"%s\" successfully.\n", cdevsw.d_name);
#endif

    return (0);
}

static int
tm1637_close(struct cdev *cdev __unused, int fflag __unused, int devtype __unused,
    struct thread *td __unused)
{
    struct tm1637_softc *sc = cdev->si_drv1; // Stored here on tm1637_attach()

    sc->inuse = false;

#ifdef DEBUG
    uprintf("Closing device \"%s\".\n", tm1637_cdevsw.d_name);
#endif

    return (0);
}

static int
tm1637_read(struct cdev *cdev, struct uio *uio, int ioflag __unused)
{
    int error;
    size_t amount;
    struct tm1637_softc *sc = cdev->si_drv1; // Stored here on tm1637_attach()

    size_t text_len = sc->tm1637_buf->len + 1;
    amount = MIN(uio->uio_resid, 
		 uio->uio_offset >= text_len ? 0 : text_len - uio->uio_offset);

    if ((error = uiomove(sc->tm1637_buf->text, amount, uio)) != 0)
	uprintf("uiomove failed!\n");

    return (error);
}

static int
tm1637_write(struct cdev *cdev, struct uio *uio, int ioflag __unused)
{
    int error;

    struct tm1637_softc *sc = cdev->si_drv1;

    if(sc->raw_mode > 0)
    {
	error = tm1637_write_segs(sc, uio);
	if (error == 0)
	{
	    error = tm1637_process_segs(sc);
	    if (error == 0)
		tm1637_update_display(sc); // No any restore needed
	}
    }
    else
    {
	error = tm1637_write_chars(sc, uio);
	if (error == 0)
	{
	    // Decode sc->tm1637_buf
	    error = tm1637_process_chars(sc);
	    if (error == 0)
		tm1637_update_display(sc);
	    else
		tm1637_restore_digits(sc);
	}
    }

    if (error != 0)
	uprintf("Write failed: bad address!\n");

    return (error);
}

static int
tm1637_probe(device_t dev)
{
    int rv = BUS_PROBE_NOWILDCARD;

#ifdef FDT
    if (ofw_bus_status_okay(dev) &&
	ofw_bus_search_compatible(dev, compat_data)->ocd_data)
	    rv = BUS_PROBE_DEFAULT;
#endif

    device_set_desc(dev, "TM1637 4 Digit 7 Segment Display");

    return (rv);
}

static int
tm1637_detach(device_t dev)
{
    struct tm1637_softc *sc = device_get_softc(dev);

    tm1637_display_off(sc);

    if (sc->cdev != NULL)
	destroy_dev(sc->cdev);

    if (sc->sclpin != NULL)
	gpio_pin_release(sc->sclpin);

    if (sc->sdapin != NULL)
	gpio_pin_release(sc->sdapin);

    free(sc->tm1637_buf, M_TM1637BUF);
    TM1637_LOCK_DESTROY(sc);

    return (0);
}

static int
tm1637_attach(device_t dev)
{
    struct tm1637_softc		*sc;
    struct sysctl_ctx_list	*ctx;
    struct sysctl_oid		*tree;
    int err;

    sc = device_get_softc(dev);
    ctx = device_get_sysctl_ctx(dev);
    tree = device_get_sysctl_tree(dev);

    sc->dev = dev;
    sc->node = ofw_bus_get_node(dev);

    /* Acquire our gpio pins. */
    err = tm1637_setup_hinted_pins(sc);

#ifdef FDT
    if (err != 0)
	err = tm1637_setup_fdt_pins(sc);
#endif

    if (err != 0) {
	device_printf(sc->dev, "no pins configured\n");
	return (ENXIO);
    }

    /* Say what we came up with for pin config. */
    device_printf(dev, "SCL pin: %s:%d, SDA pin: %s:%d\n",
	device_get_nameunit(GPIO_GET_BUS(sc->sclpin->dev)), sc->sclpin->pin,
	device_get_nameunit(GPIO_GET_BUS(sc->sdapin->dev)), sc->sdapin->pin);

    /* Create the tm1637 cdev. */
    err = make_dev_p(MAKEDEV_CHECKNAME | MAKEDEV_WAITOK,
	&sc->cdev,
	&tm1637_cdevsw,
	0,
	UID_ROOT,
	GID_WHEEL,
	0600,
	TM1637_CDEV_NAME);

    if (err != 0) {
	device_printf(dev, "Unable to create tm1637 cdev\n");
	tm1637_detach(dev);
	return (err);
    }

    sc->cdev->si_drv1 = sc;
    tm1637_set_speed(sc, TM1637_BUSFREQ);

    /* Set properties */
    uint32_t brightness;
    if (OF_getencprop(sc->node, "default-brightness-level", &brightness, sizeof(uint32_t)) == sizeof(uint32_t))
    {
	if (brightness <= 7)
	    sc->brightness = brightness;
	else
	    sc->brightness = TM1637_BRIGHT_DARKEST;
    }

    if (OF_hasprop(sc->node, "raw-mode") == 1)
	sc->raw_mode = 1;
    else
	sc->raw_mode = 0;

    /* Create sysctl variables and set their handlers */
    SYSCTL_ADD_UINT(ctx, SYSCTL_CHILDREN(tree), OID_AUTO,
	"delay", CTLFLAG_RD, &sc->udelay,
	0, "Signal change delay controlled by bus frequency, microseconds");

    SYSCTL_ADD_PROC(ctx, SYSCTL_CHILDREN(tree), OID_AUTO,
	"brightness", CTLTYPE_U8 | CTLFLAG_RW | CTLFLAG_MPSAFE | CTLFLAG_ANYBODY, sc, 0,
	&tm1637_brightness_sysctl, "CU", "brightness 0..7. 0 is a darkest one");

    SYSCTL_ADD_PROC(ctx, SYSCTL_CHILDREN(tree), OID_AUTO,
	"on", CTLTYPE_U8 | CTLFLAG_RW | CTLFLAG_MPSAFE | CTLFLAG_ANYBODY, sc, 0,
	&tm1637_set_on_sysctl, "CU", "display is on or off");

    SYSCTL_ADD_PROC(ctx, SYSCTL_CHILDREN(tree), OID_AUTO,
	"raw_mode", CTLTYPE_U8 | CTLFLAG_RW | CTLFLAG_MPSAFE, sc, 0,
	&tm1637_raw_mode_sysctl, "CU", "Raw mode of input: 0 for human readable, 1 for bytes of segments");

    TM1637_LOCK_INIT(sc);
    sc->tm1637_buf = malloc(sizeof(*sc->tm1637_buf), M_TM1637BUF, M_WAITOK | M_ZERO);

    tm1637_clear_display(sc);
    tm1637_display_on(sc);

    return (0);
}

/* Set bus speed in Hz */
static void
tm1637_set_speed(struct tm1637_softc *sc, u_int busfreq)
{
    u_int period = 1000000 / 2 / busfreq;	/* Hz -> uS */
    sc->udelay = MAX(period, 1);
}
