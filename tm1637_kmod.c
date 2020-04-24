/*-
 * Copyright (C) 2020 Alexander Mishin.
 * All rights reserved.
 *
 * THIS SOFTWARE IS PROVIDED BY AUTHOR AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL AUTHOR OR CONTRIBUTORS BE LIABLE
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

#include "tm1637_kmod.h"

#define SIGN_MINUS		0x40
#define SIGN_EMPTY		0x00

static u_char char_code[] = { 0x3f, 0x06, 0x5b, 0x4f, 0x66, 0x6d, 0x7d, 0x07, 0x7f, 0x6f };

static void tm1637_display_on(struct tm1637_softc *sc);
static void tm1637_display_off(struct tm1637_softc *sc);

#ifdef FDT
#include <dev/ofw/ofw_bus.h>
//#include <dev/ofw/ofw_bus_subr.h>

static struct ofw_compat_data compat_data[] = {
    {"tm1637",  true},
    {NULL,     false}
};

//OFWBUS_PNP_INFO(compat_data);
SIMPLEBUS_PNP_INFO(compat_data);

static device_method_t tm1637_methods[] = {
    /* Device interface */
    DEVMETHOD(device_probe,	tm1637_probe),
    DEVMETHOD(device_attach,	tm1637_attach),
    DEVMETHOD(device_detach,	tm1637_detach),

    DEVMETHOD_END
};

static driver_t tm1637_driver = {
    "tm1637",
    tm1637_methods,
    sizeof(struct tm1637_softc)
};

static devclass_t tm1637_devclass;

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
    if (OF_hasprop(sc->tm1637_node, "gpios")) {
	if ((err = gpio_pin_get_by_ofw_idx(sc->tm1637_dev, sc->tm1637_node, TM1637_SCL_IDX, &sc->tm1637_sclpin)) != 0)
	{
	    device_printf(sc->tm1637_dev, "invalid gpios property for index:%d\n", TM1637_SCL_IDX);
	    return (err);
	}
	if ((err = gpio_pin_get_by_ofw_idx(sc->tm1637_dev, sc->tm1637_node, TM1637_SDA_IDX, &sc->tm1637_sdapin)) != 0)
	{
	    device_printf(sc->tm1637_dev, "invalid gpios property for index:%d\n", TM1637_SDA_IDX);
	    return (err);
	}
    } else {
	if ((err = gpio_pin_get_by_ofw_property(sc->tm1637_dev, sc->tm1637_node, TM1637_SCL_PROPERTY, &sc->tm1637_sclpin)) != 0)
	{
	    device_printf(sc->tm1637_dev, "missing %s property\n", TM1637_SCL_PROPERTY);
	    return (err);
	}
	if ((err = gpio_pin_get_by_ofw_property(sc->tm1637_dev, sc->tm1637_node, TM1637_SDA_PROPERTY, &sc->tm1637_sdapin)) != 0)
	{
	    device_printf(sc->tm1637_dev, "missing %s property\n", TM1637_SDA_PROPERTY);
	    return (err);
	}
    }

    /* Get pin configuration from pinctrl-0 and ignore errors */
    err = fdt_pinctrl_configure(sc->tm1637_dev, 0);
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
    uint8_t brightness = sc->tm1637_brightness;
    int error;

    error = SYSCTL_OUT(req, &brightness, sizeof(brightness));
    if (error != 0 || req->newptr == NULL)
	return (error);

    error = SYSCTL_IN(req, &brightness, sizeof(brightness));
    if (error != 0)
	return (error);

    if (brightness > 7)
	return (EINVAL);

    sc->tm1637_brightness = brightness;
    tm1637_display_on(sc);

    return (0);
}

/*
 * Sysctl parameter: on
 */
static int
tm1637_set_on_sysctl(SYSCTL_HANDLER_ARGS)
{
    struct tm1637_softc *sc = arg1;
    uint8_t _on = sc->tm1637_on;
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
    uint8_t _raw_mode = sc->tm1637_raw_mode;
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
	    sc->tm1637_raw_mode = _raw_mode;
	    break;
	default:
	    error = EINVAL;
    }

    return (error);
}

/*
 * Start command or data transmission
 */
static void
tm1637_gpio_start(struct tm1637_softc *sc)
{
    gpio_pin_setflags(sc->tm1637_sclpin, GPIO_PIN_OUTPUT);
    gpio_pin_setflags(sc->tm1637_sdapin, GPIO_PIN_OUTPUT);

    gpio_pin_set_active(sc->tm1637_sclpin, true);
    gpio_pin_set_active(sc->tm1637_sdapin, true);
    DELAY(2);
    gpio_pin_set_active(sc->tm1637_sdapin, false);
}

/*
 * Send a byte of information w/ ack checking
 */
static void
tm1637_gpio_sendbyte(struct tm1637_softc *sc, u_char data)
{
    int i;
    int k = 0;
    bool noack;

    for(i=0; i<=7; i++)
    {
	gpio_pin_set_active(sc->tm1637_sclpin, false);
	// Set the data bit, CLK is low after start
	gpio_pin_set_active(sc->tm1637_sdapin, (bool)(data&(1<<i))); //LSB first
	// The data bit is ready
	gpio_pin_set_active(sc->tm1637_sclpin, true);
    }
    gpio_pin_set_active(sc->tm1637_sclpin, false);
    DELAY(5);
    // Waiting a zero data bit as an ACK
    gpio_pin_setflags(sc->tm1637_sdapin, GPIO_PIN_INPUT);

    do {
	gpio_pin_is_active(sc->tm1637_sdapin, &noack);
	if (k++ < TM1637_ACK_TIMEOUT)
	    break;
	DELAY(2);
    } while (noack);

    gpio_pin_set_active(sc->tm1637_sclpin, true);
    DELAY(2);
    gpio_pin_setflags(sc->tm1637_sdapin, GPIO_PIN_OUTPUT);
    gpio_pin_set_active(sc->tm1637_sclpin, false);
}

/*
 * Stop command or data transmission
 */
static void
tm1637_gpio_stop(struct tm1637_softc *sc)
{
    gpio_pin_set_active(sc->tm1637_sdapin, false);
    DELAY(2);
    gpio_pin_set_active(sc->tm1637_sclpin, false);
    DELAY(2);
    gpio_pin_set_active(sc->tm1637_sclpin, true);
    DELAY(2);
    gpio_pin_set_active(sc->tm1637_sdapin, true);
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
    size_t position;

#ifdef DEBUG
    uprintf("display: ");
#endif

    // If changes was made, display them
    if(first < TM1637_MAX_COLOM)
    {
	tm1637_gpio_start(sc); // Start a byte
	tm1637_gpio_sendbyte(sc, TM1637_ADDRESS_AUTO); // Send an address autoincrement command to tm1637
	tm1637_gpio_stop(sc);  // Stop a byte

	tm1637_gpio_start(sc); // Start a byte sequence
	tm1637_gpio_sendbyte(sc, TM1637_START_ADDRESS + first); // Send a start address to tm1637

	// Send change digits
	for(position=first; position<=last; position++)
	{

#ifdef DEBUG
	    uprintf("%i[    %02x]", position, sc->tm1637_digits[position]);
#endif

	    tm1637_gpio_sendbyte(sc, sc->tm1637_digits[position]); // Send colom segments to tm1637
	}
	tm1637_gpio_stop(sc); // Stop a byte sequence
    }

#ifdef DEBUG
    uprintf("\n");
#endif
}

/*
 * Display or clear a clockpoint
 */
static void
tm1637_display_clockpoint(struct tm1637_softc *sc, bool clockpoint)
{
    tm1637_gpio_start(sc); // Start a byte
    tm1637_gpio_sendbyte(sc, TM1637_ADDRESS_FIXED); // Send a fixed address command to tm1637
    tm1637_gpio_stop(sc);  // Stop a byte

    tm1637_gpio_start(sc); // Start a byte sequence
    tm1637_gpio_sendbyte(sc, TM1637_START_ADDRESS + TM1637_COLON_POSITION - 1); // Send a start address to tm1637

#ifdef DEBUG
    uprintf("display: ");
    uprintf("%i[    %02x]\n",
            TM1637_COLON_POSITION - 1,
            sc->tm1637_digits[TM1637_COLON_POSITION - 1]);
#endif

    if (clockpoint)
	sc->tm1637_digits[TM1637_COLON_POSITION - 1] |= 0x80;
    else
	sc->tm1637_digits[TM1637_COLON_POSITION - 1] &= 0x7f;

    tm1637_gpio_sendbyte(sc, sc->tm1637_digits[TM1637_COLON_POSITION - 1]); // Send colom segments to tm1637
    tm1637_gpio_stop(sc); // Stop a byte sequence
}

/*
 * Send to display all 4 digits (6 bytes will be sended)
 */
static void
tm1637_update_display(struct tm1637_softc *sc)
{
    size_t first = TM1637_MAX_COLOM, position, last = 0;

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

    // Display an optimized part of row of digits
    // or mark it for update if it is off
    if(sc->tm1637_on)
	tm1637_display_digits(sc, first, last);
    else if(first < TM1637_MAX_COLOM)
	sc->tm1637_needupdate = true;
}

/*
 * Writes all blanks to a display
 */
static void
tm1637_clear_display(struct tm1637_softc *sc)
{
    size_t position = TM1637_MAX_COLOM - 1;

    // Display all blanks
    while(position--)
    {
	sc->tm1637_digits[position] = SIGN_EMPTY;
	sc->tm1637_digits_prev[position] = SIGN_EMPTY;
    }

    // If display is off right now then mark it for update when it is on
    if(sc->tm1637_on)
	tm1637_display_digits(sc, 0, TM1637_MAX_COLOM - 1);
    else
	sc->tm1637_needupdate = true;
}

/*
 * Sets a display on with a brightness value from softc
 */
static void
tm1637_display_on(struct tm1637_softc *sc)
{
    if(sc->tm1637_needupdate)
    {
	tm1637_display_digits(sc, 0, TM1637_MAX_COLOM - 1);
	sc->tm1637_needupdate = false;
    }
    sc->tm1637_on = 1;
    tm1637_gpio_start(sc); // Start a byte
    tm1637_gpio_sendbyte(sc, 0x88|sc->tm1637_brightness);
    tm1637_gpio_stop(sc); // Stop a byte
}

/*
 * Set a display off
 */
static void
tm1637_display_off(struct tm1637_softc *sc)
{
    sc->tm1637_on = 0;
    tm1637_gpio_start(sc); // Start a byte
    tm1637_gpio_sendbyte(sc, 0x80);
    tm1637_gpio_stop(sc); // Stop a byte
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

    devname = device_get_name(sc->tm1637_dev);
    unit = device_get_unit(sc->tm1637_dev);
    busdev = device_get_parent(sc->tm1637_dev);

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
    numpins = gpiobus_get_npins(sc->tm1637_dev);
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
	device_printf(sc->tm1637_dev, 
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
	device_printf(sc->tm1637_dev, "invalid scl hint %d\n", sclnum);
	return (EINVAL);
    }
    if ((err = resource_int_value(devname, unit, "sda", &sdanum)) != 0)
	sdanum = TM1637_SDA_IDX;
    else if (sdanum < 0 || sdanum >= numpins) {
	device_printf(sc->tm1637_dev, "invalid sda hint %d\n", sdanum);
	return (EINVAL);
    }

    /* Allocate gpiobus_pin structs for the pins we found above. */
    if ((err = gpio_pin_get_by_child_index(sc->tm1637_dev, sclnum, &sc->tm1637_sclpin)) != 0)
	return (err);

    if ((err = gpio_pin_get_by_child_index(sc->tm1637_dev, sdanum, &sc->tm1637_sdapin)) != 0)
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
tm1637_ioctl(struct cdev *tm1637_cdev, u_long cmd, caddr_t data, int fflag, struct thread *td)
{
    int error = 0;

    struct tm1637_softc *sc = tm1637_cdev->si_drv1; // Stored here on tm1637_attach()

    switch (cmd)
    {
	case TM1637_IOCTL_CLEAR:
#ifdef DEBUG
	    uprintf("ioctl(display_clear)\n");
#endif
	    tm1637_clear_display(sc);
	    break;
	case TM1637_IOCTL_OFF:
#ifdef DEBUG
	    uprintf("ioctl(display_off)\n");
#endif
	    tm1637_display_off(sc);
	    break;
	case TM1637_IOCTL_ON:
#ifdef DEBUG
	    uprintf("ioctl(display_on)\n");
#endif
	    tm1637_display_on(sc);
	    break;
	case TM1637_IOCTL_BRIGHTNESS:
	    sc->tm1637_brightness = *(uint8_t*)data;
#ifdef DEBUG
	    uprintf("ioctl(brightness, %i)\n", sc->tm1637_brightness);
#endif
	    tm1637_display_on(sc);
	    break;
	case TM1637_IOCTL_CLOCKPOINT:
#ifdef DEBUG
	    uprintf("ioctl(clockpoint, %i)\n", *(uint8_t*)data);
#endif
	    tm1637_display_clockpoint(sc, *(bool*)data);
	    break;
	default:
	    error = ENOTTY;
	    break;
    }

    return (error);
}

static int
tm1637_open(struct cdev *tm1637_cdev, int oflags __unused, int devtype __unused,
    struct thread *td __unused)
{
    struct tm1637_softc *sc = tm1637_cdev->si_drv1; // Stored here on tm1637_attach()

    TM1637_LOCK(sc);
    if (sc->tm1637_inuse) {
        TM1637_UNLOCK(sc);
        return (EBUSY);
    }
    /* move to init */
    sc->tm1637_inuse = true;
    TM1637_UNLOCK(sc);

#ifdef DEBUG
    uprintf("Opened device \"%s\" successfully.\n", tm1637_cdevsw.d_name);
#endif

    return (0);
}

static int
tm1637_close(struct cdev *tm1637_cdev __unused, int fflag __unused, int devtype __unused,
    struct thread *td __unused)
{
    struct tm1637_softc *sc = tm1637_cdev->si_drv1; // Stored here on tm1637_attach()

    sc->tm1637_inuse = false;

#ifdef DEBUG
    uprintf("Closing device \"%s\".\n", tm1637_cdevsw.d_name);
#endif

    return (0);
}

static int
tm1637_read(struct cdev *tm1637_cdev, struct uio *uio, int ioflag __unused)
{
    int error;
    size_t amount;
    struct tm1637_softc *sc = tm1637_cdev->si_drv1; // Stored here on tm1637_attach()

    size_t text_len = sc->tm1637_buf->len + 1;
    amount = MIN(uio->uio_resid, 
		 uio->uio_offset >= text_len ? 0 : text_len - uio->uio_offset);

    if ((error = uiomove(sc->tm1637_buf->text, amount, uio)) != 0)
	uprintf("uiomove failed!\n");

    return (error);
}

static int
tm1637_write(struct cdev *tm1637_cdev, struct uio *uio, int ioflag __unused)
{
    int error;

    struct tm1637_softc *sc = tm1637_cdev->si_drv1;

    if(sc->tm1637_raw_mode > 0)
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

#ifdef FDT
    if (!ofw_bus_status_okay(dev))
	return (ENXIO);
    if (ofw_bus_search_compatible(dev, compat_data)->ocd_data == 0)
	return (ENXIO);
#endif

    device_set_desc(dev, "TM1637 4 Digit 7 Segment Display");

    return (BUS_PROBE_DEFAULT);
}

static int
tm1637_detach(device_t dev)
{
    struct tm1637_softc *sc = device_get_softc(dev);

    tm1637_display_off(sc);

    if (sc->tm1637_cdev != NULL)
	destroy_dev(sc->tm1637_cdev);

    if (sc->tm1637_sclpin != NULL)
	gpio_pin_release(sc->tm1637_sclpin);

    if (sc->tm1637_sdapin != NULL)
	gpio_pin_release(sc->tm1637_sdapin);

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

    sc->tm1637_dev = dev;
    sc->tm1637_node = ofw_bus_get_node(dev);

    /* Acquire our gpio pins. */
    err = tm1637_setup_hinted_pins(sc);

#ifdef FDT
    if (err != 0)
	err = tm1637_setup_fdt_pins(sc);
#endif

    if (err != 0) {
	device_printf(sc->tm1637_dev, "no pins configured\n");
	return (ENXIO);
    }

    /* Say what we came up with for pin config. */
    device_printf(dev, "SCL pin: %s:%d, SDA pin: %s:%d\n",
	device_get_nameunit(GPIO_GET_BUS(sc->tm1637_sclpin->dev)), sc->tm1637_sclpin->pin,
	device_get_nameunit(GPIO_GET_BUS(sc->tm1637_sdapin->dev)), sc->tm1637_sdapin->pin);

    /* Create the tm1637 cdev. */
    err = make_dev_p(MAKEDEV_CHECKNAME | MAKEDEV_WAITOK,
	&sc->tm1637_cdev,
	&tm1637_cdevsw,
	0,
	UID_ROOT,
	GID_WHEEL,
	0600,
	"tm1637");

    if (err != 0) {
	device_printf(dev, "Unable to create tm1637 cdev\n");
	tm1637_detach(dev);
	return (err);
    }

    sc->tm1637_cdev->si_drv1 = sc;

    /* Set properties */
    uint32_t brightness;
    if (OF_getencprop(sc->tm1637_node, "default-brightness-level", &brightness, sizeof(uint32_t)) == sizeof(uint32_t))
    {
	if (brightness <= 7)
	    sc->tm1637_brightness = brightness;
	else
	    sc->tm1637_brightness = TM1637_BRIGHT_DARKEST;
    }

    if (OF_hasprop(sc->tm1637_node, "raw-mode") == 1)
	sc->tm1637_raw_mode = 1;
    else
	sc->tm1637_raw_mode = 0;

    /* Create sysctl variables and set their handlers */
    SYSCTL_ADD_PROC(ctx, SYSCTL_CHILDREN(tree), OID_AUTO,
	"brightness", CTLTYPE_U8 | CTLFLAG_RW | CTLFLAG_MPSAFE, sc, 0,
	&tm1637_brightness_sysctl, "CU", "brightness 0..7. 0 is a darkest one");

    SYSCTL_ADD_PROC(ctx, SYSCTL_CHILDREN(tree), OID_AUTO,
	"on", CTLTYPE_U8 | CTLFLAG_RW | CTLFLAG_MPSAFE, sc, 0,
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
