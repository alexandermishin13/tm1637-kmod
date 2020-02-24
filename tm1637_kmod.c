/*
 * KLD tm1637 display
 * Mishin Alexander, 2 Feb 2020
 */

#define FDT

#include <sys/types.h>
#include <sys/module.h>
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

#define	TM1637_CDEV_NAME	"tm1637"
#define	TM1637_SCL_PROPERTY	"scl-gpios"
#define	TM1637_SDA_PROPERTY	"sda-gpios"
#define	TM1637_SCL_IDX		0
#define	TM1637_SDA_IDX		1
#define	TM1637_MIN_PINS		2

#define	TM1637_ACK_TIMEOUT	200

#define	TM1637_ADDRESS_AUTO	0x40
#define	TM1637_ADDRESS_FIXED	0x44
#define TM1637_START_ADDRESS	0xc0

#define	TM1637_BRIGHT_DARKEST	0
#define	TM1637_BRIGHT_TYPICAL	2
#define	TM1637_BRIGHTEST	7

#define	TM1637_MAX_COLOM	4
#define	TM1637_BUFFERSIZE	TM1637_MAX_COLOM + 2 // For ':' and '\0'

MALLOC_DECLARE(M_TM1637BUF);
MALLOC_DEFINE(M_TM1637BUF, "tm1637buffer", "buffer for tm1637 module");

static u_char char_code[] = { 0x3f, 0x06, 0x5b, 0x4f, 0x66, 0x6d, 0x7d, 0x07, 0x7f, 0x6f };

static int tm1637_probe(device_t);
static int tm1637_attach(device_t);
static int tm1637_detach(device_t);

struct s_message {
	char text[TM1637_BUFFERSIZE + 1];
	int len;
};

struct tm1637_softc {
	device_t		 tm1637_dev;
	gpio_pin_t		 tm1637_sclpin;
	gpio_pin_t		 tm1637_sdapin;
	uint8_t			 tm1637_brightness;
	uint8_t			 tm1637_on;
	uint8_t			 tm1637_raw_format;
	bool			 tm1637_colon;
	u_char			 tm1637_digits[TM1637_MAX_COLOM];
	u_char			 tm1637_digits_prev[TM1637_MAX_COLOM];
	struct cdev		*tm1637_cdev;
	struct s_message	*tm1637_msg;
};

static void tm1637_display_on(struct tm1637_softc *sc);
static void tm1637_display_off(struct tm1637_softc *sc);

#ifdef FDT
#include <dev/ofw/ofw_bus.h>
//#include <dev/ofw/ofw_bus_subr.h>

static struct ofw_compat_data compat_data[] = {
	{"tm1637",  true},
	{NULL,     false}
};

OFWBUS_PNP_INFO(compat_data);
SIMPLEBUS_PNP_INFO(compat_data);

static int
tm1637_setup_fdt_pins(struct tm1637_softc *sc)
{
	phandle_t node;
	int err;

	node = ofw_bus_get_node(sc->tm1637_dev);

	/*
	 * Historically, we used the first two array elements of the gpios
	 * property.  The modern bindings specify separate scl-gpios and
	 * sda-gpios properties.  We cope with whichever is present.
	 */
	if (OF_hasprop(node, "gpios")) {
		if ((err = gpio_pin_get_by_ofw_idx(sc->tm1637_dev, node, TM1637_SCL_IDX, &sc->tm1637_sclpin)) != 0)
		{
			device_printf(sc->tm1637_dev, "invalid gpios property for index:%d\n", TM1637_SCL_IDX);
			return (err);
		}
		if ((err = gpio_pin_get_by_ofw_idx(sc->tm1637_dev, node, TM1637_SDA_IDX, &sc->tm1637_sdapin)) != 0)
		{
			device_printf(sc->tm1637_dev, "invalid gpios property for index:%d\n", TM1637_SDA_IDX);
			return (err);
		}
	} else {
		if ((err = gpio_pin_get_by_ofw_property(sc->tm1637_dev, node, TM1637_SCL_PROPERTY, &sc->tm1637_sclpin)) != 0)
		{
			device_printf(sc->tm1637_dev, "missing %s property\n", TM1637_SCL_PROPERTY);
			return (err);
		}
		if ((err = gpio_pin_get_by_ofw_property(sc->tm1637_dev, node, TM1637_SDA_PROPERTY, &sc->tm1637_sdapin)) != 0)
		{
			device_printf(sc->tm1637_dev, "missing %s property\n", TM1637_SDA_PROPERTY);
			return (err);
		}
	}
	return (0);
}
#endif /* FDT */

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

static int
tm1637_raw_format_sysctl(SYSCTL_HANDLER_ARGS)
{
	struct tm1637_softc *sc = arg1;
	uint8_t _raw = sc->tm1637_raw_format;
	int error = 0;

	error = SYSCTL_OUT(req, &_raw, sizeof(_raw));
	if (error != 0 || req->newptr == NULL)
		return (error);

	error = SYSCTL_IN(req, &_raw, sizeof(_raw));
	if (error != 0)
		return (error);

	switch(_raw)
	{
	    case 0:
//		tm1637_display_off(sc);
		break;
	    case 1:
//		tm1637_display_on(sc);
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
//	gpio_pin_set_active(sc->tm1637_sclpin, false);
}

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

static void
tm1637_set_clockpoint(struct tm1637_softc *sc, bool value)
{
	sc->tm1637_colon = value;
}

/*
 * Send to display a given digit (3 bytes will be sended)
 */
static void
tm1637_display_digit(struct tm1637_softc *sc, size_t position)
{
	if (position >= TM1637_MAX_COLOM)
	  return;

	if(sc->tm1637_digits_prev[position] != sc->tm1637_digits[position])
	{
		sc->tm1637_digits_prev[position] = sc->tm1637_digits[position];

		tm1637_gpio_start(sc); // Start a byte
		tm1637_gpio_sendbyte(sc, TM1637_ADDRESS_FIXED); // Send an address autoincrement command to tm1637
		tm1637_gpio_stop(sc);  // Stop a byte

		tm1637_gpio_start(sc); // Start a byte sequence
		tm1637_gpio_sendbyte(sc, TM1637_START_ADDRESS + position); // Send a start address to tm1637

#ifdef DEBUG
		uprintf("displaay: [%i:%x]\n", position, sc->tm1637_digits[position]);
#endif

		tm1637_gpio_sendbyte(sc, sc->tm1637_digits[position]); // Send colom segments to tm1637
		tm1637_gpio_stop(sc); // Stop a byte sequence
	}
}

/*
 * Send to display all 4 digits (6 bytes will be sended)
 */
static void
tm1637_display_digits(struct tm1637_softc *sc)
{
	size_t position = TM1637_MAX_COLOM;
	size_t first = TM1637_MAX_COLOM, last = 0;

#ifdef DEBUG
	uprintf("changed: ");
#endif
	while(position--)
	{
		if(sc->tm1637_digits_prev[position] != sc->tm1637_digits[position])
		{
#ifdef DEBUG
			uprintf("[%i:%x]", position, sc->tm1637_digits[position]);
#endif
			sc->tm1637_digits_prev[position] = sc->tm1637_digits[position];

			first = position;
			if (last == 0)
			    last = position;
		}
	}

#ifdef DEBUG
	uprintf("\n");
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

		for(position=first; position<=last; position++)
		{

#ifdef DEBUG
			uprintf("[%i:%x]", position, sc->tm1637_digits[position]);
#endif

			tm1637_gpio_sendbyte(sc, sc->tm1637_digits[position]); // Send colom segments to tm1637
		}
		tm1637_gpio_stop(sc); // Stop a byte sequence

#ifdef DEBUG
	uprintf("\n");
#endif
	}
}

/*
 * Writes all blanks to a display
 */
static void
tm1637_display_clear(struct tm1637_softc *sc)
{
	size_t position = TM1637_MAX_COLOM;

	// Display all blanks
	while(position--)
	{
	    sc->tm1637_digits[position] = 0x00;
	    sc->tm1637_digits_prev[position] = 0xff; // ... forced
	}
	tm1637_display_digits(sc);
}

/*
 * Sets a display on with a brightness value from softc
 */
static void
tm1637_display_on(struct tm1637_softc *sc)
{
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

static void
tm1637_decode_string(struct tm1637_softc *sc, u_char* s)
{
	int ic, id = TM1637_MAX_COLOM - 1;

	// Number of digits + 1 for a colon, if any
	for(ic=TM1637_MAX_COLOM; ic>=0; ic--)
	{
	    unsigned char c = s[ic];
	    if(c>='0' && c<='9') // encode if digit
	    {
		sc->tm1637_digits[id--] = char_code[c&0x0f];
	    }
	    else if(c=='#') // No changes if wildcard
	    {
		id--;
	    }
	    else if(ic==2)
	    {
		// No switch to next digit, just set flag
		switch(c)
		{
		    case ':':
			sc->tm1637_colon = true;
			break;
		    case ' ':
			sc->tm1637_colon = false;
			break;
		}
	    }
	    else
		sc->tm1637_digits[id--] = 0x00;
	}

	// Set or unset a colon segment
	if(sc->tm1637_colon)
	    sc->tm1637_digits[1] |= 0x80;
	else
	    sc->tm1637_digits[1] &= 0x7f;
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

/* Character device entry points */
static struct cdevsw tm1637_cdevsw = {
    .d_version = D_VERSION,
    .d_open = tm1637_open,
    .d_close = tm1637_close,
    .d_read = tm1637_read,
    .d_write = tm1637_write,
    .d_name = TM1637_CDEV_NAME,
};

static int
tm1637_open(struct cdev *tm1637_cdev __unused, int oflags __unused, int devtype __unused,
    struct thread *td __unused)
{

#ifdef DEBUG
    uprintf("Opened device \"%s\" successfully.\n", tm1637_cdevsw.d_name);
#endif

    return (0);
}

static int
tm1637_close(struct cdev *tm1637_cdev __unused, int fflag __unused, int devtype __unused,
    struct thread *td __unused)
{

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

	size_t text_len = sc->tm1637_msg->len + 1;
	amount = MIN(uio->uio_resid, 
		     uio->uio_offset >= text_len ? 0 : text_len - uio->uio_offset);

	if ((error = uiomove(sc->tm1637_msg->text, amount, uio)) != 0)
	    uprintf("uiomove failed!\n");

	return (error);
}

static int
tm1637_write(struct cdev *tm1637_cdev, struct uio *uio, int ioflag __unused)
{
	int error;
	size_t amount;
	size_t available;
	struct tm1637_softc *sc = tm1637_cdev->si_drv1; // Stored here on tm1637_attach()

	/*
	 * We either write from the beginning or are appending -- do
	 * not allow random access.
	 */
	if (uio->uio_offset != 0 && (uio->uio_offset != sc->tm1637_msg->len))
		return (EINVAL);

	// This is a new message, reset length
	if (uio->uio_offset == 0)
		sc->tm1637_msg->len = 0;

	available = TM1637_BUFFERSIZE - sc->tm1637_msg->len;
	if (uio->uio_resid > available)
		return (EINVAL);

	// Copy the string in from user memory to kernel memory
	amount = MIN(uio->uio_resid, available);

	error = uiomove(sc->tm1637_msg->text + uio->uio_offset, amount, uio);

	// Terminate the message by zero and set the length
	sc->tm1637_msg->len = uio->uio_offset;
	sc->tm1637_msg->text[sc->tm1637_msg->len] = 0;

	if (error != 0)
	    uprintf("Write failed: bad address!\n");
	else
	{
	    tm1637_decode_string(sc, sc->tm1637_msg->text);
	    tm1637_display_digits(sc);
	}

	return (error);
}

static void
tm1637_cleanup(struct tm1637_softc *sc)
{

	device_delete_children(sc->tm1637_dev);

	if (sc->tm1637_cdev != NULL)
		destroy_dev(sc->tm1637_cdev);

	if (sc->tm1637_sclpin != NULL)
		gpio_pin_release(sc->tm1637_sclpin);

	if (sc->tm1637_sdapin != NULL)
		gpio_pin_release(sc->tm1637_sdapin);

	free(sc->tm1637_msg, M_TM1637BUF);
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
	int err;

	tm1637_display_off(sc);

	if ((err = bus_generic_detach(dev)) != 0)
		return (err);

	tm1637_cleanup(sc);

	return (0);
}

static int
tm1637_attach(device_t dev)
{
	struct tm1637_softc	*sc;
	struct sysctl_ctx_list	*ctx;
	struct sysctl_oid	*tree;
	int err;

	sc = device_get_softc(dev);
	ctx = device_get_sysctl_ctx(dev);
	tree = device_get_sysctl_tree(dev);

	sc->tm1637_dev = dev;

	/* Acquire our gpio pins. */
	err = tm1637_setup_hinted_pins(sc);
#ifdef FDT
	if (err != 0)
		err = tm1637_setup_fdt_pins(sc);
#endif
	if (err != 0) {
		device_printf(sc->tm1637_dev, "no pins configured\n");
		tm1637_cleanup(sc);
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

	sc->tm1637_brightness = TM1637_BRIGHT_DARKEST;
	sc->tm1637_colon = false;
	sc->tm1637_cdev->si_drv1 = sc;

	SYSCTL_ADD_PROC(ctx, SYSCTL_CHILDREN(tree), OID_AUTO,
	    "brightness", CTLTYPE_U8 | CTLFLAG_RW | CTLFLAG_MPSAFE, sc, 0,
	    &tm1637_brightness_sysctl, "CU", "brightness 0..7. 0 is a darkest one");

	SYSCTL_ADD_PROC(ctx, SYSCTL_CHILDREN(tree), OID_AUTO,
	    "on", CTLTYPE_U8 | CTLFLAG_RW | CTLFLAG_MPSAFE, sc, 0,
	    &tm1637_set_on_sysctl, "CU", "display is on or off");

	SYSCTL_ADD_PROC(ctx, SYSCTL_CHILDREN(tree), OID_AUTO,
	    "raw_format", CTLTYPE_U8 | CTLFLAG_RW | CTLFLAG_MPSAFE, sc, 0,
	    &tm1637_raw_format_sysctl, "CU", "4 bytes of digits segments");

	sc->tm1637_msg = malloc(sizeof(*sc->tm1637_msg), M_TM1637BUF, M_WAITOK | M_ZERO);

	tm1637_display_clear(sc);
	tm1637_display_on(sc);

	return (bus_generic_attach(dev));
}

static device_method_t tm1637_methods[] = {
	/* Device interface */
	DEVMETHOD(device_probe,		tm1637_probe),
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
