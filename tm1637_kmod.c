/*
 * KLD tm1637 display
 * Mishin Alexander, 2 Feb 2020
 */

#define FDT

#include <sys/types.h>
#include <sys/module.h>
#include <sys/systm.h>  /* uprintf */
#include <sys/conf.h>   /* cdevsw struct */
#include <sys/param.h>  /* defines used in kernel.h */
#include <sys/kernel.h> /* types used in module initialization */
#include <sys/bus.h>
#include <sys/uio.h>    /* uio struct */
#include <sys/malloc.h>
//#include <sys/taskqueue.h>

#include <dev/gpio/gpiobusvar.h>

#define BUFFERSIZE 6

#define	TM1637_CDEV_NAME	"tm1637"
#define	TM1637_SCL_PROPERTY	"scl-gpios"
#define	TM1637_SDA_PROPERTY	"sda-gpios"
#define	TM1637_SCL_IDX		0
#define	TM1637_SDA_IDX		1
#define	TM1637_MIN_PINS		2

struct tm1637_softc {
	device_t		 dev;
	gpio_pin_t		 tm1637_sclpin;
	gpio_pin_t		 tm1637_sdapin;
	struct cdev		*tm1637_cdev;
};

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

	node = ofw_bus_get_node(sc->dev);

	/*
	 * Historically, we used the first two array elements of the gpios
	 * property.  The modern bindings specify separate scl-gpios and
	 * sda-gpios properties.  We cope with whichever is present.
	 */
	if (OF_hasprop(node, "gpios")) {
		if ((err = gpio_pin_get_by_ofw_idx(sc->dev, node, TM1637_SCL_IDX, &sc->tm1637_sclpin)) != 0)
		{
			device_printf(sc->dev, "invalid gpios property for index:%d\n", TM1637_SCL_IDX);
			return (err);
		}
		if ((err = gpio_pin_get_by_ofw_idx(sc->dev, node, TM1637_SDA_IDX, &sc->tm1637_sdapin)) != 0)
		{
			device_printf(sc->dev, "invalid gpios property for index:%d\n", TM1637_SDA_IDX);
			return (err);
		}
	} else {
		if ((err = gpio_pin_get_by_ofw_property(sc->dev, node, TM1637_SCL_PROPERTY, &sc->tm1637_sclpin)) != 0)
		{
			device_printf(sc->dev, "missing %s property\n", TM1637_SCL_PROPERTY);
			return (err);
		}
		if ((err = gpio_pin_get_by_ofw_property(sc->dev, node, TM1637_SDA_PROPERTY, &sc->tm1637_sdapin)) != 0)
		{
			device_printf(sc->dev, "missing %s property\n", TM1637_SDA_PROPERTY);
			return (err);
		}
	}
	return (0);
}
#endif /* FDT */

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
	if ((err = gpio_pin_get_by_child_index(sc->dev, sclnum, &sc->tm1637_sclpin)) != 0)
		return (err);
	if ((err = gpio_pin_get_by_child_index(sc->dev, sdanum, &sc->tm1637_sdapin)) != 0)
		return (err);

	return (0);
}

/* Function prototypes */
static d_open_t      tm1637_open;
static d_close_t     tm1637_close;
static d_read_t      tm1637_read;
static d_write_t     tm1637_write;

struct s_message {
    char text[BUFFERSIZE + 1];
    int len;
};

/* vars */
//static struct cdev *tm1637_dev;
static struct s_message *tm1637_msg;

MALLOC_DECLARE(M_TM1637BUF);
MALLOC_DEFINE(M_TM1637BUF, "tm1637buffer", "buffer for tm1637 module");

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
tm1637_open(struct cdev *dev __unused, int oflags __unused, int devtype __unused,
    struct thread *td __unused)
{
    int error = 0;

    uprintf("Opened device \"%s\" successfully.\n", tm1637_cdevsw.d_name);
    return (error);
}

static int
tm1637_close(struct cdev *dev __unused, int fflag __unused, int devtype __unused,
    struct thread *td __unused)
{

    uprintf("Closing device \"%s\".\n", tm1637_cdevsw.d_name);
    return (0);
}

static int
tm1637_read(struct cdev *dev __unused, struct uio *uio, int ioflag __unused)
{
    int error;
    size_t amount;

    size_t text_len = tm1637_msg->len + 1;
    amount = MIN(uio->uio_resid, 
                 uio->uio_offset >= text_len ? 0 : text_len - uio->uio_offset);

    if ((error = uiomove(tm1637_msg->text, amount, uio)) != 0)
	uprintf("uiomove failed!\n");

    error = 0;
    return (error);
}

static int
tm1637_write(struct cdev *dev __unused, struct uio *uio, int ioflag __unused)
{
    int error;
    size_t amount;

    /*
     * We either write from the beginning or are appending -- do
     * not allow random access.
     */
    if (uio->uio_offset != 0 && (uio->uio_offset != tm1637_msg->len))
	return (EINVAL);

    // This is a new message, reset length
    if (uio->uio_offset == 0)
	tm1637_msg->len = 0;

    // Copy the string in from user memory to kernel memory
    amount = MIN(uio->uio_resid, (BUFFERSIZE - tm1637_msg->len));

    error = uiomove(tm1637_msg->text + uio->uio_offset, amount, uio);

    // Terminate the message by zero and set the length
    tm1637_msg->len = uio->uio_offset;
    tm1637_msg->text[tm1637_msg->len] = 0;

    if (error != 0)
	uprintf("Write failed: bad address!\n");
    return (error);
}

static void
tm1637_cleanup(struct tm1637_softc *sc)
{

	device_delete_children(sc->dev);

	if (sc->tm1637_cdev != NULL)
		destroy_dev(sc->tm1637_cdev);

	if (sc->tm1637_sclpin != NULL)
		gpio_pin_release(sc->tm1637_sclpin);

	if (sc->tm1637_sdapin != NULL)
		gpio_pin_release(sc->tm1637_sdapin);

	free(tm1637_msg, M_TM1637BUF);
}

static int
tm1637_probe(device_t dev)
{

#ifdef FDT
	if (!ofw_bus_status_okay(dev))
		return (ENXIO);
device_printf(dev, "probing\n");

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

	if ((err = bus_generic_detach(dev)) != 0)
		return (err);

	tm1637_cleanup(sc);

	return (0);
}

static int
tm1637_attach(device_t dev)
{
	struct tm1637_softc	*sc;
	int err;

device_printf(dev, "attaching\n");
	sc = device_get_softc(dev);

	sc->dev = dev;

	/* Acquire our gpio pins. */
	err = tm1637_setup_hinted_pins(sc);
#ifdef FDT
	if (err != 0)
		err = tm1637_setup_fdt_pins(sc);
#endif
	if (err != 0) {
		device_printf(sc->dev, "no pins configured\n");
		tm1637_cleanup(sc);
		return (ENXIO);
	}

	/* Say what we came up with for pin config. */
	device_printf(dev, "SCL pin: %s:%d, SDA pin: %s:%d\n",
	    device_get_nameunit(GPIO_GET_BUS(sc->tm1637_sclpin->dev)), sc->tm1637_sclpin->pin,
	    device_get_nameunit(GPIO_GET_BUS(sc->tm1637_sdapin->dev)), sc->tm1637_sdapin->pin);

	/* Create the RFC 2783 pps-api cdev. */

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

	tm1637_msg = malloc(sizeof(*tm1637_msg), M_TM1637BUF, M_WAITOK | M_ZERO);

	return (bus_generic_attach(dev));
//	return (0);
}

/* Driver bits */
static device_method_t tm1637_methods[] = {
	/* Device interface */
	DEVMETHOD(device_probe,			tm1637_probe),
	DEVMETHOD(device_attach,		tm1637_attach),
	DEVMETHOD(device_detach,		tm1637_detach),

	DEVMETHOD_END
};

static devclass_t tm1637_devclass;

DEFINE_CLASS_0(tm1637, tm1637_driver, tm1637_methods, sizeof(struct tm1637_softc));

#ifdef FDT
DRIVER_MODULE(tm1637, simplebus, tm1637_driver, tm1637_devclass, 0, 0);
#endif

DRIVER_MODULE(tm1637, gpiobus, tm1637_driver, tm1637_devclass, 0, 0);
MODULE_DEPEND(tm1637, gpiobus, 1, 1, 1);
