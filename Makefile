.PATH:	${SRCTOP}/sys/dev/gpio/

KMOD=tm1637
SRCS=tm1637_kmod.c

SRCS+=	\
	bus_if.h \
	device_if.h \
	gpio_if.h \
	gpiobus_if.h \
	ofw_bus_if.h \
	opt_platform.h \

CFLAGS+=  -I. -I${SRCTOP}/sys/dev/gpio/

.include <bsd.kmod.mk>
