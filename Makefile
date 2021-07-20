# $FreeBSD$

.PATH:	${SRCTOP}/sys/dev/gpio/

KMOD=tm1637
SRCS=tm1637_kmod.c
SUBDIR=man include

SRCS+=	\
	bus_if.h \
	device_if.h \
	gpio_if.h \
	gpiobus_if.h \
	ofw_bus_if.h \
	opt_platform.h \
	fdt_pinctrl_if.h \

CFLAGS+=  -I. -I${SRCTOP}/sys/dev/gpio/
#CFLAGS+=  -DI2C_LIKE
#CFLAGS+=  -DDEBUG

.include <bsd.kmod.mk>
