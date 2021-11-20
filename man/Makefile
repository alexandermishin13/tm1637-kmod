MAN=tm1637.4

PREFIX= /usr/local

MANDIR= ${PREFIX}/man/man

MANFULLDIR= ${DESTDIR}${MANDIR}4

beforeinstall:
.if !exists(${MANFULLDIR})
	@mkdir -v -p ${MANFULLDIR}
.endif

uninstall:
.if exists(${MANFULLDIR}/${MAN}.gz)
	@rm -v ${MANFULLDIR}/${MAN}.gz
	@-rmdir ${MANFULLDIR}
.endif

.include <bsd.prog.mk>
