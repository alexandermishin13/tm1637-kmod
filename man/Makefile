MAN=tm1637.4
MANSUBDIR=/arm

MANFULLDIR= ${DESTDIR}${MANDIR}4${MANSUBDIR}

beforeinstall:
.if !exists(${MANFULLDIR})
	@mkdir -p ${MANFULLDIR}
.endif

.include <bsd.prog.mk>
