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

#ifndef _TM1637_KMOD_H_
#define _TM1637_KMOD_H_

#define TM1637_CDEV_NAME	"tm1637"
#define TM1637_SCL_PROPERTY	"scl-gpios"
#define TM1637_SDA_PROPERTY	"sda-gpios"
#define TM1637_SCL_IDX		0
#define TM1637_SDA_IDX		1
#define TM1637_MIN_PINS		2

#define TM1637_ACK_TIMEOUT	200

#define TM1637_ADDRESS_AUTO	0x40
#define TM1637_ADDRESS_FIXED	0x44
#define TM1637_START_ADDRESS	0xc0

#define TM1637_BRIGHT_DARKEST	0
#define TM1637_BRIGHT_TYPICAL	2
#define TM1637_BRIGHTEST	7

#define	TM1637_COLON_POSITION	2 // 2-nd character
#define TM1637_MAX_COLOM	4
#define	TM1637_BUFFERSIZE	TM1637_MAX_COLOM + 2 // For ':' and '\n'

#define TM1637_LOCK_INIT(sc)	\
    mtx_init(&(sc)->lock, "tm1637 mtx", NULL, MTX_DEF)
#define TM1637_LOCK_DESTROY(sc)	\
    mtx_destroy(&(sc)->lock)
#define TM1637_LOCK(sc)		\
    mtx_lock(&(sc)->lock)
#define TM1637_UNLOCK(sc)	\
    mtx_unlock(&(sc)->lock)

struct s_message {
    char text[TM1637_BUFFERSIZE + 1]; // ??? +1
    int offset;
    int len;
};

struct tm1637_softc {
    device_t		 tm1637_dev;
    device_t		 tm1637_busdev;
    gpio_pin_t		 tm1637_sclpin;
    gpio_pin_t		 tm1637_sdapin;
    uint8_t		 tm1637_brightness;
    uint8_t		 tm1637_on;
    uint8_t		 tm1637_mode;
    bool		 tm1637_needupdate;
    bool		 tm1637_inuse;
    u_char		 tm1637_digits[TM1637_MAX_COLOM + 1]; // ??? 1 byte overflow
    u_char		 tm1637_digits_prev[TM1637_MAX_COLOM];
    struct mtx		 lock;
    struct cdev		*tm1637_cdev;
    struct s_message	*tm1637_buf;
    struct s_message	*tm1637_msg;
};

MALLOC_DEFINE(M_TM1637BUF, "tm1637buffer", "Buffer for tm1637 module");

static int tm1637_probe(device_t);
static int tm1637_attach(device_t);
static int tm1637_detach(device_t);

#endif /* _TM1637_KMOD_H_ */
