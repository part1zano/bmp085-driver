/*
 * Simple Echo pseudo-device KLD
 *
 * Murray Stokely
 * Søren (Xride) Straarup
 * Eitan Adler
 */

#include <sys/types.h>
#include <sys/module.h>
#include <sys/systm.h>  /* uprintf */
#include <sys/param.h>  /* defines used in kernel.h */
#include <sys/kernel.h> /* types used in module initialization */
#include <sys/conf.h>   /* cdevsw struct */
#include <sys/uio.h>    /* uio struct */
#include <sys/malloc.h>
#include <dev/iicbus/iic.h> /* iic-related shit */


#include "bmp085.h"

#define BUFFERSIZE 255

/* Function prototypes */
static d_open_t      echo_open;
static d_read_t      echo_read;

/* Character device entry points */
static struct cdevsw echo_cdevsw = {
	.d_version = D_VERSION,
	.d_open = echo_open,
	.d_read = echo_read,
	.d_name = "bmp",
};

struct s_echo {
	char msg[BUFFERSIZE + 1];
	int len;
};

/* vars */
static struct cdev *echo_dev;
static struct s_echo *echomsg;

MALLOC_DECLARE(M_ECHOBUF);
MALLOC_DEFINE(M_ECHOBUF, "echobuffer", "buffer for echo module");

/*
 * This function is called by the kld[un]load(2) system calls to
 * determine what actions to take when a module is loaded or unloaded.
 */
static int
echo_loader(struct module *m __unused, int what, void *arg __unused)
{
	int error = 0;

	switch (what) {
	case MOD_LOAD:                /* kldload */
		error = make_dev_p(MAKEDEV_CHECKNAME | MAKEDEV_WAITOK,
		    &echo_dev,
		    &echo_cdevsw,
		    0,
		    UID_ROOT,
		    GID_OPERATOR,
		    0440,
		    "bmp");
		if (error != 0) {
			break;
		}

		echomsg = malloc(sizeof(*echomsg), M_ECHOBUF, M_WAITOK | M_ZERO);
		// here we initialize the bmp085
		break;
	case MOD_UNLOAD:
		destroy_dev(echo_dev);
		free(echomsg, M_ECHOBUF);
		break;
	default:
		error = EOPNOTSUPP;
		break;
	}
	return (error);
}

static int
echo_open(struct cdev *dev __unused, int oflags __unused, int devtype __unused,
    struct thread *td __unused)
{
	int error = 0;
	char *ptr;
	// here we read the bmp085 and put what we read into char buf[255]

	char buf[255] = "t: 32\np: 10500\n\0";
	ptr = buf;
	do {
		echomsg->msg[echomsg->len++] = *ptr;
	} while (*(++ptr) != '\0');
	
	return (error);
}

/*
 * The read function just takes the buf that was saved via
 * echo_write() and returns it to userland for accessing.
 * uio(9)
 */
static int
echo_read(struct cdev *dev __unused, struct uio *uio, int ioflag __unused)
{
	size_t amt;
	int error;
	// this function should be unchanged, I guess
	amt = MIN(uio->uio_resid, uio->uio_offset >= echomsg->len + 1 ? 0 :
	    echomsg->len + 1 - uio->uio_offset);

	if ((error = uiomove(echomsg->msg, amt, uio)) != 0) {
		uprintf("uiomove failed!\n");
	}

	return (error);
}


DEV_MODULE(echo, echo_loader, NULL);
DRIVER_MODULE(bmp085, iicbus, bmp085_driver, bmp085_devclass, 0, 0); // copied from another one
MODULE_VERSION(bmp085, 1);
MODULE_DEPENDS(bmp085, iicbus, 1, 1, 1);
// idk whether or not I should put something else here
