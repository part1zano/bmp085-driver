/*
 * Simple Echo pseudo-device KLD
 *
 * Murray Stokely
 * Søren (Xride) Straarup
 * Eitan Adler
 */

// old includes? fuck these
// #include <sys/types.h>
// #include <sys/param.h>  /* defines used in kernel.h */
// #include <sys/systm.h>  /* uprintf */
// #include <sys/bus.h>
// #include <sys/kernel.h> /* types used in module initialization */
// #include <sys/module.h>
// #include <sys/conf.h>   /* cdevsw struct */
// #include <sys/uio.h>    /* uio struct */
// #include <sys/malloc.h>

// // #include <machine/bus.h>
// // #include <sys/rman.h>
// // #include <machine/resource.h>
// #include <dev/iicbus/iicbus.h>
// #include <dev/iicbus/iiconf.h>

// #include "bus_if.h"
// #include "device_if.h"

// new includes I just copied!

#include <sys/cdefs.h>
#include <sys/param.h>
#include <sys/bus.h>
#include <sys/systm.h>
#include <sys/module.h>
#include <sys/uio.h>    /* uio struct */
#include <sys/malloc.h>
#include <sys/callout.h>
#include <sys/conf.h>
#include <sys/cpu.h>
#include <sys/ctype.h>
#include <sys/kernel.h>
#include <sys/reboot.h>
#include <sys/rman.h>
#include <sys/sysctl.h>
#include <sys/limits.h>

#include <machine/bus.h>
#include <machine/md_var.h>

#include <dev/iicbus/iicbus.h>
#include <dev/iicbus/iiconf.h>

#include "bmp085.h"

static int bmp085_probe(device_t);
static int bmp085_attach(device_t);

// XXX :: I don't know what _softc does yet
struct bmp085_softc {
	device_t		sc_dev;
	struct intr_config_hook enum_hook;
	int32_t			sc_hwtype;
	uint32_t		sc_addr;
	uint32_t		sc_conf;
};


// XXX :: I don't know whether or not I should leave this empty
static device_method_t bmp085_methods[] = {
	DEVMETHOD(device_probe, bmp085_probe),
	DEVMETHOD(device_attach, bmp085_attach),

	DEVMETHOD_END
};

static driver_t bmp085_driver = {
	"bmp085",
	bmp085_methods,
	sizeof (struct bmp085_softc)
};

static devclass_t bmp085_devclass;

DRIVER_MODULE(bmp085, iicbus, bmp085_driver, bmp085_devclass, 0, 0); // copied from another one
// MODULE_VERSION(bmp085, 1);
// MODULE_DEPENDS(echo, iicbus, 1, 1, 1);
// idk whether or not I should put something else here
