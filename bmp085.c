/*
 * bmp085 thermometer/barometer driver
 *
 * Copyright (c) Maxim V Filimonov, 2015..2017
 * This code is distributed under the BSD 3 Clause License. I'm too lazy to attach the text. Just kurwa google it.
 */

#include "opt_platform.h"

#include <sys/param.h>
#include <sys/bus.h>
#include <sys/endian.h>
#include <sys/kernel.h>
#include <sys/module.h>
#include <sys/sysctl.h>
#include <sys/systm.h>

#include <machine/bus.h>

#include <dev/iicbus/iicbus.h>
#include <dev/iicbus/iiconf.h>

#ifdef FDT
#include <dev/ofw/openfirm.h>
#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>
#endif

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

static int bmp085_read(device_t dev, uint32_t addr, uint8_t reg, uint8_t *data, size_t len) {
	struct iic_msg msg[2] = {
		{ addr, IIC_M_WR | IIC_M_NOSTOP | IIC_M_NOSTART, 1, &reg },
		{ addr, IIC_M_RD | IIC_M_NOSTOP | IIC_M_NOSTART, len, data },
	};

	if (iicbus_transfer(dev, msg, nitems(msg)) != 0) {
		return -1;
	}
	return 0;
}

static int bmp085_write(device_t dev, uint32_t addr, uint8_t reg, uint8_t *data, size_t len) {
	struct iic_msg msg[1] = {
		{ addr, IIC_M_WR, len, data },
	};

	if (iicbus_transfer(dev, msg, nitems(msg)) != 0) {
		return -1;
	}
	return 0;
}

static int bmp085_probe(device_t dev) {
	struct bmp085_softc *sc;

	sc = device_get_softc(dev);
	
#ifdef FDT
	if (!ofw_bus_is_compatible(dev, "siemens,bmp085")) {
		return ENXIO;
	}
#endif
	device_set_desc(dev, "BMP085 temperature/pressure sensor");
	return BUS_PROBE_GENERIC;
}


