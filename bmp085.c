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

static bmp085_param param;
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

static void bmp085_start(void *);
static int bmp085_read(device_t, uint32_t, uint8_t, uint8_t *, size_t);
static int bmp085_write(device_t, uint32_t, uint8_t, uint8_t *, size_t);
static int bmp085_temp_sysctl(SYSCTL_HANDLER_ARGS);
static int bmp085_pressure_sysctl(SYSCTL_HANDLER_ARGS);

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
	uprintf("probing bmp085\n");
	struct bmp085_softc *sc;

	sc = device_get_softc(dev);
	device_printf(dev, "probing bmp085\n");
	
#ifdef FDT
	if (!ofw_bus_is_compatible(dev, "siemens,bmp085")) {
		return ENXIO;
	}
#endif
	device_set_desc(dev, "BMP085 temperature/pressure sensor");
	uprintf("about to return BUS_PROBE_GENERIC\n");
	return BUS_PROBE_GENERIC;
}

static int bmp085_attach(device_t dev) {
	struct bmp085_softc *sc;
	uprintf("attaching bmp085\n");

	sc = device_get_softc(dev);
	sc -> sc_dev = dev;
	sc -> sc_addr = iicbus_get_addr(dev);

	sc -> enum_hook.ich_func = bmp085_start;
	sc -> enum_hook.ich_arg = dev;

	if (config_intrhook_establish(&sc->enum_hook) != 0) {
		return ENOMEM;
	}
	device_printf(dev, "about to finish attaching device\n");
	return 0;
}

static void bmp085_start(void *xdev) {
	uprintf("about to start bmp085");
	device_t dev;
	struct bmp085_softc *sc;
	struct sysctl_ctx_list *ctx;
	struct sysctl_oid *tree_node;
	struct sysctl_oid_list *tree;

	dev = (device_t)xdev;
	sc = device_get_softc(dev);
	ctx = device_get_sysctl_ctx(dev);
	tree_node = device_get_sysctl_tree(dev);
	tree = SYSCTL_CHILDREN(tree_node);

	config_intrhook_disestablish(&sc->enum_hook);

	SYSCTL_ADD_PROC(ctx, tree, OID_AUTO, "temperature",
			CTLTYPE_INT | CTLFLAG_RD | CTLFLAG_MPSAFE, dev, 33,
			bmp085_temp_sysctl, "IK", "Current temperature");
	SYSCTL_ADD_PROC(ctx, tree, OID_AUTO, "pressure",
			CTLTYPE_INT | CTLFLAG_RD | CTLFLAG_MPSAFE, dev, 34,
			bmp085_pressure_sysctl, "IK", "Current athmospheric pressure");
	// now we're just setting it up
	uint8_t buffer_tx;
	uint8_t buffer_rx[2];

	buffer_tx = BMP_AC1;
	if (bmp085_read(sc->sc_dev, sc->sc_addr, buffer_tx, buffer_rx, 2*sizeof(uint8_t)) < 0) {
		device_printf(dev, "cannot write to sensor\n");
		return;
	}
	param.ac1 = ((buffer_rx[0] << 8) | buffer_rx[1]);

	buffer_tx = BMP_AC2;
	if (bmp085_read(sc->sc_dev, sc->sc_addr, buffer_tx, buffer_rx, 2*sizeof(uint8_t)) < 0) {
		device_printf(dev, "cannot write to sensor\n");
		return;
	}
	param.ac2 = ((buffer_rx[0] << 8) | buffer_rx[1]);

	buffer_tx = BMP_AC3;
	if (bmp085_read(sc->sc_dev, sc->sc_addr, buffer_tx, buffer_rx, 2*sizeof(uint8_t)) < 0) {
		device_printf(dev, "cannot write to sensor\n");
		return;
	}
	param.ac3 = ((buffer_rx[0] << 8) | buffer_rx[1]);

	buffer_tx = BMP_AC4;
	if (bmp085_read(sc->sc_dev, sc->sc_addr, buffer_tx, buffer_rx, 2*sizeof(uint8_t)) < 0) {
		device_printf(dev, "cannot write to sensor\n");
		return;
	}
	param.ac4 = ((buffer_rx[0] << 8) | buffer_rx[1]);

	buffer_tx = BMP_AC5;
	if (bmp085_read(sc->sc_dev, sc->sc_addr, buffer_tx, buffer_rx, 2*sizeof(uint8_t)) < 0) {
		device_printf(dev, "cannot write to sensor\n");
		return;
	}
	param.ac5 = ((buffer_rx[0] << 8) | buffer_rx[1]);

	buffer_tx = BMP_AC6;
	if (bmp085_read(sc->sc_dev, sc->sc_addr, buffer_tx, buffer_rx, 2*sizeof(uint8_t)) < 0) {
		device_printf(dev, "cannot write to sensor\n");
		return;
	}
	param.ac6 = ((buffer_rx[0] << 8) | buffer_rx[1]);

	buffer_tx = BMP_B1;
	if (bmp085_read(sc->sc_dev, sc->sc_addr, buffer_tx, buffer_rx, 2*sizeof(uint8_t)) < 0) {
		device_printf(dev, "cannot write to sensor\n");
		return;
	}
	param.b1 = ((buffer_rx[0] << 8) | buffer_rx[1]);

	buffer_tx = BMP_B2;
	if (bmp085_read(sc->sc_dev, sc->sc_addr, buffer_tx, buffer_rx, 2*sizeof(uint8_t)) < 0) {
		device_printf(dev, "cannot write to sensor\n");
		return;
	}
	param.b2 = ((buffer_rx[0] << 0) | buffer_rx[1]);

	buffer_tx = BMP_MB;
	if (bmp085_read(sc->sc_dev, sc->sc_addr, buffer_tx, buffer_rx, 2*sizeof(uint8_t)) < 0) {
		device_printf(dev, "cannot write to sensor\n");
		return;
	}
	param.mb = ((buffer_rx[0] << 0) | buffer_rx[1]);

	buffer_tx = BMP_MC;
	if (bmp085_read(sc->sc_dev, sc->sc_addr, buffer_tx, buffer_rx, 2*sizeof(uint8_t)) < 0) {
		device_printf(dev, "cannot write to sensor\n");
		return;
	}
	param.mc = ((buffer_rx[0] << 0) | buffer_rx[1]);

	buffer_tx = BMP_MD;
	if (bmp085_read(sc->sc_dev, sc->sc_addr, buffer_tx, buffer_rx, 2*sizeof(uint8_t)) < 0) {
		device_printf(dev, "cannot write to sensor\n");
		return;
	}
	param.md = ((buffer_rx[0] << 0) | buffer_rx[1]);
	
	device_printf(dev, "started bmp085");
	uprintf("started bmp085");
}

static int bmp085_temp_sysctl(SYSCTL_HANDLER_ARGS) {
	device_t dev;
	struct bmp085_softc *sc;
	int error;

	dev = (device_t)arg1;
	sc = device_get_softc(dev);

	int32_t utemp;
	int32_t x1, x2;
	int32_t temperature = 0;
	uint8_t buffer_tx[2];
	uint8_t buffer_rx[2];

	buffer_tx[0] = BMP_CR;
	buffer_tx[1] = BMP_MODE_TEMP;
	if (bmp085_write(sc->sc_dev, sc->sc_addr, BMP_CR, &buffer_tx[1], 1) != 0) {
		return EIO;
	}
	if (bmp085_read(sc->sc_dev, sc->sc_addr, BMP_DATA, buffer_rx, 2*sizeof(uint8_t)) != 0) {
		return EIO;
	}

	utemp = (int32_t)((buffer_rx[0] << 8) | buffer_rx[1]);

	x1 = ((utemp-param.ac6) * param.ac5)/32768;
	x2 = (param.mc*2048) / (x1 + param.md);
	param.b5 = x1 + x2;
	temperature = (param.b5 + 8) / 16;

	error = sysctl_handle_int(oidp, &temperature, 0, req);
	if (error != 0 || req -> newptr == NULL) {
		return error;
	}
	return 0;
}

static int bmp085_pressure_sysctl(SYSCTL_HANDLER_ARGS) {
	device_t dev;
	struct bmp085_softc *sc;
	int error;

	dev = (device_t)arg1;
	sc = device_get_softc(dev);

	int32_t upress;
	int32_t x1, x2, x3;
	int32_t b3, b6;
	uint32_t b4, b7;
	uint8_t oss = 3;
	int32_t pressure = 0;

	uint8_t buffer_tx[2];
	uint8_t buffer_rx[3];

	b3 = 0;

	buffer_tx[0] = BMP_MODE_PR0+(oss<<6);
	if (bmp085_write(sc->sc_dev, sc->sc_addr, BMP_CR, &buffer_tx[1], 1) != 0) {
		device_printf(dev, "couldnt get pressure at first\n");
		return EIO;
	}

	if (bmp085_read(sc->sc_dev, sc->sc_addr, BMP_DATA, buffer_rx, 3*sizeof(uint8_t)) != 0) {
		device_printf(dev, "couldn't actually get pressure\n");
		return EIO;
	}

	upress = (int32_t)((buffer_rx[0] << 16) | (buffer_rx[1] << 8) | buffer_rx[2]);
	upress = upress >> (8-oss);

	b6 = param.b5 - 4000;
	x1 = (param.b2*((b6*b6)/4096))/2048;
	x2 = (param.ac2*b6)/2048;
	x3 = x1+x2;

	if (oss == 3) {
		b3 = ((int32_t)param.ac1 * 4 + x3 + 2) << 1;
	} else if (oss == 2) {
		b3 = ((int32_t)param.ac1 * 4 + x3 + 2);
	} else if (oss == 1) {
		b3 = ((int32_t)param.ac1 * 4 + x3 + 2) >> 1;
	} else if (oss == 0) {
		b3 = ((int32_t)param.ac1 * 4 + x3 + 2) >> 2;
	}
	x1 = ((param.ac3)*b6)/8192;
	x2 = (param.b1 * (b6*b6/4096))/65536;
	x3 = ((x1 + x2) + 2)/4;
	b4 = param.ac4 * (uint32_t)(x3 + 32768)/32768;
	b7 = ((uint32_t)upress - b3)*(50000 >> oss);
	if (b7 < 0x80000000) {
		pressure = (b7*2)/b4;
	}
	else {
		pressure = (b7/b4)*2;
	}
	x1 = (pressure/256)*(pressure/256);
	x1 = (x1*3038)/65536;
	x2 = (-7357*pressure)/65536;
	pressure = pressure + (x1 + x2 + 3791)/16;

	error = sysctl_handle_int(oidp, &pressure, 0, req);
	if (error != 0 || req -> newptr == NULL) {
		return error;
	}
	return 0;

}
