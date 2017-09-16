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
static int bmp085_temp_sysctl(SYSCTL_HANDLER_ARGS);
// static int bmp085_pressure_sysctl(SYSCTL_HANDLER_ARGS);

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

static int bmp085_attach(device_t dev) {
	struct bmp085_softc *sc;

	sc = device_get_softc(dev);
	sc -> sc_dev = dev;
	sc -> sc_addr = iicbus_get_addr(dev);

	sc -> enum_hook.ich_func = bmp085_start;
	sc -> enum_hook.ich_arg = dev;

	if (config_intrhook_establish(&sc->enum_hook) != 0) {
		return ENOMEM;
	}
	return 0;
}

static void bmp085_start(void *xdev) {
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

	config_interhook_disestablish(&sc->enum_hook);

	SYSCTL_ADD_PROC(ctx, tree, OID_AUTO, "temperature",
			CTLTYPE_INT | CTLFLAG_RD | CTLFLAG_MPSAFE, dev, BMP085_TEMP,
			bmp085_temp_sysctl, "IK", "Current temperature");
	// SYSCTL_ADD_PROC(ctx, tree, OID_AUTO, "pressure",
	// 		CTLTYPE_INT | CTLFLAG_RD | CLTFLAG_MPSAFE, dev, BMP085_PRESS,
	// 		bmp085_pressure_sysctl, "IK", "Current athmospheric pressure");
	// now we're just setting it up
	uint8_t buffer_tx;
	uint8_t buffer_rx[2];

	buffer_tx = BMP_AC1;
	if (bmp085_read(sc->sc_dev, sc->sc_addr, buffer_tx, buffer_tx, 2*sizeof(uint8_t)) < 0) {
		device_printf("cannot write to sensor\n");
		return;
	}
	param.ac1 = ((buffer_rx[0] << 8) | buffer_rx[1]);

	buffer_tx = BMP_AC2;
	if (bmp085_read(sc->sc_dev, sc->sc_addr, buffer_tx, buffer_tx, 2*sizeof(uint8_t)) < 0) {
		device_printf("cannot write to sensor\n");
		return;
	}
	param.ac2 = ((buffer_rx[0] << 8) | buffer_rx[1]);

	buffer_tx = BMP_AC3;
	if (bmp085_read(sc->sc_dev, sc->sc_addr, buffer_tx, buffer_tx, 2*sizeof(uint8_t)) < 0) {
		device_printf("cannot write to sensor\n");
		return;
	}
	param.ac3 = ((buffer_rx[0] << 8) | buffer_rx[1]);

	buffer_tx = BMP_AC4;
	if (bmp085_read(sc->sc_dev, sc->sc_addr, buffer_tx, buffer_tx, 2*sizeof(uint8_t)) < 0) {
		device_printf("cannot write to sensor\n");
		return;
	}
	param.ac4 = ((buffer_rx[0] << 8) | buffer_rx[1]);

	buffer_tx = BMP_AC5;
	if (bmp085_read(sc->sc_dev, sc->sc_addr, buffer_tx, buffer_tx, 2*sizeof(uint8_t)) < 0) {
		device_printf("cannot write to sensor\n");
		return;
	}
	param.ac5 = ((buffer_rx[0] << 8) | buffer_rx[1]);

	buffer_tx = BMP_AC6;
	if (bmp085_read(sc->sc_dev, sc->sc_addr, buffer_tx, buffer_tx, 2*sizeof(uint8_t)) < 0) {
		device_printf("cannot write to sensor\n");
		return;
	}
	param.ac6 = ((buffer_rx[0] << 8) | buffer_rx[1]);

	buffer_tx = BMP_B1;
	if (bmp085_read(sc->sc_dev, sc->sc_addr, buffer_tx, buffer_tx, 2*sizeof(uint8_t)) < 0) {
		device_printf("cannot write to sensor\n");
		return;
	}
	param.b1 = ((buffer_rx[0] << 8) | buffer_rx[1]);

	buffer_tx = BMP_B2;
	if (bmp085_read(sc->sc_dev, sc->sc_addr, buffer_tx, buffer_tx, 2*sizeof(uint8_t)) < 0) {
		device_printf("cannot write to sensor\n");
		return;
	}
	param.b2 = ((buffer_rx[0] << 0) | buffer_rx[1]);

	buffer_tx = BMP_MB;
	if (bmp085_read(sc->sc_dev, sc->sc_addr, buffer_tx, buffer_tx, 2*sizeof(uint8_t)) < 0) {
		device_printf("cannot write to sensor\n");
		return;
	}
	param.mb = ((buffer_rx[0] << 0) | buffer_rx[1]);

	buffer_tx = BMP_MC;
	if (bmp085_read(sc->sc_dev, sc->sc_addr, buffer_tx, buffer_tx, 2*sizeof(uint8_t)) < 0) {
		device_printf("cannot write to sensor\n");
		return;
	}
	param.mc = ((buffer_rx[0] << 0) | buffer_rx[1]);

	buffer_tx = BMP_MD;
	if (bmp085_read(sc->sc_dev, sc->sc_addr, buffer_tx, buffer_tx, 2*sizeof(uint8_t)) < 0) {
		device_printf("cannot write to sensor\n");
		return;
	}
	param.md = ((buffer_rx[0] << 0) | buffer_rx[1]);
	
}

static int bmp085_temp_sysctl(SYSCTL_HANDLER_ARGS) {
	device_t dev;
	struct bmp085_softc *sc;

	dev = (device_t)arg1;
	sc = device_get_softc(dev);

	int32_t utemp;
	int32_t x1, x2;
	int32_t temperature = 0;
	uint8_t buffer_tx[2];
	uint8_t buffer_rx[2];

	buffer_tx[0] = BMP_CR;
	buffer_tx[1] = BMP_MODE_TEMP;
	if (bmp085_write(sc->sc_dev, BMP_CR, &buffer_tx[1], 1) != 0) {
		return EIO;
	}
	if (bmp085_read(sc->sc_dev, BMP_DATA, buffer_rx, 2*sizeof(uint8_t)) != 0) {
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


