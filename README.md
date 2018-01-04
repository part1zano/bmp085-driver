# bmp085 driver for FreeBSD

A driver for the BMP085 barometer/thermometer

## How to install
 * Get the FreeBSD source, e.g. `svn co https://svn0.eu.freebsd.org/base/stable/11`
 * Create the appropriate directory in `sys/modules/i2c`: `mkdir sys/modules/i2c/bmp085`
 * Edit the Makefile appropriately (e.g. change the path) and place it to the created directory
 * Apply the bmp085.patch: `patch -p0 < /path/to/bmp085-driver/bmp085.patch`
 * Apply the dts patch: `patch -p0 < /path/to/rpi-dts.patch`
 * Build the toolchain and the kernel, e.g. for Raspberry Pi do the following: `make TARGET_ARCH=armv6 KERNCONF=RPI2 kernel-toolchain buildkernel`
 * Build the module itself: `make TARGET_ARCH=armv6 KERNFAST=RPI2 OVERRIDE_MODULES=i2c/bmp085 buildkernel`
 * Copy the dtb to `/boot/dtb` as well as to `boot/msdos/` (rpi only): `cp /usr/obj/<your freebsd src dir>/sys/RPI2/modules/<your freebsd src dir>sys/modules/dtb/rpi/rpi2.dtb /path/to/boot/dtb`
 * Copy the resulting `.ko` to the appropriate directory: `cp sys/dev/i2c/bmp085.ko /path/to/rpi-boot`
 * On the Raspberry Pi, use `kldload` to use it: `kldload bmp085`
 * Use it! View `sysctl dev.bmp085` to see the temperature and pressure.

## What does it do?
It measures the current athmospheric pressure and the current temperature putting the results into the appropriate sysctls.

## What doesn't it do (yet)?
 * Wasn't tested with hardware besides the `bmp085`. In fact, should work with `bmp180` and `bme280` just as fine.
