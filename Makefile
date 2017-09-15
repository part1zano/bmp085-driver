# $FreeBSD: stable/11/sys/modules/i2c/cyapa/Makefile 319182 2017-05-30 04:11:12Z ngie $

.PATH:		/home/che/git/bmp085-driver/
KMOD		= bmp085
SRCS		= bmp085.c bmp085.h device_if.h bus_if.h iicbus_if.h

.include <bsd.kmod.mk>
