# change the following .PATH
.PATH:		/home/che/git/bmp085-driver/
KMOD		= bmp085
SRCS		= bmp085.c bmp085.h device_if.h bus_if.h iicbus_if.h

.include <bsd.kmod.mk>
