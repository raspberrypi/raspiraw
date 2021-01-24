CROSS_COMPILE ?=

CC	?= gcc
CC	:= $(CROSS_COMPILE)$(CC)
CFLAGS	?= -I/opt/vc/include -pipe -W -Wall -Wextra -g -O0 -MD
LDFLAGS	?=
LIBS	:= -L/opt/vc/lib -lrt -lbcm_host -lvcos -lmmal_core -lmmal_util -lmmal_vc_client -lvcsm -lpthread

%.o : %.c
	$(CC) $(CFLAGS) -c -o $@ $<

all: raspiraw

OBJS := raspiraw.o RaspiCLI.o
raspiraw: $(OBJS)
	$(CC) $(LDFLAGS) -o $@ $^ $(LIBS)

clean:
	-rm -f *.o *.d
	-rm -f raspiraw

-include $(OBJS:.o=.d)
