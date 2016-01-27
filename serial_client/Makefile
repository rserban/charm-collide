OPTS=-verbose
CHARMDIR=/home/radu/charm/multicore-linux32
CHARMC=$(CHARMDIR)/bin/charmc $(OPTS)


HEADERS=collide_util.h bbox.h collide_cfg.h collide_buffers.h collidecharm.h
HEADDEP=$(HEADERS) collidecharm_impl.h collide_serial.h collidecharm.decl.h

COBJS=collide_util.o collide_serial.o collidecharm.o collide_buffers.o 

#---------------------------------------------------------------------------
# This does not work:
# Code builds, but gives runtime errors

all: INTERFACE.collidecharm $(COBJS) INTERFACE.hello hello.o
	$(CHARMC) -language charm++ -o hello hello.o $(COBJS)

#---------------------------------------------------------------------------
# This works fine:
# Uses the libmodulecollidecharm from the charm++ build

hello: INTERFACE.hello hello.o
	$(CHARMC) -language charm++ -module collidecharm -o hello hello.o

#---------------------------------------------------------------------------

collide_util.o: collide_util.C $(HEADDEP)
	$(CHARMC) -c collide_util.C

collide_serial.o: collide_serial.C $(HEADDEP)
	$(CHARMC) -c collide_serial.C

collidecharm.o: collidecharm.C $(HEADDEP)
	$(CHARMC) -c collidecharm.C

collide_buffers.o: collide_buffers.C $(HEADDEP)
	$(CHARMC) -c collide_buffers.C

collidecharm.decl.h collidecharm.def.h: INTERFACE.collidecharm

INTERFACE.collidecharm: collidecharm.ci
	$(CHARMC) collidecharm.ci
	touch $@

call_init.o: call_init.c
	$(CHARMC) -c call_init.c

#---------------------------------------------------------------------------

INTERFACE.hello: hello.ci
	$(CHARMC) hello.ci

hello.o: hello.C
	$(CHARMC) -c hello.C

#---------------------------------------------------------------------------

clean:
	rm -rf *.a *.def.h *.decl.h *.o SunWS_cache $(CLIB).dep headers INTERFACE.*
	rm -f *.decl.h *.def.h conv-host *.o hello charmrun 

