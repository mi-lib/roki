# default suffices
# .z3d : 3D shape model
# .zkc : kinematic chain model
# .zkci: initial state of a kinematic chain
# .zvs : vector sequence
# .zkcs: kinematic chain configuration sequence
# .zef : external force info.

include ../config

INCLUDE=-I. -I../include -I$(PREFIX)/include
LIB=-L../lib -L$(PREFIX)/lib
BINDIR=$(PREFIX)/bin

CC=gcc
CFLAGS=-ansi -Wall -O3 $(INCLUDE) $(LIB)

LINK=-lroki `zeo-config -l`

COMPILE=$(CC) $(CFLAGS) -o $@ $< $(LINK)

INSTALL=install -m 755

CONFIGGEN=zeda-config-gen
CONFIG=roki-config
TARGET=$(CONFIG) cad2zkc zkcconv z3d2zkc zkc2z3d z3d2gp zvs2zkci zvs2zkcs\
	rk_ground rk_mp rk_fk rk_ik rk_id rk_ikcell_merge

#####################################################################
all : $(TARGET)

$(CONFIG):
	@echo " GENERATE" $@
	-@rm -f $@
	@$(CONFIGGEN) -I $(PREFIX)/include -L $(PREFIX)/lib -l "$(LINK)" -v $(VERSION) > $@

%: %.c
	@echo " CC	" $<
	@$(COMPILE)
install: $(TARGET)
	@echo " INSTALL	" $^
	@$(INSTALL) $^ $(BINDIR)
clean :
	@echo " CLEAN"
	-@rm -f *.o *~ core $(TARGET)
