include ../config

CONFIG=roki-config
BINDIR=$(PREFIX)/bin

INCLUDE=`$(CONFIG) -I`
LIB=`$(CONFIG) -L`
DEF=`$(CONFIG) -D`
LINK=`$(CONFIG) -l`

CC=gcc
CFLAGS=-ansi -Wall -O3 $(INCLUDE) $(LIB) $(DEF)

COMPILE=$(CC) $(CFLAGS) -o $@ $< $(LINK)
INSTALL=install -m 755
UNINSTALL=rm -f

TARGET=mshape2chain chain2mshape zvs2zkci zvs2zkcs\
	rk_ground rk_mp rk_fk rk_ik rk_id rk_ikcell_merge\
	urdf2ztk

#####################################################################
all : $(TARGET)

%: %.c
	@echo " CC	" $<
	@$(COMPILE)
clean:
	@echo " CLEAN"
	-@rm -f *.o *~ core $(TARGET)
install: $(TARGET)
ifneq ($(TARGET),)
	@echo " INSTALL	" $^
	@$(INSTALL) $^ $(BINDIR)
endif
uninstall:
ifneq ($(TARGET),)
	@echo " UNINSTALL	" $(TARGET)
	@$(UNINSTALL) $(BINDIR)/$(TARGET)
endif
