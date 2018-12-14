include ../config

ROOTDIR=..
LIBDIR=$(ROOTDIR)/lib
INCLUDE=-I$(ROOTDIR)/include -I$(PREFIX)/include

CC=gcc
CFLAGS=-ansi -Wall -fPIC -O3 $(INCLUDE) -funroll-loops
LD=gcc
LDFLAGS=-shared
SIGNUP=echo "RoKi ver."$(VERSION)" Copyright (C) 1998 Tomomichi Sugihara (Zhidao)" >>

OBJ=rk_g3d.o rk_contact.o\
	rk_motor.o rk_motor_none.o rk_motor_trq.o rk_motor_dc.o\
	rk_force.o rk_body.o\
	rk_joint.o rk_joint_fixed.o rk_joint_revol.o rk_joint_prism.o rk_joint_cylin.o rk_joint_hooke.o rk_joint_spher.o rk_joint_float.o rk_joint_brfloat.o\
	rk_link.o rk_chain.o\
	rk_jacobi.o\
	rk_ik_cell.o rk_ik.o rk_ik_seq.o rk_ik_imp.o\
	rk_cd.o\
	rk_abi.o
DLIB=libroki.so

TARGET=$(DLIB)

all: $(TARGET)

$(DLIB): $(OBJ)
	@echo " LD	" $^
	@$(LD) $(LDFLAGS) -o $@ $^ > /dev/null; $(SIGNUP) $@
	-@mv $@ $(LIBDIR)
%.o: %.c
	@echo " CC	" $<
	@$(CC) $(CFLAGS) -c $< > /dev/null
clean:
	@echo " CLEAN"
	-@rm -f *.o *~ \#* $(TARGET)
