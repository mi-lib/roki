CC=gcc
CFLAGS=-ansi -Wall -O3 -funroll-loops -g `roki-config --cflags`
LINK=`roki-config -l`

TARGET=$(shell ls *.c | xargs -i basename {} .c | tr -s "\n" " ")

all: $(TARGET)
%: %.c
	$(CC) $(CFLAGS) -o $@ $< $(LINK)
clean :
	-@rm -f *.o *~ core *test $(TARGET)
