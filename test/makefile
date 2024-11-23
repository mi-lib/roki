CFLAGS=`roki-config --cflags`
LINK=`roki-config -l`

CC=gcc
CFLAGS+=-ansi -Wall -O3 $(LIB) $(INCLUDE) -funroll-loops

%: %.c
	@$(CC) $(CFLAGS) -o $@ $< $(LINK)
clean :
	-@rm -f *.o *~ core *test
