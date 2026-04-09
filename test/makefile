ROKI_CFLAGS=`roki-config --cflags`
LINK=`roki-config -l`
LINKCXX=`roki-config -lcpp`

CC=gcc
CFLAGS=-ansi -Wall -O3 -funroll-loops ${ROKI_CFLAGS}

CXX=g++
CXXFLAGS=-std=c++17 -Wall -O3 -funroll-loops ${ROKI_CFLAGS}

%: %.c
	@$(CC) $(CFLAGS) -o $@ $< $(LINK)
%: %.cpp
	@$(CXX) $(CXXFLAGS) -o $@ $< $(LINKCXX)
clean :
	-@rm -f *.o *~ core *test
