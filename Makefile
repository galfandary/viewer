CXXFLAGS:=-std=c++11 -O -s -Wall -pedantic -Wno-misleading-indentation -DUSE_GLU
LDLIBS:=-lGL -lX11 -lGLU
LINK.o:=$(LINK.cc)
.PHONY: all
all: viewer
viewer.o: 3ds.hpp collide.hpp util.hpp xwin.hpp
