#CPPFLAGS:= -std=c++11 -static -fno-optimize-sibling-calls -fno-strict-aliasing -DONLINE_JUDGE -D_LINUX -lm -s -x c++ -O2 -Wall -Wno-unknown-pragmas
CPPFLAGS:= -std=c++11 -fno-optimize-sibling-calls -fno-strict-aliasing -DONLINE_JUDGE -D_LINUX -g -O0 -Wall -Wno-unknown-pragmas
cpps:=$(shell find -name '*.cpp')
objs:=$(patsubst %.cpp, %.o, $(cpps))
progname:=MyStrategy

.PHONY: all clean
all: $(progname)

clean:
	rm -f $(objs)

$(progname): $(objs)
	g++ $^ -o $@

