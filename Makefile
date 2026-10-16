CXX = clang++
CXXFLAGS = -std=c++23 -O3 -flto -Isrc -Wall -Wextra -Wpedantic -Wconversion -Wsign-conversion -Wshadow -Wold-style-cast \
	-Wcast-qual -Wnull-dereference -Wdouble-promotion -Wformat=2 -Wimplicit-fallthrough -Wundef -Wextra-semi \
	-Wunused -Wvla -Wzero-as-null-pointer-constant
ifdef SANITIZE
CXXFLAGS += -g -fsanitize=address,undefined -fno-sanitize-recover=all -fno-omit-frame-pointer
endif
HEADERS = $(wildcard src/*.h)

all: build/leapster

build/leapster: src/main.cpp $(HEADERS)
	@mkdir -p build
	$(CXX) $(CXXFLAGS) $(shell pkg-config --cflags sdl2) -o $@ $< $(shell pkg-config --libs sdl2)

clean:
	rm -rf build

.PHONY: all clean
