CORE  := ../x48ng/src/core
LUA   := $(shell pkg-config --cflags lua5.4 2>/dev/null || echo -I/usr/include/lua5.4)

CC      := gcc
CFLAGS  := -std=c99 -O0 -g -Wall -Wextra \
           -I$(CORE) -I$(CORE)/.. -I. \
           $(LUA) \
           -DTEST_HARNESS=1

SRCS := $(CORE)/emulate.c \
        $(CORE)/registers.c \
        flat_memory.c \
        stubs.c \
        generate_tests.c

TARGET  := generate_tests
VECTORS := test_vectors.txt

all: $(VECTORS)

$(TARGET): $(SRCS) harness.h
	$(CC) $(CFLAGS) -o $@ $(SRCS)

$(VECTORS): $(TARGET)
	./$(TARGET) > $(VECTORS)
	@echo "Wrote $(VECTORS) ($$(wc -l < $(VECTORS)) lines)"

clean:
	rm -f $(TARGET) $(VECTORS) *.o

.PHONY: all clean
