CC ?= gcc
CFLAGS ?= -O2 -std=c11 -Wall -Isrc
ifeq ($(OS),Windows_NT)
LDFLAGS ?= -mwindows -luser32 -lgdi32 -lwinmm
TARGET ?= game.exe
else
LDFLAGS ?= -lm
TARGET ?= game.exe
endif

SRCS = src/game_recomp.c src/snes_runtime.c src/snes_main.c
OBJS = $(SRCS:.c=.o)

all: $(TARGET)

$(TARGET): $(OBJS)
	$(CC) $(OBJS) -o $(TARGET) $(LDFLAGS)

%.o: %.c
	$(CC) $(CFLAGS) -c $< -o $@

clean:
	rm -f $(OBJS) $(TARGET)
