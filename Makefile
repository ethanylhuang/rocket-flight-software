CC = gcc
CFLAGS = -Iinclude -Wall -Wextra -O2
LDFLAGS = -lm

SRC = src/main.c src/sensors.c src/estimator.c src/utils.c
OBJ = $(SRC:.c=.o)
TARGET = rocket_sim

all: $(TARGET)

$(TARGET): $(OBJ)
	$(CC) $(OBJ) -o $(TARGET) $(LDFLAGS)

%.o: %.c
	$(CC) $(CFLAGS) -c $< -o $@

clean:
	rm -f src/*.o $(TARGET)

.PHONY: all clean
