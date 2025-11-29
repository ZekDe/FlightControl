# EKF Quaternion Attitude Estimation Library
# Makefile

CC = gcc
CFLAGS = -Wall -Wextra -O2 -std=c99
LDFLAGS = -lm

# Source files
SRCS = matrix_math.c quaternion_math.c ekf_quat.c example_usage.c
OBJS = $(SRCS:.c=.o)

# Target
TARGET = ekf_demo

# Default target
all: $(TARGET)

$(TARGET): $(OBJS)
	$(CC) $(CFLAGS) -o $@ $^ $(LDFLAGS)

%.o: %.c
	$(CC) $(CFLAGS) -c -o $@ $<

# Dependencies
matrix_math.o: matrix_math.c matrix_math.h
quaternion_math.o: quaternion_math.c quaternion_math.h
ekf_quat.o: ekf_quat.c ekf_quat.h matrix_math.h quaternion_math.h
example_usage.o: example_usage.c ekf_quat.h quaternion_math.h matrix_math.h

# Clean
clean:
	rm -f $(OBJS) $(TARGET)

# Debug build
debug: CFLAGS += -g -DDEBUG
debug: $(TARGET)

# Run
run: $(TARGET)
	./$(TARGET)

.PHONY: all clean debug run
