CC = gcc
CFLAGS = -shared -Wall -Wextra -Wl,-soname,kalman -fPIC -O2

kalman.so: kalman.c kalman.h
	$(CC) $(CFLAGS) $< -lm -o $@

.PHONY: clean
clean:
	rm -f kalman.so
