
CC=gcc
CFLAGS=-Wall -g
LIBS += -lm
test: ga-test.c
	$(CC) $(CFLAGS) ../heuristics_ga.c ../../src/robc_heuristics.c ga-test.c  -o test.bin $(LIBS)

	
.PHONY: clean
clean:
	rm -rf *.o *.bin

