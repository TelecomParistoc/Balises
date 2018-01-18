SRC := main.c doc_generator.c header_generator.c interface_parser.c interface_generator.c utils.c
OBJ := $(SRC:%.c=%.o)
CC := gcc
CFLAGS += -Wall -Wextra
LFLAGS += -lxml2 -L /usr/include

all: xml

xml: $(OBJ)
	gcc -o $@ $^ $(LFLAGS)

clean:
	rm -f *.o
	rm xml
