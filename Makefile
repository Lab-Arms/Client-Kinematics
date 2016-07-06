CC=gcc

INCLUDE_DIR=./include
BIN_DIR=./bin
OBJ_DIR=./obj
SRC_DIR=./src

CFLAGS=-c -Wall -I$(INCLUDE_DIR)
LDFLAGS=

SOURCES=
OBJECTS=$(SOURCES:.c=.o)

all:
	@echo -e "\nCompilando os subprojetos.\n"
	@for dir in $(SRC_DIR) ; do \
		(cd $$dir ; ${MAKE} all) \
	done
	@echo -e "\nCompilação dos subprojetos terminada.\n"

.PHONY: clean
clean:
	@echo -e "\nLimpando os objetos e binários de todos os subprojetos.\n"
	@for dir in $(SRC_DIR) ; do \
		(cd $$dir ; ${MAKE} clean) \
	done
	@echo -e "\nLimpeza finalizada.\n"
