all: ../obj/client.o ../obj/matrix.o ../obj/kinematics.o
	gcc -Wall -lm ../obj/kinematics.o ../obj/matrix.o ../../Raspberry/obj/rasp_uart.o ../obj/client.o -o ../bin/client.elf

../obj/%.o: %.c
	gcc -Wall -c -I../../Raspberry/include -I../include $< -o $@

.PHONY: clean
clean:
	rm -rf $(wildcard ../obj/* ../bin/*)
