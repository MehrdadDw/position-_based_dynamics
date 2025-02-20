makefile:
all: simulation

simulation:
	g++ pbd.cpp -o pbd -lsfml-graphics -lsfml-window -lsfml-system

clean:
	-rm pbd