TARGET:=a.out 
C_FILES:=$(wildcard src/*.cpp)
H_FILES:=$(wildcard include/*.h) $(wildcard include/*.hpp)

FLAGS:=-Wall -g -lm
INCLUDE:=-I include `sdl2-config --cflags --libs` 

$(TARGET): $(C_FILES) $(H_FILES)
	g++ $(FLAGS) -o $@ $(C_FILES) $(INCLUDE)

.PHONY: clean
clean:
	rm $(TARGET)


.PHONY: run
run:
	./$(TARGET)
