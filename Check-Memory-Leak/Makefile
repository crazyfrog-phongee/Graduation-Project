TITLE_COLOR = \033[33m
NO_COLOR = \033[0m
FLAGS = -Wall -std=c11 -Werror -fdiagnostics-color=auto -lm
DEBUG_LVL = 2

# when excuting make, compile all exe's
all: clean-all exec_file

# When trying to compile one of the executables, first look for its .c files
# Then check if the libraries are in the lib folder
exec_file: main.c common.c
	@echo "$(TITLE_COLOR)\n***** CPPCHECK *****$(NO_COLOR)"
	cppcheck --enable=all --suppress=missingIncludeSystem main.c common.c
	@echo "$(TITLE_COLOR)\n***** COMPILING sensor_gateway *****$(NO_COLOR)"
	gcc -c -g main.c       	-o main.o      	$(FLAGS)
	gcc -c -g common.c    	-o common.o   	$(FLAGS)
	@echo "$(TITLE_COLOR)\n***** LINKING exec_file *****$(NO_COLOR)"
	gcc -g main.o common.o -lpthread -o ./bin/exec_file -Wall -fdiagnostics-color=auto -lm

# do not look for files called clean, clean-all or this will be always a target
.PHONY: clean clean-all 

clean:
	rm -rf ./bin/*

clean-all: clean
	rm -rf *.o *~ 

leak: all
	@echo "$(TITLE_COLOR)\n***** LEAK CHECK sensor_gateway *****$(NO_COLOR)"
	valgrind --leak-check=full -v --track-origins=yes --show-leak-kinds=all ./bin/exec_file

run:
	@echo "$(TITLE_COLOR)\n***** RUNNING sensor_gateway *****$(NO_COLOR)"
	./bin/exec_file