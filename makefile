#################################################
##
##	GLHL makefile
##		Author: Pablo Ramon Soria
##		Date:	2014-10-25
##
#################################################

release ?= false

APP_NAME 		= particleFilter

CXX = g++
CXX_FLAGS_BASE 		= -std=c++11 -Werror
CXX_RELEASE_FLAGS 	= -O3
CXX_DEBUG_FLAGS 	= -g

SRC_FILES 		:= $(shell find ./src/ -name '*.cpp')
OBJ_FILES 		:= $(patsubst %.cpp, %.o, $(SRC_FILES))

INCLUDE_DIR		:=	-I/usr/include \
				-I/home/juancarlos/programming/aruco-1.2.4/src \
				-I/home/juancarlos/programming/ardrone-pp/src
				
LIB_DIR			:=	-L/usr/lib	-L/home/juancarlos/programming/aruco-1.2.4/build/src
DEPENDENCIES		:=  -laruco -lardronepp -lopencv_core -lopencv_highgui -lopencv_imgproc -lopencv_calib3d

ifeq ($(release), false)
	CXX_FLAGS = $(CXX_FLAGS_BASE) $(CXX_DEBUG_FLAGS)
endif
ifeq ($(release), true)
	CXX_FLAGS = $(CXX_FLAGS_BASE) $(CXX_RELEASE_FLAGS)
endif


all: rebuild

run: 
	./$(APP_NAME)

rebuild: clean build

clean:
	rm -f $(OBJ_FILES)

build:	$(OBJ_FILES)
	@echo "----------------------------------------------"
	$(CXX) -o $(APP_NAME) $^ $(INCLUDE_DIR) $(LIB_DIR) $(DEPENDENCIES) 

%.o: %.cpp
	@echo "----------------------------------------------"
	echo "Compiling: " $^
	$(CXX) $(CXX_FLAGS) -c $^ -o $@ $(INCLUDE_DIR) $(LIB_DIR) $(DEPENDENCIES)  
