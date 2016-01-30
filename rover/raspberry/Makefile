SOURCES = robot.cpp I2Cdev.cpp MPU9150.cpp MadgwickAHRS.cpp PID.cpp bmp085.cpp io.cpp motors.cpp radio.cpp cam_gui.cpp camera.cpp PCA9685.cpp logic.cpp
PROGRAM = robot

CXX = ccache g++
CCFLAGS = -fdiagnostics-color=always -pipe -Ofast -mfpu=neon-vfpv4 -mfloat-abi=hard -mcpu=cortex-a7 -Wall -Wstrict-aliasing=0
INCLUDES = -pthread -lwiringPi -lwiringPiDev -lm -lrf24-bcm -lopencv_core -lopencv_imgproc -lopencv_highgui -l bcm2835
OBJ_DIR = obj/

BUILD_PRINT = \033[1;34mBuilding $<\033[0m
OBJS = ${SOURCES:%.cpp=%.o}

.SUFFIXES: .cpp .o

.cpp.o:
	@echo "$(BUILD_PRINT)"
	@${CXX} ${CCFLAGS} -c $< -o $(OBJ_DIR)$@ ${INCLUDES}

$(PROGRAM): ${OBJS}
	@echo "$(BUILD_PRINT)"
	@${CXX} ${CCFLAGS} ${OBJS:%=$(OBJ_DIR)%} -o $(PROGRAM) ${INCLUDES}
	
clean:
	rm -rf $(OBJ_DIR)*.o $(PROGRAM)