# Set a variable for commands below
set(PROJECT_NAME remote)

# Define your project and language
project(${PROJECT_NAME} C CXX)

set(${PROJECT_NAME}_BOARD nano)
set(${PROJECT_NAME}_BOARD_CPU atmega328old)
set(nano.build.mcu "atmega328p")

# Define the source code
set(${PROJECT_NAME}_SRCS main.cpp)

# Define the port for uploading code to the Arduino
set(${PROJECT_NAME}_PORT COM4)

set(Servo_RECURSE True)

# Command to generate code arduino firmware (.hex file)
generate_arduino_firmware(${PROJECT_NAME})
