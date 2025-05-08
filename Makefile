CXX = g++
CXXFLAGS = -Wall -Wextra -g -std=c++17
INC_DIRS = include /usr/local/include
LIB_DIRS = build/lib /usr/local/lib
LDFLAGS = -Lbuild/lib -lzmq

SRC_DIR = src
INCLUDE_DIR = include
LIB_SRC_DIR = libsrc
BUILD_LIB_DIR = build/lib
OBJ_DIR = build/obj
BIN_DIR  = build/bin

# External header-only libraries
EXT_INC_DIRS = /usr/local/include

# External compiled libraries
EXT_LIBS = -lzmq

# Executable targets
TARGETS = $(BIN_DIR)/zmq_server $(BIN_DIR)/zmq_client

# Source files for each target
zmq_server_SRC = $(SRC_DIR)/zmq_server.cpp
zmq_client_SRC = $(SRC_DIR)/zmq_client.cpp

# Library source files
LIB_SRC_FILES := $(wildcard $(LIB_SRC_DIR)/*.cpp)

# Object files for library
LIB_SRC_OBJS := $(patsubst $(LIB_SRC_DIR)/%.cpp,$(OBJ_DIR)/%.o,$(LIB_SRC_FILES))

# All source files for executables
SRC_FILES := $(zmq_server_SRC) $(zmq_client_SRC)
# Object files for executables
SRC_OBJS := $(patsubst $(SRC_DIR)/%.cpp,$(OBJ_DIR)/%.o,$(SRC_FILES))

# All object files
OBJS := $(SRC_OBJS) $(LIB_SRC_OBJS)

# Add external include directories
CXXFLAGS += $(addprefix -I,$(INC_DIRS) $(EXT_INC_DIRS))
LDFLAGS += $(addprefix -L,$(LIB_DIRS)) $(EXT_LIBS)

all: $(TARGETS)

# Create directories
$(OBJ_DIR):
	mkdir -p $(OBJ_DIR)

$(BUILD_LIB_DIR):
	mkdir -p $(BUILD_LIB_DIR)

$(BIN_DIR):
	mkdir -p $(BIN_DIR)

# Rule to build executables
$(BIN_DIR)/%: $(OBJ_DIR)/%.o $(BUILD_LIB_DIR)/libfunc.a | $(BIN_DIR)
	$(CXX) $(CXXFLAGS) -o $@ $< $(BUILD_LIB_DIR)/libfunc.a $(LDFLAGS)
	install_name_tool -change @rpath/libzmq.5.dylib /usr/local/lib/libzmq.5.dylib $@ # macOS

# Rule to compile source files from SRC_DIR into object files
$(OBJ_DIR)/%.o: $(SRC_DIR)/%.cpp | $(OBJ_DIR)
	$(CXX) $(CXXFLAGS) -c $< -o $@

# Rule to compile source files from LIB_SRC_DIR into object files
$(OBJ_DIR)/%.o: $(LIB_SRC_DIR)/%.cpp $(INCLUDE_DIR)/test.h | $(OBJ_DIR)
	$(CXX) $(CXXFLAGS) -c $< -o $@

# Rule to build the static library
$(BUILD_LIB_DIR)/libfunc.a: $(OBJ_DIR)/func.o | $(BUILD_LIB_DIR)
	ar rcs $(BUILD_LIB_DIR)/libfunc.a $(OBJ_DIR)/func.o

clean:
	rm -rf build

.PHONY: all clean