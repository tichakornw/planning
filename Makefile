CC = g++
LD = g++

CXXFLAGS = -std=c++20 -Wall -g
LDFLAGS = -lm

INCLUDE_DIR = include
TEST_DIR = tests
SRC_DIR = src
EXAMPLE_DIR = examples
BUILD_DIR = build
TMP_DIR = tmp

DEP = $(wildcard $(INCLUDE_DIR)/*.h)
DEP_TEST = $(wildcard $(TEST_DIR)/*.h)
DEP_EXAMPLE = $(wildcard $(EXAMPLE_DIR)/*.h)
INC = -I$(INCLUDE_DIR)

# Main program
SRC = $(wildcard $(SRC_DIR)/*.cpp)
OBJ = $(SRC:$(SRC_DIR)/%.cpp=$(TMP_DIR)/%.o)
OBJ_CORE = $(filter-out $(TMP_DIR)/main.o, $(OBJ))
EXEC = $(BUILD_DIR)/planner

# Examples
EXAMPLE_SRC = $(wildcard $(EXAMPLE_DIR)/*.cpp)
EXAMPLE_MAIN_SRC = $(wildcard $(EXAMPLE_DIR)/main_*.cpp)
EXAMPLE_HELPER_SRC = $(filter-out $(EXAMPLE_MAIN_SRC), $(EXAMPLE_SRC))
EXAMPLE_EXEC = $(EXAMPLE_MAIN_SRC:$(EXAMPLE_DIR)/main_%.cpp=$(BUILD_DIR)/%)

OBJ_EXAMPLE = $(OBJ_CORE) \
               $(EXAMPLE_HELPER_SRC:$(EXAMPLE_DIR)/%.cpp=$(TMP_DIR)/%.o)

# Test executable
TEST_SRC = $(wildcard $(TEST_DIR)/*.cpp)
TEST_EXEC = $(BUILD_DIR)/test

# Include all example cpp files for linking into test executable
OBJ_TEST = $(OBJ_CORE) \
           $(TEST_SRC:$(TEST_DIR)/%.cpp=$(TMP_DIR)/%.o) \
           $(EXAMPLE_HELPER_SRC:$(EXAMPLE_DIR)/%.cpp=$(TMP_DIR)/%.o)

.SUFFIXES:

# --------------------------------------------------------------
ECHO = @printf "\033[1;32m==>\033[0m %s\n"

.PHONY: default
default: $(EXEC)

.PHONY: all
all: $(EXEC) test examples

# Main executable
$(EXEC): $(OBJ)
	@mkdir -p $(BUILD_DIR)
	$(ECHO) "Building main executable: $@"
	@$(LD) $^ -o $@ $(LDFLAGS)

# Test executable.
PHONY: test
test: $(TEST_EXEC)
$(TEST_EXEC): $(OBJ_TEST)
	@mkdir -p $(BUILD_DIR)
	$(ECHO) "Building test executable: $@"
	@$(LD) $^ -o $@ $(LDFLAGS)

# Example executables
.PHONY: examples
examples: $(EXAMPLE_EXEC)
$(BUILD_DIR)/%: $(EXAMPLE_DIR)/main_%.cpp $(OBJ_EXAMPLE)
	@mkdir -p $(BUILD_DIR)
	$(ECHO) "Building example: $@"
	@$(CC) $(CXXFLAGS) -o $@ $^ $(INC) $(LDFLAGS)

# --------------------------------------------------------------
# Object compilation rules
$(TMP_DIR)/%.o: $(SRC_DIR)/%.cpp $(DEP)
	@mkdir -p $(TMP_DIR)
	@$(CC) $(CXXFLAGS) -c $< $(INC) -o $@ -MMD -MP

$(TMP_DIR)/%.o: $(TEST_DIR)/%.cpp $(DEP) $(DEP_TEST)
	@mkdir -p $(TMP_DIR)
	@$(CC) $(CXXFLAGS) -c $< $(INC) -o $@ -MMD -MP

$(TMP_DIR)/%.o: $(EXAMPLE_DIR)/%.cpp $(DEP) $(DEP_EXAMPLE)
	@mkdir -p $(TMP_DIR)
	@$(CC) $(CXXFLAGS) -c $< $(INC) -o $@ -MMD -MP

# --------------------------------------------------------------
# Clean
.PHONY: clean clear
clean clear:
	@rm -rf $(BUILD_DIR)/*
	@rm -rf $(TMP_DIR)/*
