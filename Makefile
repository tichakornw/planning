CC = g++
LD = g++

CXXFLAGS = -std=c++20 -Wall -g
LDFLAGS = -lm

INCLUDE_DIR = include
TEST_DIR = tests
SRC_DIR = src
BUILD_DIR = build
TMP_DIR = tmp

DEP = $(wildcard $(INCLUDE_DIR)/*.h)
DEP_TEST = $(wildcard $(TEST_DIR)/*.h)
INC = -I$(INCLUDE_DIR)

EXEC = $(BUILD_DIR)/planner
SRC = $(wildcard $(SRC_DIR)/*.cpp)
OBJ = $(SRC:$(SRC_DIR)/%.cpp=$(TMP_DIR)/%.o)

EXEC_TEST = $(BUILD_DIR)/test
SRC_TEST = $(wildcard $(TEST_DIR)/*.cpp)
OBJ_TEST = $(filter-out $(TMP_DIR)/main.o, $(OBJ)) $(SRC_TEST:$(TEST_DIR)/%.cpp=$(TMP_DIR)/%.o)

.SUFFIXES:

# --------------------------------------------------------------

.PHONY: all
all: $(EXEC)
$(EXEC): $(OBJ)
	@$(LD) $^ -o $@ $(LDFLAGS)

# --------------------------------------------------------------

.PHONY: test
test: $(EXEC_TEST)
$(EXEC_TEST): $(OBJ_TEST)
	@$(LD) $^ -o $@ $(LDFLAGS)

# --------------------------------------------------------------

$(TMP_DIR)/%.o: $(SRC_DIR)/%.cpp $(DEP)
	@mkdir -p $(TMP_DIR)
	@$(CC) $(CXXFLAGS) -c $< $(INC) -o $@ -MMD -MP

$(TMP_DIR)/%.o: $(TEST_DIR)/%.cpp $(DEP) $(DEP_TEST)
	@mkdir -p $(TMP_DIR)
	@$(CC) $(CXXFLAGS) -c $< $(INC) -o $@ -MMD -MP

# --------------------------------------------------------------
.PHONY: clean, clear
clean clear:
	@rm -f $(BUILD_DIR)/*
	@rm -f $(TMP_DIR)/*
