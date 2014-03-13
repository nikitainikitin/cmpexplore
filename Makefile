CC = g++
CFLAGS = -O2 -fomit-frame-pointer -fforce-addr -fexpensive-optimizations -falign-functions=32 -falign-loops=32
#CFLAGS = -O3 -ffast-math
#CFLAGS = -gdwarf-2
#CFLAGS = -pg -O2 -fforce-addr -fexpensive-optimizations -falign-functions=32 -falign-loops=32
#CFLAGS = -g
INC_DIR = ./include
SRC_DIR = ./src
OBJ_DIR = ./obj
OBJ_TARGET = cmpexplore

NM_INC = -I./newmat10
NM_LIB = -L./newmat10 -lnewmat
INCLUDE = -I./include -I./include/ggraph -I./include/cmp -I./include/model \
        -I./include/arch -I./include/explore -I./include/stat -I./include/perf \
        -I./include/phys -I./include/power -I../ggraph/include $(NM_INC)

OBJECTS = \
	$(OBJ_DIR)/ggraph/GVertex.o \
	$(OBJ_DIR)/ggraph/GDEdge.o \
	$(OBJ_DIR)/ggraph/GGraph.o \
	$(OBJ_DIR)/ggraph/Timer.o \
	$(OBJ_DIR)/cmp/Component.o \
	$(OBJ_DIR)/cmp/Device.o \
	$(OBJ_DIR)/cmp/Processor.o \
	$(OBJ_DIR)/cmp/Memory.o \
	$(OBJ_DIR)/cmp/Cluster.o \
	$(OBJ_DIR)/cmp/Interconnect.o \
	$(OBJ_DIR)/cmp/CmpBuilder.o \
	$(OBJ_DIR)/cmp/CmpConfig.o \
	$(OBJ_DIR)/cmp/MeshIcTile.o \
	$(OBJ_DIR)/cmp/MeshIcLink.o \
	$(OBJ_DIR)/cmp/MeshIc.o \
	$(OBJ_DIR)/cmp/BusIc.o \
	$(OBJ_DIR)/cmp/URingIc.o \
	$(OBJ_DIR)/cmp/BRingIc.o \
	$(OBJ_DIR)/cmp/XBarIc.o \
	$(OBJ_DIR)/model/RouterModel.o \
	$(OBJ_DIR)/model/BusModel.o \
	$(OBJ_DIR)/model/Function.o \
	$(OBJ_DIR)/arch/ArchConfig.o \
	$(OBJ_DIR)/arch/ParamIterator.o \
        $(OBJ_DIR)/arch/ArchPlanner.o \
	$(OBJ_DIR)/explore/ExplConf.o \
	$(OBJ_DIR)/explore/ExplEngine.o \
	$(OBJ_DIR)/explore/ExhEngine.o \
	$(OBJ_DIR)/explore/EoEngine.o \
	$(OBJ_DIR)/explore/SaEngine.o \
	$(OBJ_DIR)/explore/HcEngine.o \
        $(OBJ_DIR)/explore/RbEngine.o \
	$(OBJ_DIR)/stat/StatConfig.o \
	$(OBJ_DIR)/stat/Statistics.o \
	$(OBJ_DIR)/perf/CmpAnalytPerfModel.o \
	$(OBJ_DIR)/perf/IterativePerfModel.o \
	$(OBJ_DIR)/perf/MatlabPerfModel.o \
	$(OBJ_DIR)/phys/PhysicalModel.o \
	$(OBJ_DIR)/power/PowerModel.o \
	$(OBJ_DIR)/Util.o \
	$(OBJ_DIR)/Config.o \
	$(OBJ_DIR)/Debug.o \
	$(OBJ_DIR)/Parser.o \
	$(OBJ_DIR)/main.o

$(OBJ_DIR)/%.o: $(SRC_DIR)/%.cpp
	$(CC) $(CFLAGS) $(INCLUDE) -c $< -o $@

$(OBJ_DIR)/ggraph/%.o: $(SRC_DIR)/ggraph/%.cpp
	$(CC) $(CFLAGS) $(INCLUDE) -c $< -o $@

$(OBJ_DIR)/cmp/%.o: $(SRC_DIR)/cmp/%.cpp
	$(CC) $(CFLAGS) $(INCLUDE) -c $< -o $@

$(OBJ_DIR)/model/%.o: $(SRC_DIR)/model/%.cpp
	$(CC) $(CFLAGS) $(INCLUDE) -c $< -o $@

$(OBJ_DIR)/arch/%.o: $(SRC_DIR)/arch/%.cpp
	$(CC) $(CFLAGS) $(INCLUDE) -c $< -o $@

$(OBJ_DIR)/explore/%.o: $(SRC_DIR)/explore/%.cpp
	$(CC) $(CFLAGS) $(INCLUDE) -c $< -o $@

$(OBJ_DIR)/stat/%.o: $(SRC_DIR)/stat/%.cpp
	$(CC) $(CFLAGS) $(INCLUDE) -c $< -o $@

$(OBJ_DIR)/perf/%.o: $(SRC_DIR)/perf/%.cpp
	$(CC) $(CFLAGS) $(INCLUDE) -c $< -o $@

$(OBJ_DIR)/phys/%.o: $(SRC_DIR)/phys/%.cpp
	$(CC) $(CFLAGS) $(INCLUDE) -c $< -o $@

$(OBJ_DIR)/power/%.o: $(SRC_DIR)/power/%.cpp
	$(CC) $(CFLAGS) $(INCLUDE) -c $< -o $@

all: newmat mkobjdir $(OBJECTS)
	$(CC) $(OBJECTS) $(CFLAGS) $(NM_LIB) -o $(OBJ_TARGET)

newmat:
	$(MAKE) -C newmat10 -f nm_gnu.mak libnewmat.a

mkobjdir:
	mkdir -p $(OBJ_DIR)
	mkdir -p $(OBJ_DIR)/ggraph
	mkdir -p $(OBJ_DIR)/cmp
	mkdir -p $(OBJ_DIR)/model
	mkdir -p $(OBJ_DIR)/arch
	mkdir -p $(OBJ_DIR)/explore
	mkdir -p $(OBJ_DIR)/stat
	mkdir -p $(OBJ_DIR)/perf
	mkdir -p $(OBJ_DIR)/phys
	mkdir -p $(OBJ_DIR)/power

clean:
	rm -rf $(OBJ_DIR)
	rm -f ./cmpexplore
