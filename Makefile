OMPL_DIR = /usr
CXXFLAGS = -g -std=c++11 # change to -g when debugging code
INCLUDE_FLAGS = -I${OMPL_DIR}/include
LD_FLAGS = -L${OMPL_DIR}/lib -lompl -lompl_app -lboost_program_options -lboost_system
CXX=g++

MyRigidBodyPlanning: MyRigidBodyPlanning.o randomtree.o
	$(CXX) $(CXXFLAGS) -o MyRigidBodyPlanning MyRigidBodyPlanning.o randomtree.o $(LD_FLAGS)

clean:
	rm *.o

%.o: %.cpp
	$(CXX) -c $(CXXFLAGS) $(INCLUDE_FLAGS) $< -o $@
