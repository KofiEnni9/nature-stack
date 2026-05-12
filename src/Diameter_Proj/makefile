CXX = g++
CXXFLAGS = -std=c++17 -O2 -pthread

INCLUDES = \
  -I/scratch/ld212/bag_files/nature-stack/pcl-install/include/pcl-1.15 \
  -I/scratch/ld212/bag_files/nature-stack/flann-install/include \
  -I/scratch/ld212/bag_files/nature-stack/include

LDFLAGS = \
  -L/scratch/ld212/bag_files/nature-stack/pcl-install/lib \
  -L/scratch/ld212/bag_files/nature-stack/flann-install/lib \
  -Wl,-rpath,/scratch/ld212/bag_files/nature-stack/pcl-install/lib \
  -Wl,-rpath,/scratch/ld212/bag_files/nature-stack/flann-install/lib

LIBS = \
  -lpcl_common -lpcl_io -lpcl_octree -lpcl_kdtree -lpcl_search \
  -lpcl_features -lpcl_filters -lpcl_segmentation -lpcl_sample_consensus \
  -lflann_cpp -lboost_system -lboost_filesystem

DEFINES = -DEIGEN_MAX_ALIGN_BYTES=32

SRCS = kdTree.cpp euclidean_cluster.cpp main.cpp ../perception/elevation_grid.cpp

TARGET = outputpcd

all: $(TARGET)

$(TARGET): $(SRCS)
	$(CXX) $(CXXFLAGS) $(DEFINES) $(INCLUDES) $(SRCS) $(LDFLAGS) $(LIBS) -o $(TARGET)

clean:
	rm -f $(TARGET)