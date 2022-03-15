cmake_minimum_required(VERSION 2.8.3)
project(op_tlr)

find_package(autoware_build_flags REQUIRED)

find_package(CUDA REQUIRED)
find_package(OpenCV REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge  
  roscpp  
  std_msgs
  image_transport  
  sensor_msgs
  autoware_msgs
  geometry_msgs
  roslint  
  tf  
  op_planner
  op_ros_helpers
)

find_package(Eigen3 QUIET)

if (NOT EIGEN3_FOUND)
    # Fallback to cmake_modules
    find_package(cmake_modules REQUIRED)
    find_package(Eigen REQUIRED)
    set(EIGEN3_INCLUDE_DIRS ${EIGEN_INCLUDE_DIRS})
    set(EIGEN3_LIBRARIES ${EIGEN_LIBRARIES})  # Not strictly necessary as Eigen is head only
    # Possibly map additional variables to the EIGEN3_ prefix.
else ()
    set(EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
endif ()


set(
    CUDA_NVCC_FLAGS    
    "-O3        
    -gencode arch=compute_35,code=sm_35 
    -gencode arch=compute_50,code=[sm_50,compute_50] 
    -gencode arch=compute_52,code=[sm_52,compute_52]    
    -gencode arch=compute_61,code=[sm_61,compute_61] 
    -gencode arch=compute_70,code=[sm_70,compute_70] 
    -gencode arch=compute_86,code=[sm_86,compute_86] "    
    )

add_definitions(-DGPU -DCUDNN -DCUDNN_HALF -DOPENCV -fopenmp -lgomp -ffp-contract=fast -mavx -mavx2 -msse3 -msse4.1 -msse4.2 -msse4a)

catkin_package(
  INCLUDE_DIRS ${EIGEN3_INCLUDE_DIRS}
  CATKIN_DEPENDS 
)

###################################
## catkin specific configuration ##
###################################

set(DARKNET_DIR "darknet")
set(DARKNET_SRC_DIR "${DARKNET_DIR}/src")


include_directories(
  include
  /usr/local/cuda-11.1/include  
  src  
  ${DARKNET_DIR}/include  
  ${DARKNET_DIR}/3rdparty/stb/include
  ${DARKNET_SRC_DIR}
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS}
)

link_directories(    
  /usr/local/cuda-11.1/lib64
)

set(DARKNET_SRC_FILES
	${DARKNET_SRC_DIR}/activation_layer.c
	${DARKNET_SRC_DIR}/activations.c
	${DARKNET_SRC_DIR}/art.c
	${DARKNET_SRC_DIR}/avgpool_layer.c
	${DARKNET_SRC_DIR}/batchnorm_layer.c
	${DARKNET_SRC_DIR}/blas.c
	${DARKNET_SRC_DIR}/box.c
	${DARKNET_SRC_DIR}/captcha.c
	${DARKNET_SRC_DIR}/cifar.c
	${DARKNET_SRC_DIR}/classifier.c
	${DARKNET_SRC_DIR}/coco.c
	${DARKNET_SRC_DIR}/col2im.c
	${DARKNET_SRC_DIR}/compare.c
	${DARKNET_SRC_DIR}/connected_layer.c
	${DARKNET_SRC_DIR}/conv_lstm_layer.c
	${DARKNET_SRC_DIR}/convolutional_layer.c
	${DARKNET_SRC_DIR}/cost_layer.c
	${DARKNET_SRC_DIR}/cpu_gemm.c
	${DARKNET_SRC_DIR}/crnn_layer.c
	${DARKNET_SRC_DIR}/crop_layer.c
	${DARKNET_SRC_DIR}/dark_cuda.c
	#${DARKNET_SRC_DIR}/darknet.c
	${DARKNET_SRC_DIR}/data.c
	${DARKNET_SRC_DIR}/deconvolutional_layer.c
	${DARKNET_SRC_DIR}/demo.c
	${DARKNET_SRC_DIR}/detection_layer.c
	${DARKNET_SRC_DIR}/detector.c
	${DARKNET_SRC_DIR}/dice.c
	${DARKNET_SRC_DIR}/dropout_layer.c
	${DARKNET_SRC_DIR}/gaussian_yolo_layer.c
	${DARKNET_SRC_DIR}/gemm.c
	${DARKNET_SRC_DIR}/getopt.c
	${DARKNET_SRC_DIR}/gettimeofday.c
	${DARKNET_SRC_DIR}/go.c
	${DARKNET_SRC_DIR}/gru_layer.c
	${DARKNET_SRC_DIR}/http_stream.cpp
	${DARKNET_SRC_DIR}/im2col.c
	${DARKNET_SRC_DIR}/image_opencv.cpp
	${DARKNET_SRC_DIR}/image.c
	${DARKNET_SRC_DIR}/layer.c
	${DARKNET_SRC_DIR}/list.c
	${DARKNET_SRC_DIR}/local_layer.c
	${DARKNET_SRC_DIR}/lstm_layer.c
	${DARKNET_SRC_DIR}/matrix.c
	${DARKNET_SRC_DIR}/maxpool_layer.c
	${DARKNET_SRC_DIR}/network.c
	${DARKNET_SRC_DIR}/nightmare.c
	${DARKNET_SRC_DIR}/normalization_layer.c
	${DARKNET_SRC_DIR}/option_list.c
	${DARKNET_SRC_DIR}/parser.c
	${DARKNET_SRC_DIR}/region_layer.c	
	${DARKNET_SRC_DIR}/reorg_layer.c
	${DARKNET_SRC_DIR}/reorg_old_layer.c
	${DARKNET_SRC_DIR}/rnn_layer.c
	${DARKNET_SRC_DIR}/rnn_vid.c
	${DARKNET_SRC_DIR}/route_layer.c
	${DARKNET_SRC_DIR}/sam_layer.c
	${DARKNET_SRC_DIR}/scale_channels_layer.c
	${DARKNET_SRC_DIR}/shortcut_layer.c
	${DARKNET_SRC_DIR}/softmax_layer.c
	${DARKNET_SRC_DIR}/super.c	
	${DARKNET_SRC_DIR}/swag.c
	${DARKNET_SRC_DIR}/tag.c
	${DARKNET_SRC_DIR}/tree.c
	${DARKNET_SRC_DIR}/upsample_layer.c		
	${DARKNET_SRC_DIR}/utils.c	
	${DARKNET_SRC_DIR}/voxel.c
	${DARKNET_SRC_DIR}/writing.c
	#${DARKNET_SRC_DIR}/yolo_console_dll.cpp
	${DARKNET_SRC_DIR}/yolo_layer.c		
	${DARKNET_SRC_DIR}/yolo_v2_class.cpp
	${DARKNET_SRC_DIR}/yolo.c
	
	#${DARKNET_SRC_DIR}/upsample_layer.c		
	#${DARKNET_SRC_DIR}/l2norm_layer.c	
	#${DARKNET_SRC_DIR}/logistic_layer.c
		
	${DARKNET_SRC_DIR}/activation_kernels.cu
	${DARKNET_SRC_DIR}/avgpool_layer_kernels.cu
	${DARKNET_SRC_DIR}/blas_kernels.cu
	${DARKNET_SRC_DIR}/col2im_kernels.cu
	${DARKNET_SRC_DIR}/convolutional_kernels.cu
	${DARKNET_SRC_DIR}/crop_layer_kernels.cu
	${DARKNET_SRC_DIR}/deconvolutional_kernels.cu
	${DARKNET_SRC_DIR}/dropout_layer_kernels.cu
	${DARKNET_SRC_DIR}/im2col_kernels.cu
	${DARKNET_SRC_DIR}/maxpool_layer_kernels.cu
	${DARKNET_SRC_DIR}/network_kernels.cu
	${DARKNET_SRC_DIR}/version.h.in
)


cuda_add_executable(op_tlr
  nodes/op_tlr.cpp
  nodes/DarknetDetector.cpp  
  nodes/op_tlr_core.cpp 	
  "${DARKNET_SRC_FILES}"
)


target_link_libraries(op_tlr
   m
   pthread
   stdc++
   cuda
   cudart
   cublas
   curand
   gomp
   cudnn
   ${catkin_LIBRARIES}
   ${OpenCV_LIBRARIES}   
)

add_dependencies(op_tlr ${catkin_EXPORTED_TARGETS})
