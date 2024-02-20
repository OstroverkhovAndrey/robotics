
CC = g++

CFLAGS = -c -Wall -g -I./src
OPENCV_FLAGS = -I/usr/local/include/opencv4 -L/usr/lib/x86_64-linux-gnu -lopencv_gapi -lopencv_stitching -lopencv_aruco -lopencv_barcode -lopencv_bgsegm -lopencv_bioinspired -lopencv_ccalib -lopencv_dnn_objdetect -lopencv_dnn_superres -lopencv_dpm -lopencv_face -lopencv_freetype -lopencv_fuzzy -lopencv_hfs -lopencv_img_hash -lopencv_intensity_transform -lopencv_line_descriptor -lopencv_mcc -lopencv_quality -lopencv_rapid -lopencv_reg -lopencv_rgbd -lopencv_saliency -lopencv_stereo -lopencv_structured_light -lopencv_phase_unwrapping -lopencv_superres -lopencv_optflow -lopencv_surface_matching -lopencv_tracking -lopencv_highgui -lopencv_datasets -lopencv_text -lopencv_plot -lopencv_videostab -lopencv_videoio -lopencv_wechat_qrcode -lopencv_xfeatures2d -lopencv_shape -lopencv_ml -lopencv_ximgproc -lopencv_video -lopencv_xobjdetect -lopencv_objdetect -lopencv_calib3d -lopencv_imgcodecs -lopencv_features2d -lopencv_dnn -lopencv_flann -lopencv_xphoto -lopencv_photo -lopencv_imgproc -lopencv_core
#OPENCV_FLAGS = -I/usr/local/include/opencv4 -L/usr/local/lib -lopencv_gapi -lopencv_stitching -lopencv_aruco -lopencv_barcode -lopencv_bgsegm -lopencv_bioinspired -lopencv_ccalib -lopencv_dnn_objdetect -lopencv_dnn_superres -lopencv_dpm -lopencv_face -lopencv_freetype -lopencv_fuzzy -lopencv_hfs -lopencv_img_hash -lopencv_intensity_transform -lopencv_line_descriptor -lopencv_mcc -lopencv_quality -lopencv_rapid -lopencv_reg -lopencv_rgbd -lopencv_saliency -lopencv_stereo -lopencv_structured_light -lopencv_phase_unwrapping -lopencv_superres -lopencv_optflow -lopencv_surface_matching -lopencv_tracking -lopencv_highgui -lopencv_datasets -lopencv_text -lopencv_plot -lopencv_videostab -lopencv_videoio -lopencv_wechat_qrcode -lopencv_xfeatures2d -lopencv_shape -lopencv_ml -lopencv_ximgproc -lopencv_video -lopencv_xobjdetect -lopencv_objdetect -lopencv_calib3d -lopencv_imgcodecs -lopencv_features2d -lopencv_dnn -lopencv_flann -lopencv_xphoto -lopencv_photo -lopencv_imgproc -lopencv_core

PYTHON_FLAGS = -lm -std=c++17 -O3 -I/usr/include/python3.10/ -L /usr/lib/x86_64-linux-gnu/ -lpython3.10

OBJECTS = ./build/matrix.o ./build/normal_distribution.o ./build/visualization.o ./build/visualization_info.o ./build/visualization_opencv.o ./build/main.o ./build/robot_emulation.o ./build/robot.o ./build/regulator.o ./build/kalman_filter.o ./build/constants.o
EXECUTABLE = ./build/main

all: $(OBJECTS)
	$(CC) $(OBJECTS) -o $(EXECUTABLE) $(OPENCV_FLAGS) $(PYTHON_FLAGS)

run: all
	$(EXECUTABLE) simple_kalman_filter

./build/matrix.o: ./src/base/math/matrix.cpp ./src/base/math/matrix.hpp
	$(CC) ./src/base/math/matrix.cpp -o ./build/matrix.o $(CFLAGS)

./build/constants.o: ./src/base/constants/constants.cpp ./src/base/constants/constants.hpp
	$(CC) ./src/base/constants/constants.cpp -o ./build/constants.o $(CFLAGS)

./build/normal_distribution.o: ./src/base/random_variable/normal_distribution.cpp ./src/base/random_variable/normal_distribution.hpp
	$(CC) ./src/base/random_variable/normal_distribution.cpp -o ./build/normal_distribution.o $(CFLAGS) $(PYTHON_FLAGS)

./build/visualization.o: ./src/visualization/visualization.cpp ./src/visualization/visualization.hpp
	$(CC) ./src/visualization/visualization.cpp -o ./build/visualization.o $(CFLAGS)

./build/visualization_info.o: ./src/visualization/visualization_info.cpp ./src/visualization/visualization_info.hpp
	$(CC) ./src/visualization/visualization_info.cpp -o ./build/visualization_info.o $(CFLAGS)

./build/visualization_opencv.o: ./src/visualization/visualization_opencv.cpp ./src/visualization/visualization_opencv.hpp
	$(CC) ./src/visualization/visualization_opencv.cpp -o ./build/visualization_opencv.o $(CFLAGS) $(OPENCV_FLAGS) $(PYTHON_FLAGS)

./build/main.o: ./src/kalman_filter/main.cpp
	$(CC) ./src/kalman_filter/main.cpp -o ./build/main.o $(CFLAGS) $(OPENCV_FLAGS) $(PYTHON_FLAGS)

./build/kalman_filter.o: ./src/kalman_filter/kalman_filter.cpp ./src/kalman_filter/kalman_filter.hpp
	$(CC) ./src/kalman_filter/kalman_filter.cpp -o ./build/kalman_filter.o $(CFLAGS)

./build/robot.o: ./src/robot/robot.cpp ./src/robot/robot.hpp
	$(CC) ./src/robot/robot.cpp -o ./build/robot.o $(CFLAGS)

./build/robot_emulation.o: ./src/robot/robot_emulation.cpp ./src/robot/robot_emulation.hpp
	$(CC) ./src/robot/robot_emulation.cpp -o ./build/robot_emulation.o $(CFLAGS)

./build/regulator.o: ./src/base/regulator/regulator.cpp ./src/base/regulator/regulator.hpp
	$(CC) ./src/base/regulator/regulator.cpp -o ./build/regulator.o $(CFLAGS)

clean:
	rm -rf ./build/*.o ./main 
