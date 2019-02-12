rasterizer_cpp:
	g++ -std=c++11 -o rasterizer hw2_math_ops.cpp hw2_file_ops.cpp rasterizer.cpp -g

test: rasterizer_cpp
	./rasterizer ../../inputs/culling_disabled_inputs/empty_box/empty_box_scene.txt \
		../../inputs/culling_disabled_inputs/empty_box/empty_box_camera.txt

clean:
	rm -rf *.ppm *.png ./rasterizer

