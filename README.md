# ceng477-rasterization

##### Authors: Doruk Coşkun (/Doruk-Coskun) & Çağrı Eser (/cagries)

![An example](https://raw.githubusercontent.com/Doruk-Coskun/ceng477-rasterization/master/Homework2/outputs/culling_enabled_outputs/filled_box/filled_box_6.ppm.png?token=AYU2WzC3zxMCsyPYBfU3jxnLfgq144pqks5ca_icwA%3D%3D)

![Another example](https://raw.githubusercontent.com/Doruk-Coskun/ceng477-rasterization/master/Homework2/outputs/culling_enabled_outputs/horse_and_mug/horse_and_mug_2.ppm.png?token=AYU2Wzaak4S_gaGxugg22Mm1Q1uS29Tgks5ca_hjwA%3D%3D)

We have implemented Modeling Transformation, Viewing Transformation, and Rasterization stages of the Forward Rendering Pipeline in C++. After the rasterization stage, we have interpolated the triangle colors according to the vertex colors. We have also implemented backface culling and wireframe mode which is enabled according to the scene data.

Check `Homework2.pdf` for more implementation details.

# How to Run

Download the repository and its submodules by running

```
$ git clone https://github.com/Doruk-Coskun/ceng477-rasterization
```

To build the project, run the following commands from the project root directory:

```
$ cd Homework2/source/cpp
$ make
```

This creates the `rasterizer` executable inside the same directory.

From the `Homework2/inputs` folder choose the desired scene. Inside the `Homework2/source/cpp`, run:

```
$ ./rasterizer <scene.txt> <camera.txt>
```

to generate output PPM formatted images for each camera defined in the given camera file.
