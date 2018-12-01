#include <cstdio>
#include <cstdlib>
#include <cmath>
#include <cstring>
#include "hw2_types.h"
#include "hw2_math_ops.h"
#include "hw2_file_ops.h"
#include <iostream>

#include <cassert>

// After his r = m.
void equalizeMatrices(double r[4][4], double m[4][4]) {
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            r[i][j] = m[i][j];
        }
    }
}

// After this r is a zero matrix.
void makeZeroMatrix(double r[4][4]) {
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            r[i][j] = 0;
        }
    }
}

/**
 * @brief Namespace for modeling transformations.
 */
namespace model_tr {

/**
 * @brief Translate the 4D matrix r by the
 *   translation t and store the result in r
 *
 * @param r[4][4]
 * @param t
 */
void getTranslationMatrix(double r[4][4], Translation t) {
    makeIdentityMatrix(r);
    r[0][3] = t.tx;
    r[1][3] = t.ty;
    r[2][3] = t.tz;
}



/**
 * @brief Scale the 4D matrix r by the scaling factors in
 * s.
 *
 * @param r[4][4]
 * @param s
 */
void getScalingMatrix(double r[4][4], Scaling s) {
    makeIdentityMatrix(r);
    r[0][0] = s.sx;
    r[1][1] = s.sy;
    r[2][2] = s.sz;
}

/**
 * @brief Helper for getRotationMatrix.
 */
void getRotationXMatrix(double r[4][4], Rotation rot, bool sign) {
    double b = rot.uy;
    double c = rot.uz;
    double d = std::sqrt(b * b + c * c);

    if (d == 0) {
        makeIdentityMatrix(r);
        return;
    }

    r[1][1] = c / d;
    r[1][2] = (-b) / d;

    r[2][1] = b / d;
    r[2][2] = c / d;

    if (!sign) {
        r[1][2] = -r[1][2];
        r[2][1] = -r[2][1];
    }

}

/**
 * @brief Helper for getRotationMatrix.
 */
void getRotationYMatrix(double r[4][4], Rotation rot, bool sign) {
    double b = rot.uy;
    double c = rot.uz;
    double d = std::sqrt(b * b + c * c);

    r[0][0] = d;
    r[0][2] = (rot.ux);
    r[2][0] = -rot.ux;
    r[2][2] = d;

    if (!sign) {
        r[0][2] = -r[0][2];
        r[2][0] = -r[2][0];
    }
}

/**
 * @brief Helper for getRotationMatrix.
 */
void getRotationZMatrix(double r[4][4], Rotation rot) {

    double theta = (rot.angle * M_PI) / 180;
    r[0][0] = std::cos(theta);
    r[0][1] = -std::sin(theta);
    r[1][0] = std::sin(theta);
    r[1][1] = std::cos(theta);
}

/**
 * @brief Get the rotation matrix according to the
 * rotation rot.
 *
 * @param r[4][4]
 * @param rot
 */
void getRotationMatrix(double r[4][4], Rotation rot) {
    makeIdentityMatrix(r);
    double rotx[4][4], roty[4][4], rotz[4][4];
    double i1[4][4], i2[4][4], i3[4][4];

    makeIdentityMatrix(rotx);
    makeIdentityMatrix(roty);
    makeIdentityMatrix(rotz);

    getRotationXMatrix(rotx, rot, false);
    getRotationYMatrix(roty, rot, true);
    getRotationZMatrix(rotz, rot);

    multiplyMatrixWithMatrix(i1, rotx, roty);
    multiplyMatrixWithMatrix(i2, i1, rotz);

    getRotationXMatrix(rotx, rot, true);
    getRotationYMatrix(roty, rot, false);

    multiplyMatrixWithMatrix(i3, i2, roty);
    multiplyMatrixWithMatrix(r, i3, rotx);

}

}; // namespace model_tr

/**
 * @brief Namespace for various test suites.
 */
namespace tests {

/**
 * @brief Checks equality depending on a small interval eps.
 *
 * @param val1 First value
 * @param val2 Second value 
 * @param eps  Equality threshold
 */
bool near_equal(double val1, double val2, double eps) {
    return std::abs(val1 - val2) < eps;
}


/**
 * @brief Runs a small set of modeling
 *   transform tests which fail
 *   if they don't satisfy the assertions.
 */
void test_modeling_transforms() {
    std::size_t num_tests = 0;

    // Test 1
    double v[4] = {0,1,0, 1}; 
    double r1[4][4];
    makeIdentityMatrix(r1);

    double val = 1 / std::sqrt(3);
    Rotation rot = {
        240,
        val,
        val,
        val
    };

    model_tr::getRotationMatrix(r1, rot);

    double result[4];
    multiplyMatrixWithVec4d(result, r1, v);
    double eps = 1e-4;

    assert(
        near_equal(result[0], 1, eps) &&
        near_equal(result[1], 0, eps) &&
        near_equal(result[2], 0, eps)
    );
    num_tests++;

    // Test 2
    v[0] = 2;
    v[1] = 0;
    v[2] = 0;

    rot.angle = 90;
    rot.ux = rot.uz = 0;
    rot.uy = 1;

    makeIdentityMatrix(r1);
    model_tr::getRotationMatrix(r1, rot);
    multiplyMatrixWithVec4d(result, r1, v);

    assert(
        near_equal(result[0], 0, eps) &&
        near_equal(result[1], 0, eps) &&
        near_equal(result[2], -2, eps)
    );
    num_tests++;

    // Test 3: Translation
    
    v[0] = 1;
    v[1] = -2;
    v[2] = 3;

    Translation t = {
        2,
        0,
        -10
    };

    makeIdentityMatrix(r1);
    model_tr::getTranslationMatrix(r1, t);
    multiplyMatrixWithVec4d(result, r1, v);

    assert(
        near_equal(result[0], 3, eps) &&
        near_equal(result[1], -2, eps) &&
        near_equal(result[2], -7, eps)
    );
    num_tests++;

    Scaling s = {
        0.5, -5, 0
    };

    makeIdentityMatrix(r1);
    model_tr::getScalingMatrix(r1, s);
    multiplyMatrixWithVec4d(result, r1, result);

    assert(
        near_equal(result[0], 1.5, eps) &&
        near_equal(result[1], 10, eps) &&
        near_equal(result[2], 0, eps)
    );
    num_tests++;

    std::cout << "[Tests] passed " << num_tests << " tests." << std::endl;
}

}; // namespace tests

Camera cameras[100];
int numberOfCameras = 0;

Model models[1000];
int numberOfModels = 0;

Color colors[100000];
int numberOfColors = 0;

Translation translations[1000];
int numberOfTranslations = 0;

Rotation rotations[1000];
int numberOfRotations = 0;

Scaling scalings[1000];
int numberOfScalings = 0;

Vec3 vertices[100000];
int numberOfVertices = 0;

Color backgroundColor;

// backface culling setting, default disabled
int backfaceCullingSetting = 0;

Color **image;



/*
	Initializes image with background color
*/
void initializeImage(Camera cam) {
    int i, j;

    for (i = 0; i < cam.sizeX; i++)
        for (j = 0; j < cam.sizeY; j++) {
            image[i][j].r = backgroundColor.r;
            image[i][j].g = backgroundColor.g;
            image[i][j].b = backgroundColor.b;

        }
}

/*
	Transformations, culling, rasterization are done here.
	You can define helper functions inside this file (rasterizer.cpp) only.
	Using types in "hw2_types.h" and functions in "hw2_math_ops.cpp" will speed you up while working.
*/
void modelingTransformation(Model model) {
    // Have to keep new vertice coordinates afeter transformations.
    Vec3 triangleVertices[model.numberOfTriangles][3];
    
    // transformMatrix is the result of all the transform operations.
    // operationMatrix is a tmp matrix to hold new transform operation.
    double transformMatrix[4][4], operationMatrix[4][4], tmpMatrix[4][4];
    makeIdentityMatrix(transformMatrix);
    makeIdentityMatrix(operationMatrix);
    
    // Copy current vertices coordiates of the model.
    for (int i = 0; i < model.numberOfTriangles; i++) {
        triangleVertices[i][0] = vertices[model.triangles[i].vertexIds[0]];
        triangleVertices[i][1] = vertices[model.triangles[i].vertexIds[1]];
        triangleVertices[i][2] = vertices[model.triangles[i].vertexIds[2]];
    }
    
    // Form the transform matrix which is going to be applied to all vertices.
    for (int i = 0; i < model.numberOfTransformations; i++) {
        switch (model.transformationTypes[i]) {
            case 'r':
                model_tr::getRotationMatrix(operationMatrix, rotations[model.transformationIDs[i]]);
                break;
            case 't':
                model_tr::getTranslationMatrix(operationMatrix, translations[model.transformationIDs[i]]);
                break;
            case 's':
                model_tr::getScalingMatrix(operationMatrix, scalings[model.transformationIDs[i]]);
                break;
        }
        
        multiplyMatrixWithMatrix(tmpMatrix, operationMatrix, transformMatrix);
        equalizeMatrices(transformMatrix, tmpMatrix);
        
        // Prints out the current operationMatrix and last version of transformMatrix.
//
//        std::cout << "Operation Matrix" << std::endl;
//        std::cout << operationMatrix[1][1] << ' ';
//        std::cout << operationMatrix[0][3] << ' ';
//        std::cout << operationMatrix[1][3] << ' ';
//        std::cout << operationMatrix[2][3] << std::endl;
//        std::cout << "Transform Matrix" << std::endl;
//        std::cout << transformMatrix[1][1] << ' ';
//        std::cout << transformMatrix[0][3] << ' ';
//        std::cout << transformMatrix[1][3] << ' ';
//        std::cout << transformMatrix[2][3] << std::endl;

    }
    
    // Apply transformMatrix to all vertices.
    double point[4], tmpVec[4];
    for (int i = 0; i < model.numberOfTriangles; i++) {
        for (int j = 0; j < 3; j++) {
            point[0] = triangleVertices[i][j].x;
            point[1] = triangleVertices[i][j].y;
            point[2] = triangleVertices[i][j].z;
            point[3] = 1;
            
            multiplyMatrixWithVec4d(tmpVec, transformMatrix, point);
            
            triangleVertices[i][j].x = tmpVec[0];
            triangleVertices[i][j].y = tmpVec[1];
            triangleVertices[i][j].z = tmpVec[2];
        }
    }
    
    // Print out transformed vertecies of each triangle of the model.
    for (int i = 0; i < model.numberOfTriangles; i++) {
        std::cout << "Triangle: " << i << std::endl;
        for (int j = 0; j < 3; j++) {
            std::cout << "Vertex " << j << ": ";
            std::cout << triangleVertices[i][j].x << ' ';
            std::cout << triangleVertices[i][j].y << ' ';
            std::cout << triangleVertices[i][j].z << std::endl;
        }
    }
}

void forwardRenderingPipeline(Camera cam) {
    // TODO: IMPLEMENT HERE
    //tests::test_modeling_transforms();
    
    std::cout << "Number of models " << numberOfModels << std::endl;
    
    // Transforms every model in the scene.
    for (int i = 0; i < numberOfModels; i++) {
        modelingTransformation(models[i]);
    }

}


int main(int argc, char **argv) {
    int i, j;

    if (argc < 2) {
        std::cout << "Usage: ./rasterizer <scene file> <camera file>" << std::endl;
        return 1;
    }

    // read camera and scene files
    readSceneFile(argv[1]);
    readCameraFile(argv[2]);

    image = 0;

    // TODO:
    // remove this later:
    numberOfCameras = 1;
    for (i = 0; i < numberOfCameras; i++) {

        // allocate memory for image
        if (image) {
			for (j = 0; j < cameras[i].sizeX; j++) {
		        delete image[j];
		    }

			delete[] image;
		}

        image = new Color*[cameras[i].sizeX];

        if (image == NULL) {
            std::cout << "ERROR: Cannot allocate memory for image." << std::endl;
            exit(1);
        }

        for (j = 0; j < cameras[i].sizeX; j++) {
            image[j] = new Color[cameras[i].sizeY];
            if (image[j] == NULL) {
                std::cout << "ERROR: Cannot allocate memory for image." << std::endl;
                exit(1);
            }
        }


        // initialize image with basic values
        initializeImage(cameras[i]);

        // do forward rendering pipeline operations
        forwardRenderingPipeline(cameras[i]);

        // generate PPM file
        writeImageToPPMFile(cameras[i]);

        // Converts PPM image in given path to PNG file, by calling ImageMagick's 'convert' command.
        // Notice that os_type is not given as 1 (Ubuntu) or 2 (Windows), below call doesn't do conversion.
        // Change os_type to 1 or 2, after being sure that you have ImageMagick installed.
        convertPPMToPNG(cameras[i].outputFileName, 99);
    }

    return 0;

}
