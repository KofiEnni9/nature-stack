#include "leastsqrfitting.cpp"
#include <cstdio>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <iostream>


int main()
{


// input pcd = "/scratch/ld212/bag_files/backyard/pointclouds/1680633522931820011.pcd"
// output pcd = "/scratch/ld212/bag_files/backyard/pointclouds/result_output.pcd"



    FILE* file = fopen("src/Diameter_Proj/subpcdimages/cropped_2.ply", "rb");
    if (!file) {
        perror("Failed to open file");
        return 1;
    }

    // Read the PLY file header
    char line[256];
    int n = 0;
    while (fgets(line, sizeof(line), file)) {
        sscanf(line, "element vertex %d", &n);
        if (strncmp(line, "end_header", 10) == 0) {
            break;
        }
    }
    
    // Read the point cloud data
    glm::vec3* points = (glm::vec3*)malloc(n * sizeof(glm::vec3));
    if (!points) {
        perror("Failed to allocate memory");
        fclose(file);
        return 1;
    }

    int pointCount = 0;
    while (pointCount < n) {
        double xyz[3];
        if (fread(xyz, sizeof(double), 3, file) != 3) break;
        points[pointCount++] = glm::vec3((float)xyz[0], (float)xyz[1], (float)xyz[2]);
    }

    printf("Read %d points from PLY file.\n", pointCount);
    for (int i = 0; i < 10 && i < pointCount; i++) {
        printf("  %.4f, %.4f, %.4f\n", points[i].x, points[i].y, points[i].z);
    }

    fclose(file);

    // ---- Run the cylinder fitting ----
    float rSqr;
    glm::vec3 C, W;
    float error = FitCylinder(pointCount, points, rSqr, C, W);

    // ---- Print results ----
    // printf("=== Known Cylinder ===\n");
    // printf("  Axis:   (%.3f, %.3f, %.3f)\n", knownAxis.x, knownAxis.y, knownAxis.z);
    // printf("  Center: (%.3f, %.3f, %.3f)\n", knownCenter.x, knownCenter.y, knownCenter.z);
    // printf("  Radius: %.3f\n", knownRadius);

    printf("\n=== Fitted Cylinder ===\n");
    printf("  Axis:   (%.3f, %.3f, %.3f)\n", W.x, W.y, W.z);
    printf("  Center: (%.3f, %.3f, %.3f)\n", C.x, C.y, C.z);
    printf("  Radius: %.3f  (sqrt of rSqr=%.3f)\n", sqrt(rSqr), rSqr);
    printf("  Error:  %e\n", error);

    return 0;
}



// running this program
// g++ src/Diameter_Proj/diam_proj.cpp -I /scratch/ld212/bag_files/nature-stack/include -o diam_proj