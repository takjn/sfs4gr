/*
** tiny Point Cloud Library
**
** Copyright (c) 2017 Jun Takeda
**
** Permission is hereby granted, free of charge, to any person obtaining
** a copy of this software and associated documentation files (the
** "Software"), to deal in the Software without restriction, including
** without limitation the rights to use, copy, modify, merge, publish,
** distribute, sublicense, and/or sell copies of the Software, and to
** permit persons to whom the Software is furnished to do so, subject to
** the following conditions:
**
** The above copyright notice and this permission notice shall be
** included in all copies or substantial portions of the Software.
**
** THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
** EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
** MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
** IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY
** CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
** TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
** SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
**
** [ MIT license: http://www.opensource.org/licenses/mit-license.php ]
*/

#include <bitset>
#include <stdio.h>
#include "tinypcl.hpp"

PointCloud::PointCloud(void) {
    clear();
}

// 指定された点の値の読み取り
unsigned char PointCloud::get(unsigned int index) {
    return point_cloud_data[index];
}

unsigned char PointCloud::get(unsigned int x, unsigned int y, unsigned int z) {
    return point_cloud_data(x,y,z);
}

// 指定された点に値を設定
void PointCloud::set(unsigned int index, unsigned char val) {
    point_cloud_data[index] = val;
}

void PointCloud::set(unsigned int x, unsigned int y, unsigned int z, unsigned char val) {
    point_cloud_data(x,y,z) = val;
}

// 点群データの初期化
void PointCloud::clear(void) {
    for (int z=0;z<POINTS;z++) {
        for (int y=0;y<POINTS;y++) {
            for (int x=0;x<POINTS;x++) {
                point_cloud_data(x,y,z) = 1;
            }
        }
    }
}

// 外周部のデータの削除（メッシュ化に必要）
void PointCloud::remove_edge(void) {
    for (int i=0; i<POINTS; i++) {
        for (int j=0; j<POINTS; j++) {
            // 外周部は除去
            point_cloud_data(i,j,0) = 0;
            point_cloud_data(i,0,j) = 0;
            point_cloud_data(0,i,j) = 0;
            point_cloud_data(i,j,POINTS-1) = 0;
            point_cloud_data(i,POINTS-1,j) = 0;
            point_cloud_data(POINTS-1,i,j) = 0;
        }
    }
}

// 点群データをメッシュ化してplyファイルとして保存
void PointCloud::save_as_ply(const char* file_name) {
    FILE *fp_ply = fopen(file_name, "w");

    // 先に面の数を数えておく
    int x,y,z;
    int face_count=0;
    for (z=1; z<POINTS-1; z++) {
        for (y=1; y<POINTS-1; y++) {
            for (x=1; x<POINTS-1; x++) {
                if (point_cloud_data(x,y,z) == 1) {
                    if (point_cloud_data(x,y,z+1) == 0) face_count++;
                    if (point_cloud_data(x+1,y,z) == 0) face_count++;
                    if (point_cloud_data(x,y,z-1) == 0) face_count++;
                    if (point_cloud_data(x-1,y,z) == 0) face_count++;
                    if (point_cloud_data(x,y+1,z) == 0) face_count++;
                    if (point_cloud_data(x,y-1,z) == 0) face_count++;
                }
            }
        }
    }

	// write PLY file header
    fprintf(fp_ply,"ply\n");
    fprintf(fp_ply,"format ascii 1.0\n");
    fprintf(fp_ply,"element vertex %d\n", face_count*4);
    fprintf(fp_ply,"property float x\n");
    fprintf(fp_ply,"property float y\n");
    fprintf(fp_ply,"property float z\n");
    fprintf(fp_ply,"property uchar red\n");
    fprintf(fp_ply,"property uchar green\n");
    fprintf(fp_ply,"property uchar blue\n");
    fprintf(fp_ply,"element face %d\n", face_count);
    fprintf(fp_ply,"property list uint8 int32 vertex_indices\n");
    fprintf(fp_ply,"end_header\n");

    // Vertexの出力
    for (z=1; z<POINTS-1; z++) {
        for (y=1; y<POINTS-1; y++) {
            for (x=1; x<POINTS-1; x++) {
                if (point_cloud_data(x,y,z) == 1) {

                    if (point_cloud_data(x,y,z-1) == 0) {
                        fprintf(fp_ply,"%f %f %f 200 200 200\n", x    *SCALE, y    *SCALE, z    *SCALE);
                        fprintf(fp_ply,"%f %f %f 200 200 200\n", x    *SCALE, (y+1)*SCALE, z    *SCALE);
                        fprintf(fp_ply,"%f %f %f 200 200 200\n", (x+1)*SCALE, (y+1)*SCALE, z    *SCALE);
                        fprintf(fp_ply,"%f %f %f 200 200 200\n", (x+1)*SCALE, y    *SCALE, z    *SCALE);
                    }

                    if (point_cloud_data(x,y,z+1) == 0) {
                        fprintf(fp_ply,"%f %f %f 200 200 200\n", x    *SCALE, y    *SCALE, (z+1)*SCALE);
                        fprintf(fp_ply,"%f %f %f 200 200 200\n", (x+1)*SCALE, y    *SCALE, (z+1)*SCALE);
                        fprintf(fp_ply,"%f %f %f 200 200 200\n", (x+1)*SCALE, (y+1)*SCALE, (z+1)*SCALE);
                        fprintf(fp_ply,"%f %f %f 200 200 200\n", x    *SCALE, (y+1)*SCALE, (z+1)*SCALE);
                    }

                    if (point_cloud_data(x-1,y,z) == 0) {
                        fprintf(fp_ply,"%f %f %f 200 200 200\n", x    *SCALE, y    *SCALE, z    *SCALE);
                        fprintf(fp_ply,"%f %f %f 200 200 200\n", x    *SCALE, y    *SCALE, (z+1)*SCALE);
                        fprintf(fp_ply,"%f %f %f 200 200 200\n", x    *SCALE, (y+1)*SCALE, (z+1)*SCALE);
                        fprintf(fp_ply,"%f %f %f 200 200 200\n", x    *SCALE, (y+1)*SCALE, z    *SCALE);
                    }

                    if (point_cloud_data(x+1,y,z) == 0) {
                        fprintf(fp_ply,"%f %f %f 200 200 200\n", (x+1)*SCALE, y    *SCALE, z    *SCALE);
                        fprintf(fp_ply,"%f %f %f 200 200 200\n", (x+1)*SCALE, (y+1)*SCALE, z    *SCALE);
                        fprintf(fp_ply,"%f %f %f 200 200 200\n", (x+1)*SCALE, (y+1)*SCALE, (z+1)*SCALE);
                        fprintf(fp_ply,"%f %f %f 200 200 200\n", (x+1)*SCALE, y    *SCALE, (z+1)*SCALE);
                    }

                    if (point_cloud_data(x,y-1,z) == 0) {
                        fprintf(fp_ply,"%f %f %f 200 200 200\n", x    *SCALE, y    *SCALE, z    *SCALE);
                        fprintf(fp_ply,"%f %f %f 200 200 200\n", (x+1)*SCALE, y    *SCALE, z    *SCALE);
                        fprintf(fp_ply,"%f %f %f 200 200 200\n", (x+1)*SCALE, y    *SCALE, (z+1)*SCALE);
                        fprintf(fp_ply,"%f %f %f 200 200 200\n", x    *SCALE, y    *SCALE, (z+1)*SCALE);
                    }

                    if (point_cloud_data(x,y+1,z) == 0) {
                        fprintf(fp_ply,"%f %f %f 200 200 200\n", x    *SCALE, (y+1)*SCALE, z    *SCALE);
                        fprintf(fp_ply,"%f %f %f 200 200 200\n", x    *SCALE, (y+1)*SCALE, (z+1)*SCALE);
                        fprintf(fp_ply,"%f %f %f 200 200 200\n", (x+1)*SCALE, (y+1)*SCALE, (z+1)*SCALE);
                        fprintf(fp_ply,"%f %f %f 200 200 200\n", (x+1)*SCALE, (y+1)*SCALE, z    *SCALE);
                    }
                }
            }
        }
    }

    for (int i=0;i<face_count;i++) {
        int idx = i*4;
        fprintf(fp_ply,"4 %d %d %d %d\n", idx, idx+1, idx+2, idx+3);
    }

	fclose(fp_ply);
}

// 点群データをメッシュ化してSTLファイルとして保存
void PointCloud::save_as_stl(const char* file_name) {
    FILE *fp_stl = fopen(file_name, "w");

    // write STL file header
    fprintf(fp_stl,"solid result-ascii\n");
    
    for (int z=1; z<POINTS-1; z++) {
        for (int y=1; y<POINTS-1; y++) {
            for (int x=1; x<POINTS-1; x++) {
                if (point_cloud_data(x,y,z) == 1) {

                    // STLファイルの出力
                    if (point_cloud_data(x,y,z+1) == 0) {
                        fprintf(fp_stl,"facet normal 0 0 1\n");
                        fprintf(fp_stl,"outer loop\n");
                        fprintf(fp_stl,"vertex %f %f %f\n", (x+0)*SCALE, (y+0)*SCALE, (z+1)*SCALE);
                        fprintf(fp_stl,"vertex %f %f %f\n", (x+1)*SCALE, (y+0)*SCALE, (z+1)*SCALE);
                        fprintf(fp_stl,"vertex %f %f %f\n", (x+0)*SCALE, (y+1)*SCALE, (z+1)*SCALE);
                        fprintf(fp_stl,"endloop\n");
                        fprintf(fp_stl,"endfacet\n");
                        fprintf(fp_stl,"facet normal 0 0 1\n");
                        fprintf(fp_stl,"outer loop\n");
                        fprintf(fp_stl,"vertex %f %f %f\n", (x+1)*SCALE, (y+1)*SCALE, (z+1)*SCALE);
                        fprintf(fp_stl,"vertex %f %f %f\n", (x+0)*SCALE, (y+1)*SCALE, (z+1)*SCALE);
                        fprintf(fp_stl,"vertex %f %f %f\n", (x+1)*SCALE, (y+0)*SCALE, (z+1)*SCALE);
                        fprintf(fp_stl,"endloop\n");
                        fprintf(fp_stl,"endfacet\n");
                    }

                    if (point_cloud_data(x+1,y,z) == 0) {
                        fprintf(fp_stl,"facet normal 1 0 0\n");
                        fprintf(fp_stl,"outer loop\n");
                        fprintf(fp_stl,"vertex %f %f %f\n", (x+1)*SCALE, (y+0)*SCALE, (z+1)*SCALE);
                        fprintf(fp_stl,"vertex %f %f %f\n", (x+1)*SCALE, (y+0)*SCALE, (z+0)*SCALE);
                        fprintf(fp_stl,"vertex %f %f %f\n", (x+1)*SCALE, (y+1)*SCALE, (z+1)*SCALE);
                        fprintf(fp_stl,"endloop\n");
                        fprintf(fp_stl,"endfacet\n");
                        fprintf(fp_stl,"facet normal 1 0 0\n");
                        fprintf(fp_stl,"outer loop\n");
                        fprintf(fp_stl,"vertex %f %f %f\n", (x+1)*SCALE, (y+1)*SCALE, (z+0)*SCALE);
                        fprintf(fp_stl,"vertex %f %f %f\n", (x+1)*SCALE, (y+1)*SCALE, (z+1)*SCALE);
                        fprintf(fp_stl,"vertex %f %f %f\n", (x+1)*SCALE, (y+0)*SCALE, (z+0)*SCALE);
                        fprintf(fp_stl,"endloop\n");
                        fprintf(fp_stl,"endfacet\n");
                    }

                    if (point_cloud_data(x,y,z-1) == 0) {
                        fprintf(fp_stl,"facet normal 0 0 -1\n");
                        fprintf(fp_stl,"outer loop\n");
                        fprintf(fp_stl,"vertex %f %f %f\n", (x+1)*SCALE, (y+0)*SCALE, (z+0)*SCALE);
                        fprintf(fp_stl,"vertex %f %f %f\n", (x+0)*SCALE, (y+0)*SCALE, (z+0)*SCALE);
                        fprintf(fp_stl,"vertex %f %f %f\n", (x+1)*SCALE, (y+1)*SCALE, (z+0)*SCALE);
                        fprintf(fp_stl,"endloop\n");
                        fprintf(fp_stl,"endfacet\n");
                        fprintf(fp_stl,"facet normal 0 0 -1\n");
                        fprintf(fp_stl,"outer loop\n");
                        fprintf(fp_stl,"vertex %f %f %f\n", (x+0)*SCALE, (y+1)*SCALE, (z+0)*SCALE);
                        fprintf(fp_stl,"vertex %f %f %f\n", (x+1)*SCALE, (y+1)*SCALE, (z+0)*SCALE);
                        fprintf(fp_stl,"vertex %f %f %f\n", (x+0)*SCALE, (y+0)*SCALE, (z+0)*SCALE);
                        fprintf(fp_stl,"endloop\n");
                        fprintf(fp_stl,"endfacet\n");
                    }

                    if (point_cloud_data(x-1,y,z) == 0) {
                        fprintf(fp_stl,"facet normal -1 0 0\n");
                        fprintf(fp_stl,"outer loop\n");
                        fprintf(fp_stl,"vertex %f %f %f\n", (x+0)*SCALE, (y+0)*SCALE, (z+0)*SCALE);
                        fprintf(fp_stl,"vertex %f %f %f\n", (x+0)*SCALE, (y+0)*SCALE, (z+1)*SCALE);
                        fprintf(fp_stl,"vertex %f %f %f\n", (x+0)*SCALE, (y+1)*SCALE, (z+0)*SCALE);
                        fprintf(fp_stl,"endloop\n");
                        fprintf(fp_stl,"endfacet\n");
                        fprintf(fp_stl,"facet normal -1 0 0\n");
                        fprintf(fp_stl,"outer loop\n");
                        fprintf(fp_stl,"vertex %f %f %f\n", (x+0)*SCALE, (y+1)*SCALE, (z+1)*SCALE);
                        fprintf(fp_stl,"vertex %f %f %f\n", (x+0)*SCALE, (y+1)*SCALE, (z+0)*SCALE);
                        fprintf(fp_stl,"vertex %f %f %f\n", (x+0)*SCALE, (y+0)*SCALE, (z+1)*SCALE);
                        fprintf(fp_stl,"endloop\n");
                        fprintf(fp_stl,"endfacet\n");
                    }

                    if (point_cloud_data(x,y+1,z) == 0) {
                        fprintf(fp_stl,"facet normal 0 1 0\n");
                        fprintf(fp_stl,"outer loop\n");
                        fprintf(fp_stl,"vertex %f %f %f\n", (x+0)*SCALE, (y+1)*SCALE, (z+1)*SCALE);
                        fprintf(fp_stl,"vertex %f %f %f\n", (x+1)*SCALE, (y+1)*SCALE, (z+1)*SCALE);
                        fprintf(fp_stl,"vertex %f %f %f\n", (x+0)*SCALE, (y+1)*SCALE, (z+0)*SCALE);
                        fprintf(fp_stl,"endloop\n");
                        fprintf(fp_stl,"endfacet\n");
                        fprintf(fp_stl,"facet normal 0 1 0\n");
                        fprintf(fp_stl,"outer loop\n");
                        fprintf(fp_stl,"vertex %f %f %f\n", (x+1)*SCALE, (y+1)*SCALE, (z+0)*SCALE);
                        fprintf(fp_stl,"vertex %f %f %f\n", (x+0)*SCALE, (y+1)*SCALE, (z+0)*SCALE);
                        fprintf(fp_stl,"vertex %f %f %f\n", (x+1)*SCALE, (y+1)*SCALE, (z+1)*SCALE);
                        fprintf(fp_stl,"endloop\n");
                        fprintf(fp_stl,"endfacet\n");
                    }

                    if (point_cloud_data(x,y-1,z) == 0) {
                        fprintf(fp_stl,"facet normal 0 -1 0\n");
                        fprintf(fp_stl,"outer loop\n");
                        fprintf(fp_stl,"vertex %f %f %f\n", (x+1)*SCALE, (y+0)*SCALE, (z+1)*SCALE);
                        fprintf(fp_stl,"vertex %f %f %f\n", (x+0)*SCALE, (y+0)*SCALE, (z+1)*SCALE);
                        fprintf(fp_stl,"vertex %f %f %f\n", (x+1)*SCALE, (y+0)*SCALE, (z+0)*SCALE);
                        fprintf(fp_stl,"endloop\n");
                        fprintf(fp_stl,"endfacet\n");
                        fprintf(fp_stl,"facet normal 0 -1 0\n");
                        fprintf(fp_stl,"outer loop\n");
                        fprintf(fp_stl,"vertex %f %f %f\n", (x+0)*SCALE, (y+0)*SCALE, (z+0)*SCALE);
                        fprintf(fp_stl,"vertex %f %f %f\n", (x+1)*SCALE, (y+0)*SCALE, (z+0)*SCALE);
                        fprintf(fp_stl,"vertex %f %f %f\n", (x+0)*SCALE, (y+0)*SCALE, (z+1)*SCALE);
                        fprintf(fp_stl,"endloop\n");
                        fprintf(fp_stl,"endfacet\n");
                    }
                }
            }
        }
    }
    // write STL file footer
    fprintf(fp_stl,"endsolid\n");
    fclose(fp_stl);
}

// 点群データをXYZファイルとして保存
void PointCloud::save_as_xyz(const char* file_name) {
    FILE *fp_xyz = fopen(file_name, "w");

    for (int z=1; z<POINTS-1; z++) {
        for (int y=1; y<POINTS-1; y++) {
            for (int x=1; x<POINTS-1; x++) {
                if (point_cloud_data(x,y,z) == 1) {

                    // 物体内部の点は出力しない
                    int count = 0;
                    for (int i=-1;i<2;i++) {
                        for (int j=-1;j<2;j++) {
                            for (int k=-1;k<2;k++) {
                                if (point_cloud_data((x+i),(y+j),(z+k)) == 0) count++;
                            }
                        }
                    }

                    if (count>4) {
                        // 点群データ(Point Cloud Data)の出力
                        fprintf(fp_xyz,"%f -%f %f\n", x*SCALE, y*SCALE, z*SCALE);
                    }
                }
            }
        }
    }

    fclose(fp_xyz);
}