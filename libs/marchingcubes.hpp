/*
** Polygonising a scalar field
** 
** Also known as: "3D Contouring", "Marching Cubes", "Surface Reconstruction"
** Written by Paul Bourke
** May 1994
** http://paulbourke.net/geometry/polygonise/
*/

typedef struct {
    double x;
    double y;
    double z;
} XYZ;

typedef struct {
   XYZ p[3];
} TRIANGLE;

typedef struct {
   XYZ p[8];
   double val[8];
} GRIDCELL;

int Polygonise(GRIDCELL ,double ,TRIANGLE *);
XYZ VertexInterp(double, XYZ, XYZ, double, double);
