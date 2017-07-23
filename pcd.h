#include <bitset>
using std::bitset;

// 点群データのパラメーター
// PCD_POINTS * PCD_SCALE が復元空間の大きさ(mm)となる
#define PCD_POINTS 200  // 点群空間の大きさ
#define PCD_SCALE 0.5   // 点群の間隔(mm)

#define point_cloud_data(x,y,z)  point_cloud_data[(x) + ((y)*PCD_POINTS) + (PCD_POINTS*PCD_POINTS*(z))]

void clear_point_cloud_data(bitset<PCD_POINTS*PCD_POINTS*PCD_POINTS> &);
void remove_edge(bitset<PCD_POINTS*PCD_POINTS*PCD_POINTS> &);
void save_as_stl(const char*, bitset<PCD_POINTS*PCD_POINTS*PCD_POINTS> &);
void save_as_ply(const char*, bitset<PCD_POINTS*PCD_POINTS*PCD_POINTS> &);
void save_as_xyz(const char*, bitset<PCD_POINTS*PCD_POINTS*PCD_POINTS> &);
