//
// OrderedPly.h
//
// Copyright (C) 2019  国立研究開発法人 産業技術総合研究所
//
// 著作権者による許可なしに，このソフトウェアおよび文書に関する
// 情報を使用，複製，第三者へ開示するあらゆる行為を禁止します．
// このソフトウェアは「現状のまま」で提供されます．明示的，暗黙的を問わず，
// 商品性，適合性を含め，このソフトウェアについてのいかなる保証もありません．
// 作者または著作権者は，契約行為，不法行為，またはそれ以外にかかわらず，
// ソフトウェアに起因または関連し，あるいはソフトウェアの使用またはその他の
// 扱いによって生じる一切の請求，損害，その他の義務について責任を負いません．
//
// Copyright (C) 2019  AIST, All rights reserved.
//
// Any using, copying, disclosing information regarding the software and
// documentation without permission of the copyright holders are prohibited.
// The software is provided "AS IS", without warranty of any kind, express or
// implied, including all implied warranties of merchantability and fitness.
// In no event shall the authors or copyright holders be liable for any claim,
// damages or other liability, whether in an action of contract, tort or 
// otherwise, arising from, out of or in connection with the software or 
// the use or other dealings in the software.
#ifndef ORDERED_PLY_H
#define ORDERED_PLY_H
#include <string>
#include <vector>
#include <array>

typedef enum PhoXiControlVersion {
  PC_VER_ANY,	// Unknown version
  PC_VER_1_1,	// PhoXi Control v1.1.x
  PC_VER_1_2	// PhoXi Control v1.2.x
} PCver;

//
// OrderedPly構造体.
//
// PhoXiが出力するPLYファイルのデータ構造. 欠落した3D点の扱いが特殊なPLY.
// ・3D点の属性は, 3D座標, 法線ベクトル, RGBカラー, 輝度, 距離, 確信度.
// ・3D点の数＝画像の幅×高さ.
// ・全ての3D点がラスタスキャン順に並び, 全点にカラーの情報が付与される.
// ・欠落した3D点は3D座標が(0,0,0), 法線ベクトルが(0,0,1)となる. 
//
struct OrderedPly {
  PCver version;		// PhoXi Controlのバージョン
  int size;			// 読み込んだ点の数
  int last;			// vertexの最後の属性. 次のvertexへの目印.

  // element vertex
  std::vector<std::array<float, 3> >  point;	   // 3次元座標(x,y,z)の列
  std::vector<std::array<float, 3> >  normal;	   // 法線ベクトル(nx,ny,nz)の列
  std::vector<std::array<u_char, 3> > color;	   // カラー(red,green,blue)の列
  std::vector<float>		      texture;	   // 輝度(Texture32)の列
  std::vector<float>		      depth;	   // 距離(Depth32)の列.
  std::vector<float>		      confidence;  // 確信度(Confidence32)の列.

  // element camera
  std::array<float, 3> view;
  std::array<float, 3> x_axis;
  std::array<float, 3> y_axis;
  std::array<float, 3> z_axis;

  // element phoxi_frame_params
  int   frame_width;
  int   frame_height;
  int   frame_index;
  float frame_start_time;
  float frame_duration;
  float frame_computation_duration;	// v1.2.x beta以降.
  float frame_transfer_duration;	// v1.2.x beta以降.

  // element camera_matrix
  float cm[9];				// v1.2.x beta以降.

  // element distortion_matrix
  float dm[14];				// v1.2.x beta以降.

  // element camera_resolution
  float width;				// v1.2.x beta以降.
  float height;				// v1.2.x beta以降.

  // element frame_binning
  float horizontal;			// v1.2.x beta以降.
  float vertical;			// v1.2.x beta以降.


  OrderedPly() : version(PC_VER_ANY), size(0), last(0),
    frame_width(0), frame_height(0),
    frame_index(0), frame_start_time(0), frame_duration(0),
    frame_computation_duration(0), frame_transfer_duration(0),
    width(0), height(0), horizontal(0), vertical(0)
  {
    for (int i = 0; i < 9;  i++) cm[i] = 0;
    for (int i = 0; i < 14; i++) dm[i] = 0;
  };
};

//
// OrderedPlyの読み込み
//
class OPlyReader {
 private:
  std::string filename;
  OrderedPly& data;
  PCver       guess;

 public:
  OPlyReader(std::string inputFile, OrderedPly& inputData);

  // ファイルを開く（PhoXi Ordered Ply形式）
  void read(void);
};

//
// OrderedPlyの書き込み
//
class OPlyWriter {
 private:
  std::string filename;
  OrderedPly& data;

 public:
  OPlyWriter(std::string outputFile, OrderedPly& inputData);

  // ファイルの保存（PhoXi Ordered Ply形式）
  void write(void);
};

#endif // ORDERED_PLY_H
