//
// OrderedPly.cpp
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
#include <vector>
#include <iostream>
#include <exception>
#include <cstdlib>
#include <cstdio>
#include <cassert>
#include "OrderedPly.h"
#include "rply/rply.h"

enum VertexProperty {
  X, Y, Z, NX, NY, NZ, RED, GREEN, BLUE, TEXTURE32, DEPTH32, CONFIDENCE32
};

enum CameraProperty {
  VIEW_PX, VIEW_PY, VIEW_PZ, X_AXISX, X_AXISY, X_AXISZ,
  Y_AXISX, Y_AXISY, Y_AXISZ, Z_AXISX, Z_AXISY, Z_AXISZ
};

enum PhoXiFrameParamsProperty {
  FRAME_WIDTH, FRAME_HEIGHT, FRAME_INDEX, FRAME_START_TIME, FRAME_DURATION,
  FRAME_COMPUTATION_DURATION, FRAME_TRANSFER_DURATION
};

enum CameraMatrixProperty {
  CM0, CM1, CM2, CM3, CM4, CM5, CM6, CM7, CM8 
};

enum DistortionMatrixProperty {
  DM0, DM1, DM2,  DM3,  DM4,  DM5,  DM6,
  DM7, DM8, DM9, DM10, DM11, DM12, DM13
};

enum CameraResolutionProperty {
  WIDTH, HEIGHT
};

enum FrameBinningProperty {
  HORIZONTAL, VERTICAL
};

  
extern "C" {
  //
  // 3次元点の属性を読み込むコールバック関数
  //
  int read_vertex(p_ply_argument argument)
  {
    double value;
    void* pdata = NULL;
    long n = -1;		// nは読み込む属性の指定

    ply_get_argument_user_data(argument, &pdata, &n);
    OrderedPly* data = static_cast<OrderedPly*>(pdata);

    assert(data != NULL);
    assert((n >= X) && (n <= CONFIDENCE32));

    value = ply_get_argument_value(argument);	// 属性値

    if ((n == X) || (n == Y) || (n == Z)) {
      data->point[data->size][n] = value;
    } else if ((n == NX) || (n == NY) || (n == NZ)) {
      data->normal[data->size][n - NX] = value;
    } else if ((n == RED) || (n == GREEN) || (n == BLUE)) {
      data->color[data->size][n - RED] = value;
    } else if (n == TEXTURE32) {
      data->texture[data->size] = value;
    } else if (n == DEPTH32) {
      data->depth[data->size] = value;
    } else {
      assert(n == CONFIDENCE32);
      data->confidence[data->size] = value;
    }

    if (n == data->last) {
      data->size++;		// 点の数を一つ増やす
    }
    return 1;
  }


  //
  // カメラの属性を読み込むコールバック関数
  //
  int read_camera(p_ply_argument argument)
  {
    double value;
    void* pdata = NULL;
    long n = -1;		// nは読み込む属性の指定

    ply_get_argument_user_data(argument, &pdata, &n);
    OrderedPly* data = static_cast<OrderedPly*>(pdata);

    assert(data != NULL);
    assert((n >= VIEW_PX) && (n <= Z_AXISZ));

    value = ply_get_argument_value(argument);	// 属性値

    if (n == VIEW_PX) {
      data->view[0] = value;
    } else if (n == VIEW_PY) {
      data->view[1] = value;
    } else if (n == VIEW_PZ) {
      data->view[2] = value;
    } else if (n == X_AXISX) {
      data->x_axis[0] = value;
    } else if (n == X_AXISY) {
      data->x_axis[1] = value;
    } else if (n == X_AXISZ) {
      data->x_axis[2] = value;
    } else if (n == Y_AXISX) {
      data->y_axis[0] = value;
    } else if (n == Y_AXISY) {
      data->y_axis[1] = value;
    } else if (n == Y_AXISZ) {
      data->y_axis[2] = value;
    } else if (n == Z_AXISX) {
      data->z_axis[0] = value;
    } else if (n == Z_AXISY) {
      data->z_axis[1] = value;
    } else {
      assert(n == Z_AXISZ);
      data->z_axis[2] = value;
    }

    return 1;
  }

  //
  // フレームの属性を読み込むコールバック関数
  //
  int read_phoxi_frame_params(p_ply_argument argument)
  {
    double value;
    void* pdata = NULL;
    long n = -1;		// nは読み込む属性の指定

    ply_get_argument_user_data(argument, &pdata, &n);
    OrderedPly* data = static_cast<OrderedPly*>(pdata);

    assert(data != NULL);
    assert((n >= FRAME_WIDTH) && (n <= FRAME_TRANSFER_DURATION));

    value = ply_get_argument_value(argument);	// 属性値

    if (n == FRAME_WIDTH)
    {
      data->frame_width = value;
    } else if (n == FRAME_HEIGHT) {
      data->frame_height = value;
    } else if (n == FRAME_INDEX) {
      data->frame_index = value;
    } else if (n == FRAME_START_TIME) {
      data->frame_start_time = value;
    } else if (n == FRAME_DURATION) {
      data->frame_duration = value;
    } else if (n == FRAME_COMPUTATION_DURATION) {
      data->frame_computation_duration = value;
    } else {
      assert(n == FRAME_TRANSFER_DURATION);
      data->frame_transfer_duration = value;
    }

    return 1;
  }

  //
  // カメラ行列の属性を読み込むコールバック関数
  //
  int read_camera_matrix(p_ply_argument argument)
  {
    double value;
    void* pdata = NULL;
    long n = -1;		// nは読み込む属性の指定

    ply_get_argument_user_data(argument, &pdata, &n);
    OrderedPly* data = static_cast<OrderedPly*>(pdata);

    assert(data != NULL);
    assert((n >= CM0) && (n <= CM8));

    value = ply_get_argument_value(argument);	// 属性値
    data->cm[n] = value;

    return 1;
  }

  //
  // 歪み行列の属性を読み込むコールバック関数
  //
  int read_distortion_matrix(p_ply_argument argument)
  {
    double value;
    void* pdata = NULL;
    long n = -1;		// nは読み込む属性の指定

    ply_get_argument_user_data(argument, &pdata, &n);
    OrderedPly* data = static_cast<OrderedPly*>(pdata);

    assert(data != NULL);
    assert((n >= DM0) && (n <= DM13));

    value = ply_get_argument_value(argument);	// 属性値
    data->dm[n] = value;

    return 1;
  }

  //
  // カメラ解像度の属性を読み込むコールバック関数
  //
  int read_camera_resolution(p_ply_argument argument)
  {
    double value;
    void* pdata = NULL;
    long n = -1;		// nは読み込む属性の指定

    ply_get_argument_user_data(argument, &pdata, &n);
    OrderedPly* data = static_cast<OrderedPly*>(pdata);

    assert(data != NULL);
    assert((n >= WIDTH) && (n <= HEIGHT));

    value = ply_get_argument_value(argument);	// 属性値

    if (n == WIDTH)
    {
      data->width = value;
    } else {
      assert(n == HEIGHT);
      data->height = value;
    }

    return 1;
  }

  //
  // フレームビニングの属性を読み込むコールバック関数
  //
  int read_frame_binning(p_ply_argument argument)
  {
    double value;
    void* pdata = NULL;
    long n = -1;		// nは読み込む属性の指定

    ply_get_argument_user_data(argument, &pdata, &n);
    OrderedPly* data = static_cast<OrderedPly*>(pdata);

    assert(data != NULL);
    assert((n >= HORIZONTAL) && (n <= VERTICAL));

    value = ply_get_argument_value(argument);	// 属性値

    if (n == HORIZONTAL)
    {
      data->horizontal = value;
    } else {
      assert(n == VERTICAL);
      data->vertical = value;
    }

    return 1;
  }

} // extern "C"

OPlyReader::OPlyReader(std::string inputFile, OrderedPly& inputData):
  filename(inputFile), data(inputData), guess(PC_VER_1_1) {}

//
// 指定のファイルを開き、点群を読み込む（Ordered PLY形式ファイル）
//
void OPlyReader::read(void)
{
  // 入力ファイルを開く
  p_ply src = ply_open(filename.c_str(), NULL, 0, NULL);
  if (!src) {
    throw std::runtime_error("error: failed to open input file. " + filename);
  }

  // ファイルヘッダの読み込み
  if (!ply_read_header(src)) {
    ply_close(src);
    throw std::runtime_error("error: failed to read file header.");
  }

  // コールバック関数の設定
  p_ply_element element = NULL;

  while ((element = ply_get_next_element(src, element))) {
    const char* element_name = NULL;
    long ninstances = 0;
    ply_get_element_info(element, &element_name, &ninstances);

    // 要素名が vertex, camera, phoxi_frame_params なら読み込み対象
    std::string en = element_name;

    if (en == "vertex") {	// 点群に頂点数(ninstances)分の容量を確保
      data.point.resize(ninstances);
      data.normal.resize(ninstances);
      data.color.resize(ninstances);
      data.texture.resize(ninstances);
      data.depth.resize(ninstances);
      data.confidence.resize(ninstances);
      data.size    = 0;		// 点の数をゼロに初期化
      data.last    = X;		// vertexの最後の要素を調べる準備
    }

    p_ply_property property = NULL;
    while ((property = ply_get_next_property(element, property))) {

      const char* property_name = NULL;
      ply_get_property_info(property, &property_name, NULL, NULL, NULL);

      // 属性名がx, y, zなら読み込み対象
      std::string pn = property_name;
      int ok;

      if ((en == "vertex") && (pn == "x")) {
        ok = ply_set_read_cb(src, "vertex", "x", read_vertex, &data, X);
	data.last = X;
      } else if ((en == "vertex") && (pn == "y")) {
        ok = ply_set_read_cb(src, "vertex", "y", read_vertex, &data, Y);
	data.last = Y;
      } else if ((en == "vertex") && (pn == "z")) {
        ok = ply_set_read_cb(src, "vertex", "z", read_vertex, &data, Z);
	data.last = Z;
      } else if ((en == "vertex") && (pn == "nx")) {
        ok = ply_set_read_cb(src, "vertex", "nx", read_vertex, &data, NX);
	data.last = NX;
      } else if ((en == "vertex") && (pn == "ny")) {
        ok = ply_set_read_cb(src, "vertex", "ny", read_vertex, &data, NY);
	data.last = NY;
      } else if ((en == "vertex") && (pn == "nz")) {
        ok = ply_set_read_cb(src, "vertex", "nz", read_vertex, &data, NZ);
	data.last = NZ;
      } else if ((en == "vertex") && (pn == "red")) {
        ok = ply_set_read_cb(src, "vertex", "red", read_vertex, &data, RED);
	data.last = RED;
      } else if ((en == "vertex") && (pn == "green")) {
        ok = ply_set_read_cb(src, "vertex", "green", read_vertex, &data, GREEN);
	data.last = GREEN;
      } else if ((en == "vertex") && (pn == "blue")) {
        ok = ply_set_read_cb(src, "vertex", "blue", read_vertex, &data, BLUE);
	data.last = BLUE;
      } else if ((en == "vertex") && (pn == "Texture32")) {
        ok = ply_set_read_cb(src, "vertex", "Texture32",
			     read_vertex, &data, TEXTURE32);
	data.last = TEXTURE32;
      } else if ((en == "vertex") && (pn == "Depth32")) {
        ok = ply_set_read_cb(src, "vertex", "Depth32",
			     read_vertex, &data, DEPTH32);
	data.last = DEPTH32;
      } else if ((en == "vertex") && (pn == "Confidence32")) {
        ok = ply_set_read_cb(src, "vertex", "Confidence32",
			     read_vertex, &data, CONFIDENCE32);
	data.last = CONFIDENCE32;
      } else if ((en == "camera") && (pn == "view_px")) {
        ok = ply_set_read_cb(src, "camera", "view_px",
			     read_camera, &data, VIEW_PX);
      } else if ((en == "camera") && (pn == "view_py")) {
        ok = ply_set_read_cb(src, "camera", "view_py",
			     read_camera, &data, VIEW_PY);
      } else if ((en == "camera") && (pn == "view_pz")) {
        ok = ply_set_read_cb(src, "camera", "view_pz",
			     read_camera, &data, VIEW_PZ);
      } else if ((en == "camera") && (pn == "x_axisx")) {
        ok = ply_set_read_cb(src, "camera", "x_axisx",
			     read_camera, &data, X_AXISX);
      } else if ((en == "camera") && (pn == "x_axisy")) {
        ok = ply_set_read_cb(src, "camera", "x_axisy",
			     read_camera, &data, X_AXISY);
      } else if ((en == "camera") && (pn == "x_axisz")) {
        ok = ply_set_read_cb(src, "camera", "x_axisz",
			     read_camera, &data, X_AXISZ);
      } else if ((en == "camera") && (pn == "y_axisx")) {
        ok = ply_set_read_cb(src, "camera", "y_axisx",
			     read_camera, &data, Y_AXISX);
      } else if ((en == "camera") && (pn == "y_axisy")) {
        ok = ply_set_read_cb(src, "camera", "y_axisy",
			     read_camera, &data, Y_AXISY);
      } else if ((en == "camera") && (pn == "y_axisz")) {
        ok = ply_set_read_cb(src, "camera", "y_axisz",
			     read_camera, &data, Y_AXISZ);
      } else if ((en == "camera") && (pn == "z_axisx")) {
        ok = ply_set_read_cb(src, "camera", "z_axisx",
			     read_camera, &data, Z_AXISX);
      } else if ((en == "camera") && (pn == "z_axisy")) {
        ok = ply_set_read_cb(src, "camera", "z_axisy",
			     read_camera, &data, Z_AXISY);
      } else if ((en == "camera") && (pn == "z_axisz")) {
        ok = ply_set_read_cb(src, "camera", "z_axisz",
			     read_camera, &data, Z_AXISZ);
      } else if ((en == "phoxi_frame_params") && (pn == "frame_width")) {
        ok = ply_set_read_cb(src, "phoxi_frame_params", "frame_width",
			     read_phoxi_frame_params, &data, FRAME_WIDTH);
      } else if ((en == "phoxi_frame_params") && (pn == "frame_height")) {
        ok = ply_set_read_cb(src, "phoxi_frame_params", "frame_height",
			     read_phoxi_frame_params, &data, FRAME_HEIGHT);
      } else if ((en == "phoxi_frame_params") && (pn == "frame_index")) {
        ok = ply_set_read_cb(src, "phoxi_frame_params", "frame_index",
			     read_phoxi_frame_params, &data, FRAME_INDEX);
      } else if ((en == "phoxi_frame_params") && (pn == "frame_start_time")) {
        ok = ply_set_read_cb(src, "phoxi_frame_params", "frame_start_time",
			     read_phoxi_frame_params, &data, FRAME_START_TIME);
      } else if ((en == "phoxi_frame_params") && (pn == "frame_duration")) {
        ok = ply_set_read_cb(src, "phoxi_frame_params", "frame_duration",
			     read_phoxi_frame_params, &data, FRAME_DURATION);
      } else if ((en == "phoxi_frame_params")
		 && (pn == "frame_computation_duration")) {
        ok = ply_set_read_cb(src, "phoxi_frame_params",
			     "frame_computation_duration",
			     read_phoxi_frame_params, &data,
			     FRAME_COMPUTATION_DURATION);
	guess = PC_VER_1_2;
      } else if ((en == "phoxi_frame_params")
		 && (pn == "frame_transfer_duration")) {
        ok = ply_set_read_cb(src, "phoxi_frame_params",
			     "frame_transfer_duration",
			     read_phoxi_frame_params, &data,
			     FRAME_TRANSFER_DURATION);
	guess = PC_VER_1_2;
      } else if ((en == "camera_matrix") && (pn == "cm0")) {
        ok = ply_set_read_cb(src, "camera_matrix", "cm0",
			     read_camera_matrix, &data, CM0);
	guess = PC_VER_1_2;
      } else if ((en == "camera_matrix") && (pn == "cm1")) {
        ok = ply_set_read_cb(src, "camera_matrix", "cm1",
			     read_camera_matrix, &data, CM1);
      } else if ((en == "camera_matrix") && (pn == "cm2")) {
        ok = ply_set_read_cb(src, "camera_matrix", "cm2",
			     read_camera_matrix, &data, CM2);
      } else if ((en == "camera_matrix") && (pn == "cm3")) {
        ok = ply_set_read_cb(src, "camera_matrix", "cm3",
			     read_camera_matrix, &data, CM3);
      } else if ((en == "camera_matrix") && (pn == "cm4")) {
        ok = ply_set_read_cb(src, "camera_matrix", "cm4",
			     read_camera_matrix, &data, CM4);
      } else if ((en == "camera_matrix") && (pn == "cm5")) {
        ok = ply_set_read_cb(src, "camera_matrix", "cm5",
			     read_camera_matrix, &data, CM5);
      } else if ((en == "camera_matrix") && (pn == "cm6")) {
        ok = ply_set_read_cb(src, "camera_matrix", "cm6",
			     read_camera_matrix, &data, CM6);
      } else if ((en == "camera_matrix") && (pn == "cm7")) {
        ok = ply_set_read_cb(src, "camera_matrix", "cm7",
			     read_camera_matrix, &data, CM7);
      } else if ((en == "camera_matrix") && (pn == "cm8")) {
        ok = ply_set_read_cb(src, "camera_matrix", "cm8",
			     read_camera_matrix, &data, CM8);
      } else if ((en == "distortion_matrix") && (pn == "dm0")) {
        ok = ply_set_read_cb(src, "distortion_matrix", "dm0",
			     read_distortion_matrix, &data, DM0);
	guess = PC_VER_1_2;
      } else if ((en == "distortion_matrix") && (pn == "dm1")) {
        ok = ply_set_read_cb(src, "distortion_matrix", "dm1",
			     read_distortion_matrix, &data, DM1);
      } else if ((en == "distortion_matrix") && (pn == "dm2")) {
        ok = ply_set_read_cb(src, "distortion_matrix", "dm2",
			     read_distortion_matrix, &data, DM2);
      } else if ((en == "distortion_matrix") && (pn == "dm3")) {
        ok = ply_set_read_cb(src, "distortion_matrix", "dm3",
			     read_distortion_matrix, &data, DM3);
      } else if ((en == "distortion_matrix") && (pn == "dm4")) {
        ok = ply_set_read_cb(src, "distortion_matrix", "dm4",
			     read_distortion_matrix, &data, DM4);
      } else if ((en == "distortion_matrix") && (pn == "dm5")) {
        ok = ply_set_read_cb(src, "distortion_matrix", "dm5",
			     read_distortion_matrix, &data, DM5);
      } else if ((en == "distortion_matrix") && (pn == "dm6")) {
        ok = ply_set_read_cb(src, "distortion_matrix", "dm6",
			     read_distortion_matrix, &data, DM6);
      } else if ((en == "distortion_matrix") && (pn == "dm7")) {
        ok = ply_set_read_cb(src, "distortion_matrix", "dm7",
			     read_distortion_matrix, &data, DM7);
      } else if ((en == "distortion_matrix") && (pn == "dm8")) {
        ok = ply_set_read_cb(src, "distortion_matrix", "dm8",
			     read_distortion_matrix, &data, DM8);
      } else if ((en == "distortion_matrix") && (pn == "dm9")) {
        ok = ply_set_read_cb(src, "distortion_matrix", "dm9",
			     read_distortion_matrix, &data, DM9);
      } else if ((en == "distortion_matrix") && (pn == "dm10")) {
        ok = ply_set_read_cb(src, "distortion_matrix", "dm10",
			     read_distortion_matrix, &data, DM10);
      } else if ((en == "distortion_matrix") && (pn == "dm11")) {
        ok = ply_set_read_cb(src, "distortion_matrix", "dm11",
			     read_distortion_matrix, &data, DM11);
      } else if ((en == "distortion_matrix") && (pn == "dm12")) {
        ok = ply_set_read_cb(src, "distortion_matrix", "dm12",
			     read_distortion_matrix, &data, DM12);
      } else if ((en == "distortion_matrix") && (pn == "dm13")) {
        ok = ply_set_read_cb(src, "distortion_matrix", "dm13",
			     read_distortion_matrix, &data, DM13);
      } else if ((en == "camera_resolution") && (pn == "width")) {
        ok = ply_set_read_cb(src, "camera_resolution", "width",
			     read_camera_resolution, &data, WIDTH);
	guess = PC_VER_1_2;
      } else if ((en == "camera_resolution") && (pn == "height")) {
        ok = ply_set_read_cb(src, "camera_resolution", "height",
			     read_camera_resolution, &data, HEIGHT);
	guess = PC_VER_1_2;
      } else if ((en == "frame_binning") && (pn == "horizontal")) {
        ok = ply_set_read_cb(src, "frame_binning", "horizontal",
			     read_frame_binning, &data, HORIZONTAL);
	guess = PC_VER_1_2;
      } else if ((en == "frame_binning") && (pn == "vertical")) {
        ok = ply_set_read_cb(src, "frame_binning", "vertical",
			     read_frame_binning, &data, VERTICAL);
	guess = PC_VER_1_2;
      } else {
	// ignore unknown elements and properties. 
	ok = 1;
      }

      if (!ok) {
	ply_close(src);
	throw std::runtime_error("error: failed to read point cloud.");
      }
    } // while (property)
  } // while (element)

  // データ本体の読み込み
  if (!ply_read(src)) {
    ply_close(src);
    throw std::runtime_error("error: failed to read data body.");
  }

  // 入力ファイルを閉じる
  if (!ply_close(src)) {
    throw std::runtime_error("error: failed to close input file.");
  }

  // 読み込まれない属性の容量を０にする
  if (data.last < CONFIDENCE32) {
    data.confidence.clear();
  }
  if (data.last < DEPTH32) {
    data.depth.clear();
  }
  if (data.last < TEXTURE32) {
    data.texture.clear();
  }
  if (data.last < RED) {
    data.color.clear();
  }
  if (data.last < NX) {
    data.normal.clear();
  }

  data.version = guess;	// このPLYを出力したPhoXi Controlのバージョン


  // 本当にOrdered PLYかチェック
  if (data.size != data.frame_width * data.frame_height) {
    throw std::runtime_error("error: size mismatch. invalid ordered ply.");
  }

  return;
}


OPlyWriter::OPlyWriter(std::string outputFile, OrderedPly& outputData):
  filename(outputFile), data(outputData) {}

//
// 指定のファイルを開き、点群を保存する（Ordered PLY形式ファイル）
//
void OPlyWriter::write(void)
{
    p_ply dst = ply_create(filename.c_str(), PLY_DEFAULT, NULL, 0, NULL);
    if (!dst) {
      throw std::runtime_error("error: failed to open output file.");
    }

    // ---------- 出力ファイルにヘッダを書き込む ----------

    // メタデータ (Width, Height)
    char obj_info[BUFSIZ] = "";
    snprintf(obj_info, BUFSIZ - 1,
	     "Photoneo PLY PointCloud ( Width = %d; Height = %d)",
	     data.frame_width, data.frame_height);
    if (!ply_add_obj_info(dst, obj_info)) {
      throw std::runtime_error("error: failed to write obj_info.");
    }

    // 点 (vertex)
    const int npoints = data.frame_width * data.frame_height;
    if (!ply_add_element(dst, "vertex", npoints)) {
      throw std::runtime_error("error: failed to write \"vertex\" element.");
    }

    if (!ply_add_property(dst, "x", PLY_FLOAT, PLY_CHAR, PLY_CHAR)) {
      throw std::runtime_error("error: failed to write \"x\" property.");
    }

    if (!ply_add_property(dst, "y", PLY_FLOAT, PLY_CHAR, PLY_CHAR)) {
      throw std::runtime_error("error: failed to write \"y\" property.");
    }

    if (!ply_add_property(dst, "z", PLY_FLOAT, PLY_CHAR, PLY_CHAR)) {
      throw std::runtime_error("error: failed to write \"z\" property.");
    }

    if (data.normal.size() > 0) {
      if (!ply_add_property(dst, "nx", PLY_FLOAT, PLY_CHAR, PLY_CHAR)) {
	throw std::runtime_error("error: failed to write \"nx\" property.");
      }

      if (!ply_add_property(dst, "ny", PLY_FLOAT, PLY_CHAR, PLY_CHAR)) {
	throw std::runtime_error("error: failed to write \"ny\" property.");
      }

      if (!ply_add_property(dst, "nz", PLY_FLOAT, PLY_CHAR, PLY_CHAR)) {
	throw std::runtime_error("error: failed to write \"nz\" property.");
      }
    }

    if (data.color.size() > 0) {
      if (!ply_add_property(dst, "red", PLY_UCHAR, PLY_CHAR, PLY_CHAR)) {
	throw std::runtime_error("error: failed to write \"red\" property.");
      }

      if (!ply_add_property(dst, "green", PLY_UCHAR, PLY_CHAR, PLY_CHAR)) {
	throw std::runtime_error("error: failed to write \"green\" property.");
      }
      
      if (!ply_add_property(dst, "blue", PLY_UCHAR, PLY_CHAR, PLY_CHAR)) {
	throw std::runtime_error("error: failed to write \"blue\" property.");
      }
    }

    if (data.texture.size() > 0) {
      if (!ply_add_property(dst, "Texture32",
			    PLY_FLOAT, PLY_CHAR, PLY_CHAR)) {
	throw std::runtime_error("error: failed to write"
				 " \"Texture32\" property.");
      }
    }

    if (data.depth.size() > 0) {
      if (!ply_add_property(dst, "Depth32",
			    PLY_FLOAT, PLY_CHAR, PLY_CHAR)) {
	throw std::runtime_error("error: failed to write"
				 " \"Depth32\" property.");
      }
    }

    if (data.confidence.size() > 0) {
      if (!ply_add_property(dst, "Confidence32",
			    PLY_FLOAT, PLY_CHAR, PLY_CHAR)) {
	throw std::runtime_error("error: failed to write"
				 " \"Confidence32\" property.");
      }
    }

    // カメラ (camera)
    if (!ply_add_element(dst, "camera", 1)) {
      throw std::runtime_error("error: failed to write \"camera\" element.");
    }

    if (!ply_add_property(dst, "view_px", PLY_FLOAT, PLY_CHAR, PLY_CHAR)) {
      throw std::runtime_error("error: failed to write \"view_px\" property.");
    }

    if (!ply_add_property(dst, "view_py", PLY_FLOAT, PLY_CHAR, PLY_CHAR)) {
      throw std::runtime_error("error: failed to write \"view_py\" property.");
    }

    if (!ply_add_property(dst, "view_pz", PLY_FLOAT, PLY_CHAR, PLY_CHAR)) {
      throw std::runtime_error("error: failed to write \"view_pz\" property.");
    }

    if (!ply_add_property(dst, "x_axisx", PLY_FLOAT, PLY_CHAR, PLY_CHAR)) {
      throw std::runtime_error("error: failed to write \"x_axisx\" property.");
    }
    
    if (!ply_add_property(dst, "x_axisy", PLY_FLOAT, PLY_CHAR, PLY_CHAR)) {
      throw std::runtime_error("error: failed to write \"x_axisy\" property.");
    }
    
    if (!ply_add_property(dst, "x_axisz", PLY_FLOAT, PLY_CHAR, PLY_CHAR)) {
      throw std::runtime_error("error: failed to write \"x_axisz\" property.");
    }
    
    if (!ply_add_property(dst, "y_axisx", PLY_FLOAT, PLY_CHAR, PLY_CHAR)) {
      throw std::runtime_error("error: failed to write \"y_axisx\" property.");
    }
    
    if (!ply_add_property(dst, "y_axisy", PLY_FLOAT, PLY_CHAR, PLY_CHAR)) {
      throw std::runtime_error("error: failed to write \"y_axisy\" property.");
    }
    
    if (!ply_add_property(dst, "y_axisz", PLY_FLOAT, PLY_CHAR, PLY_CHAR)) {
      throw std::runtime_error("error: failed to write \"y_axisz\" property.");
    }
    
    if (!ply_add_property(dst, "z_axisx", PLY_FLOAT, PLY_CHAR, PLY_CHAR)) {
      throw std::runtime_error("error: failed to write \"z_axisx\" property.");
    }
    
    if (!ply_add_property(dst, "z_axisy", PLY_FLOAT, PLY_CHAR, PLY_CHAR)) {
      throw std::runtime_error("error: failed to write \"z_axisy\" property.");
    }
    
    if (!ply_add_property(dst, "z_axisz", PLY_FLOAT, PLY_CHAR, PLY_CHAR)) {
      throw std::runtime_error("error: failed to write \"z_axisz\" property.");
    }

    // フレーム (phoxi_frame_params)
    if (!ply_add_element(dst, "phoxi_frame_params", 1)) {
      throw std::runtime_error("error: failed to write"
			       " \"phoxi_frame_params\" element.");
    }

    if (!ply_add_property(dst, "frame_width",
			  PLY_UINT, PLY_CHAR, PLY_CHAR)) {
      throw std::runtime_error("error: failed to write"
			       " \"frame_width\" property.");
    }

    if (!ply_add_property(dst, "frame_height",
			  PLY_UINT, PLY_CHAR, PLY_CHAR)) {
      throw std::runtime_error("error: failed to write"
			       " \"frame_height\" property.");
    }

    if (!ply_add_property(dst, "frame_index",
			  PLY_UINT, PLY_CHAR, PLY_CHAR)) {
      throw std::runtime_error("error: failed to write"
			       " \"frame_index\" property.");
    }

    if (!ply_add_property(dst, "frame_start_time",
			  PLY_FLOAT, PLY_CHAR, PLY_CHAR)) {
      throw std::runtime_error("error: failed to write"
			       " \"frame_start_time\" property.");
    }

    if (!ply_add_property(dst, "frame_duration",
			  PLY_FLOAT, PLY_CHAR, PLY_CHAR)) {
      throw std::runtime_error("error: failed to write"
			       " \"frame_duration\" property.");
    }

    if (data.version >= PC_VER_1_2) {
      if (!ply_add_property(dst, "frame_computation_duration",
			    PLY_FLOAT, PLY_CHAR, PLY_CHAR)) {
	throw std::runtime_error("error: failed to write"
				 " \"frame_computation_duration\" property.");
      }

      if (!ply_add_property(dst, "frame_transfer_duration",
			    PLY_FLOAT, PLY_CHAR, PLY_CHAR)) {
	throw std::runtime_error("error: failed to write"
				 " \"frame_transfer_duration\" property.");
      }
    }

    // カメラ行列 (camera_matrix)
    if (data.version >= PC_VER_1_2) {
      if (!ply_add_element(dst, "camera_matrix", 1)) {
	throw std::runtime_error("error: failed to write"
				 " \"camera_matrix\" element.");
      }

      if (!ply_add_property(dst, "cm0", PLY_FLOAT, PLY_CHAR, PLY_CHAR)) {
	throw std::runtime_error("error: failed to write \"cm0\" property.");
      }

      if (!ply_add_property(dst, "cm1", PLY_FLOAT, PLY_CHAR, PLY_CHAR)) {
	throw std::runtime_error("error: failed to write \"cm1\" property.");
      }

      if (!ply_add_property(dst, "cm2", PLY_FLOAT, PLY_CHAR, PLY_CHAR)) {
	throw std::runtime_error("error: failed to write \"cm2\" property.");
      }

      if (!ply_add_property(dst, "cm3", PLY_FLOAT, PLY_CHAR, PLY_CHAR)) {
	throw std::runtime_error("error: failed to write \"cm3\" property.");
      }

      if (!ply_add_property(dst, "cm4", PLY_FLOAT, PLY_CHAR, PLY_CHAR)) {
	throw std::runtime_error("error: failed to write \"cm4\" property.");
      }

      if (!ply_add_property(dst, "cm5", PLY_FLOAT, PLY_CHAR, PLY_CHAR)) {
	throw std::runtime_error("error: failed to write \"cm5\" property.");
      }

      if (!ply_add_property(dst, "cm6", PLY_FLOAT, PLY_CHAR, PLY_CHAR)) {
	throw std::runtime_error("error: failed to write \"cm6\" property.");
      }

      if (!ply_add_property(dst, "cm7", PLY_FLOAT, PLY_CHAR, PLY_CHAR)) {
	throw std::runtime_error("error: failed to write \"cm7\" property.");
      }

      if (!ply_add_property(dst, "cm8", PLY_FLOAT, PLY_CHAR, PLY_CHAR)) {
	throw std::runtime_error("error: failed to write \"cm8\" property.");
      }
    }

    // 歪み行列 (distortion_matrix)
    if (data.version >= PC_VER_1_2) {
      if (!ply_add_element(dst, "distortion_matrix", 1)) {
	throw std::runtime_error("error: failed to write"
				 " \"distortion_matrix\" element.");
      }

      if (!ply_add_property(dst, "dm0", PLY_FLOAT, PLY_CHAR, PLY_CHAR)) {
	throw std::runtime_error("error: failed to write \"dm0\" property.");
      }

      if (!ply_add_property(dst, "dm1", PLY_FLOAT, PLY_CHAR, PLY_CHAR)) {
	throw std::runtime_error("error: failed to write \"dm1\" property.");
      }

      if (!ply_add_property(dst, "dm2", PLY_FLOAT, PLY_CHAR, PLY_CHAR)) {
	throw std::runtime_error("error: failed to write \"dm2\" property.");
      }

      if (!ply_add_property(dst, "dm3", PLY_FLOAT, PLY_CHAR, PLY_CHAR)) {
	throw std::runtime_error("error: failed to write \"dm3\" property.");
      }

      if (!ply_add_property(dst, "dm4", PLY_FLOAT, PLY_CHAR, PLY_CHAR)) {
	throw std::runtime_error("error: failed to write \"dm4\" property.");
      }

      if (!ply_add_property(dst, "dm5", PLY_FLOAT, PLY_CHAR, PLY_CHAR)) {
	throw std::runtime_error("error: failed to write \"dm5\" property.");
      }

      if (!ply_add_property(dst, "dm6", PLY_FLOAT, PLY_CHAR, PLY_CHAR)) {
	throw std::runtime_error("error: failed to write \"dm6\" property.");
      }

      if (!ply_add_property(dst, "dm7", PLY_FLOAT, PLY_CHAR, PLY_CHAR)) {
	throw std::runtime_error("error: failed to write \"dm7\" property.");
      }

      if (!ply_add_property(dst, "dm8", PLY_FLOAT, PLY_CHAR, PLY_CHAR)) {
	throw std::runtime_error("error: failed to write \"dm8\" property.");
      }

      if (!ply_add_property(dst, "dm9", PLY_FLOAT, PLY_CHAR, PLY_CHAR)) {
	throw std::runtime_error("error: failed to write \"dm9\" property.");
      }

      if (!ply_add_property(dst, "dm10", PLY_FLOAT, PLY_CHAR, PLY_CHAR)) {
	throw std::runtime_error("error: failed to write \"dm10\" property.");
      }

      if (!ply_add_property(dst, "dm11", PLY_FLOAT, PLY_CHAR, PLY_CHAR)) {
	throw std::runtime_error("error: failed to write \"dm11\" property.");
      }

      if (!ply_add_property(dst, "dm12", PLY_FLOAT, PLY_CHAR, PLY_CHAR)) {
	throw std::runtime_error("error: failed to write \"dm12\" property.");
      }

      if (!ply_add_property(dst, "dm13", PLY_FLOAT, PLY_CHAR, PLY_CHAR)) {
	throw std::runtime_error("error: failed to write \"dm13\" property.");
      }
    }

    // カメラ解像度 (camera_resolution)
    if (data.version >= PC_VER_1_2) {
      if (!ply_add_element(dst, "camera_resolution", 1)) {
	throw std::runtime_error("error: failed to write"
				 " \"camera_resolution\" element.");
      }

      if (!ply_add_property(dst, "width", PLY_FLOAT, PLY_CHAR, PLY_CHAR)) {
	throw std::runtime_error("error: failed to write"
				 " \"width\" property.");
      }

      if (!ply_add_property(dst, "height", PLY_FLOAT, PLY_CHAR, PLY_CHAR)) {
	throw std::runtime_error("error: failed to write"
				 " \"height\" property.");
      }
    }
    
    // フレームビニング (frame_bining)
    if (data.version >= PC_VER_1_2) {
      if (!ply_add_element(dst, "frame_binning", 1)) {
	throw std::runtime_error("error: failed to write"
				 " \"frame_binning\" element.");
      }

      if (!ply_add_property(dst, "horizontal",
			    PLY_FLOAT, PLY_CHAR, PLY_CHAR)) {
	throw std::runtime_error("error: failed to write"
				 " \"horizontal\" property.");
      }

      if (!ply_add_property(dst, "vertical",
			    PLY_FLOAT, PLY_CHAR, PLY_CHAR)) {
	throw std::runtime_error("error: failed to write"
				 " \"vertical\" property.");
      }
    }
    
    // ヘッダの出力
    if (!ply_write_header(dst)) {
      throw std::runtime_error("error: failed to write ply header.");
    }
    
    // ---------- 出力ファイルにデータ本体を書き込む ----------

    // vertex
    for (int i = 0; i < npoints; i++) {
      const float x = data.point[i][X];
      const float y = data.point[i][Y];
      const float z = data.point[i][Z];
      ply_write(dst, x);
      ply_write(dst, y);
      ply_write(dst, z);
      
      if (data.normal.size() > 0) {
	const float nx = data.normal[i][0];
	const float ny = data.normal[i][1];
	const float nz = data.normal[i][2];
	ply_write(dst, nx);
	ply_write(dst, ny);
	ply_write(dst, nz);
      }

      if (data.color.size() > 0) {
        const u_char red   = data.color[i][0];
        const u_char green = data.color[i][1];
        const u_char blue  = data.color[i][2];
	ply_write(dst, red);
	ply_write(dst, green);
	ply_write(dst, blue);
      }

      if (data.texture.size() > 0) {
	const float intensity = data.texture[i];
	ply_write(dst, intensity);
      }

      if (data.depth.size() > 0) {
	const float range = data.depth[i];
	ply_write(dst, range);
      }

      if (data.confidence.size() > 0) {
	const float conf = data.depth[i];
	ply_write(dst, conf);
      }
    }

    // camera
    ply_write(dst, data.view[0]);	// view_px
    ply_write(dst, data.view[1]);	// view_py
    ply_write(dst, data.view[2]);	// view_pz
    ply_write(dst, data.x_axis[0]);	// x_axisx
    ply_write(dst, data.x_axis[1]);	// x_axisy
    ply_write(dst, data.x_axis[2]);	// x_axisz
    ply_write(dst, data.y_axis[0]);	// y_axisx
    ply_write(dst, data.y_axis[1]);	// y_axisy
    ply_write(dst, data.y_axis[2]);	// y_axisz
    ply_write(dst, data.z_axis[0]);	// z_axisx
    ply_write(dst, data.z_axis[1]);	// z_axisy
    ply_write(dst, data.z_axis[2]);	// z_axisz

    // phoxi_frame_params
    ply_write(dst, data.frame_width);
    ply_write(dst, data.frame_height);
    ply_write(dst, data.frame_index);
    ply_write(dst, data.frame_start_time);
    ply_write(dst, data.frame_duration);
    if (data.version >= PC_VER_1_2) {
      ply_write(dst, data.frame_computation_duration);
      ply_write(dst, data.frame_transfer_duration);
    }

    // camera_matrix
    if (data.version >= PC_VER_1_2) {
      ply_write(dst, data.cm[0]);
      ply_write(dst, data.cm[1]);
      ply_write(dst, data.cm[2]);
      ply_write(dst, data.cm[3]);
      ply_write(dst, data.cm[4]);
      ply_write(dst, data.cm[5]);
      ply_write(dst, data.cm[6]);
      ply_write(dst, data.cm[7]);
      ply_write(dst, data.cm[8]);
    }

    // distortion_matrix
    if (data.version >= PC_VER_1_2) {
      ply_write(dst, data.dm[0]);
      ply_write(dst, data.dm[1]);
      ply_write(dst, data.dm[2]);
      ply_write(dst, data.dm[3]);
      ply_write(dst, data.dm[4]);
      ply_write(dst, data.dm[5]);
      ply_write(dst, data.dm[6]);
      ply_write(dst, data.dm[7]);
      ply_write(dst, data.dm[8]);
      ply_write(dst, data.dm[9]);
      ply_write(dst, data.dm[10]);
      ply_write(dst, data.dm[11]);
      ply_write(dst, data.dm[12]);
      ply_write(dst, data.dm[13]);
    }

    // camera_resolution
    if (data.version >= PC_VER_1_2) {
      ply_write(dst, data.width);
      ply_write(dst, data.height);
    }

    // frame_binning
    if (data.version >= PC_VER_1_2) {
      ply_write(dst, data.horizontal);
      ply_write(dst, data.vertical);
    }

    // 後片付け
    if (!ply_close(dst)) {
      throw std::runtime_error("error: failed to close output file.");
    }

    return;
}
