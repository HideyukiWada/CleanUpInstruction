#ifndef ViewImage_h
#define ViewImage_h

#include <stdlib.h>
#include <assert.h>
#include <math.h>

namespace sigverse
{

	//typedef unsigned short ImageDataType;

	enum ImageDataType{
		IMAGE_DATA_TYPE_ANY = 0,
		IMAGE_DATA_WINDOWS_BMP,
	};


	//typedef unsigned short ColorBitType;

	enum ColorBitType{
		COLORBIT_ANY = 0,
		COLORBIT_24,
		DEPTHBIT_8,
	};

	enum ImageDataSize {
		IMAGE_320X240 = 0,
		IMAGE_320X1,
	};

	/**
	* @brief 画像データ情報クラス
	*/
	class ViewImageInfo
	{
	private:
		ImageDataType	m_dataType;
		ColorBitType	m_cbType;
		int		m_width;
		int		m_height;

	public:
		/**
		* @brief コンストラクタ
		*
		* @param dataType 画像データタイプ
		* @param cbType   カラービットタイプ
		* @param sz       画像サイズ
		*/
		ViewImageInfo(ImageDataType dataType, ColorBitType cbType, ImageDataSize sz) : m_dataType(dataType), m_cbType(cbType)
		{
		
			switch(sz) {
	case IMAGE_320X240:
		m_width = 320; m_height = 240;
		break;
	case IMAGE_320X1:
		m_width = 320; m_height = 1;
		break;
	default:
		assert(0);
		break;
			}
		}
		/**
		* @brief コンストラクタ
		*
		* @param dataType 画像データタイプ
		* @param cbType   カラービットタイプ
		* @param w        幅(pixel)
		* @param h        高さ(pixel)
		*/
		ViewImageInfo(ImageDataType dataType, ColorBitType cbType, int w, int h)
			: m_dataType(dataType), m_cbType(cbType), m_width(w), m_height(h)
		{
		}
		//! コピーコンストラクタ
		ViewImageInfo(const ViewImageInfo &o)
			: m_dataType(o.m_dataType),
			m_cbType(o.m_cbType),
			m_width(o.m_width), m_height(o.m_height)
		{
		}
		//! 画像データ形式(BMP etc...)を得る
		ImageDataType getDataType() const { return m_dataType; }
		//! カラータイプを得る
		ColorBitType  getColorBitType() const { return m_cbType; }
		//! 画像の幅(pixel)を得る
		int	      getWidth() const { return m_width; }
		//! 画像の高さ(pixel)を得る
		int	      getHeight() const { return m_height; }

		//! 1ピクセルあたりのデータサイズを得る
		int	getBytesPerOnePixel() const {
			int b;
			switch(m_cbType) {
	case COLORBIT_24:
		b = 3; break;
	case DEPTHBIT_8:
		b = 1; break;
	default:
		assert(0);	// error
		b = 0; break;
			}

			return b;
		}
	};


	/**
	* @brief 視覚データクラス
	*/
	class ViewImage
	{
	private:
		ViewImageInfo	m_info;
		char *		m_buf;
		int		m_buflen;
	
		double m_fov;
		double m_ar;   //aspect ratio
	public:
		/**
		*  @brief コンストラクタ
		*
		*  @param info 視覚データ情報
		*/
		ViewImage(const ViewImageInfo &info)
			: m_info(info), m_buf(0), m_buflen(0)
		{
			m_buflen = calcBufferSize(info);
			m_buf = new char [m_buflen];
		}

		//! デストラクタ
		~ViewImage() {
			if (m_buf) {
				delete [] m_buf; m_buf = 0;
			}
		}

	private:
		// sekikawa(2007/10/12)
		int getWidthBytes(int width, int bytesPerOnePixel);

		int	calcBufferSize(const ViewImageInfo &info);

	public:
		//! 視覚データ情報の取得
		const ViewImageInfo & getInfo() { return m_info; }

		//! 画像の幅(pixel)の取得
		int  getWidth() const { return m_info.getWidth(); }
		//! 画像の高さ(pixel)の取得
		int  getHeight() const { return m_info.getHeight(); }
		//! 画像データの取得
		char * getBuffer() const { return m_buf; }
		//! 画像データ設定
		void setBuffer(char *buf) { m_buf = buf; }
		//! 画像データ長の取得
		int   getBufferLength() const { return m_buflen; }
		//! 視野角設定
		void setFOVy(double fov){ m_fov = fov;} 
		//! アスペクト比設定
		void setAspectRatio(double ar){ m_ar = ar;}
		//! 視野角取得(y方向)
		double getFOVy(){ return m_fov;} 
		//! 視野角取得(x方向)
		double getFOVx(){ return 2*atan(tan(m_fov*0.5)*m_ar);} 
		//! アスペクト比取得
		double getAspectRatio(){ return m_ar;}

		//virtual void setDimension(int n){} 

        // sekikawa(2007/10/12)
        // convert RGBA format to BGR and turn y-axis upside down
        void setBitImageAsWindowsBMP(unsigned char *bitImage);

        /**                                                                                                                                                                                                  
         * @brief 画像データを Windows ビットマップ形式で保存する                                                                                                                                            
         *                                                                                                                                                                                                   
         * @param fname 保存先ファイル名                                                                                                                                                                     
         */
        bool    saveAsWindowsBMP(const char *fname);

	};
} // namespace sigverse
#endif // ViewImage_h
 
