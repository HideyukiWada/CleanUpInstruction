#ifndef __SGV_MESSAGE__
#define __SGV_MESSAGE__

#include "SgvSocket.h"
#include "ViewImage.h"
#include <vector>
#include <map>
#include <string>


namespace sigverse
{
	class RecvMsgEvent
	{
	private:
		std::string   m_from;
		std::string   m_msg;
	public:
		bool  setData(std::string data, int size);

		//! 送信元エージェント名を取得する                                                                
		const char *getSender() { return m_from.c_str(); }

		//! i番目の文字列の取得                                                                           
		const char *getMsg() { return m_msg.c_str(); }
	};

	class SIGService
	{
	public:
		SIGService(std::string name);
		SIGService();
		virtual ~SIGService();

		//! メッセージ送信 
		bool sendMsg(std::string to, std::string msg);

		//! サーバを介さずエンティティコントローラにメッセージを直接送信
		//! サービスに接続済みのエンティティにのみ送信可
		bool sendMsgToCtr(std::string to, std::string msg);

		//! メッセージ送信(相手が複数) 
		bool sendMsg(std::vector<std::string> to, std::string msg);

		//! サーバに接続
		bool connect(std::string host, int port);

		//! サーバと切断
		bool disconnect();

		//! コントローラとの接続を切断する(エンティティ名指定)
		bool disconnectFromController(std::string entityName);

		//! 接続中の全コントローラと切断する
		void disconnectFromAllController();

		//! ビューワーとの接続を切断する
		void disconnectFromViewer();

		//! データ受信ループを開始する(メッセージ受信コントローラからの接続リクエスト開始）
		//@ time ループ時間(sec), -1.0は無限
		void startLoop(double time = -1.0);

		//! サーバ、コントローラ、ビューワーからデータを受信しているかどうかチェックする(usecマイクロ秒間)
		bool checkRecvData(int usec);

		//! ビューワーに接続する
		//@ 接続するにはビューワーを立ち上げておく必要がある
		bool connectToViewer();

		//! ビューワーに接続中か
		bool getIsConnectedView(){return m_connectedView;}

		/** 
		* @brief エンティティが持つカメラ画像データをビューワーから取得
		* @param entityName       カメラを持つエンティティ名
		* @param camID            カメラのID番号
		* @param cbit             ビット深度
		* @param size             イメージサイズ
		*/
		ViewImage* captureView(std::string entityName, 
			int camID = 1,
			ColorBitType cbit = COLORBIT_24, 
			ImageDataSize size = IMAGE_320X240);

		/** 
		* @brief エンティティが持つカメラから視線方向にあるオブジェクトまでの距離を取得
		* @param entityName       持つエンティティ名
		* @param offset           距離データoffset値
		* @param range            距離データ取得可能範囲
		* @param camID            カメラのID番号
		* @param cbit             ビット深度
		*/
		unsigned char distanceSensor(std::string entityName, 
			double offset = 0.0, 
			double range = 255.0, 
			int camID = 1,
			ColorBitType cbit = DEPTHBIT_8
			);

		/** 
		* @brief エンティティが持つカメラから水平面方向の距離データ取得
		* @param entityName       エンティティ名
		* @param offset           距離データoffset値
		* @param range            距離データ取得可能範囲
		* @param camID            カメラのID番号
		* @param cbit             ビット深度
		* @param size             データサイズ
		*/
		ViewImage* distanceSensor1D(std::string entityName, 
			double offset = 0.0, 
			double range = 255.0, 
			int camID = 1,
			ColorBitType cbit = DEPTHBIT_8,
			sigverse::ImageDataSize size = IMAGE_320X1);

		/** 
		* @brief エンティティが持つカメラから2次元距離データ取得
		* @param entityName       エンティティ名
		* @param offset           距離データoffset値
		* @param range            距離データ取得可能範囲
		* @param camID            カメラのID番号
		* @param cbit             ビット深度
		* @param size             データサイズ
		*/
		ViewImage* distanceSensor2D(std::string entityName, 
			double offset = 0.0, 
			double range = 255.0, 
			int camID = 1,
			ColorBitType cbit = DEPTHBIT_8,
			sigverse::ImageDataSize size = IMAGE_320X240);


		/** 
		* @brief 補正前の深度画像(zbuffer)取得
		* @param entityName       エンティティ名
		* @param offset           距離データoffset値
		* @param range            距離データ取得可能範囲
		* @param camID            カメラのID番号
		* @param cbit             ビット深度
		* @param size             データサイズ
		*/
		ViewImage* getDepthImage(std::string entityName, 
			double offset = 0.0, 
			double range = 255.0, 
			int camID = 1,
			ColorBitType cbit = DEPTHBIT_8,
			sigverse::ImageDataSize size = IMAGE_320X240);


		//! 自分のサービス名を取得する
		std::string getName() { return m_name; }

		//! 自分のサービス名を設定する
		void setName(std::string name) { m_name = name; }

		//! サーバに接続中のサービスの数を取得
		int getNewServiceNum() { return (int)m_elseServices.size() ; }

		//! サーバに接続中の他のサービス名をすべて取得する
		std::vector<std::string> getAllOtherServices() { return m_elseServices; }

		//! 接続されているエンティティの名前を取得
		std::vector<std::string> getAllConnectedEntitiesName() {
			return m_entitiesName;
		}

		//! サーバに接続されている他のサービス名を取得する(一度取得すると開放)
		/*
		std::string getElseServiceName() {
			int size = (int)m_elseServices.size(); 
			std::string name = m_elseServices[size - 1];
			m_elseServices.pop_back();
			return name;
		}
		*/

		//! サーバに接続されている他のサービスを追加する
		//void setElseServiceName(std::string name) {
		//m_elseServices.push_back(name);
		//}

		SgvSocket* getSgvSocket(){
			return m_sgvsock;
		}

		//! コントローラのSgvSocketを取得する
		SgvSocket* getControllerSocket(std::string name){
			std::map<std::string, sigverse::SgvSocket*>::iterator it;
			it = m_consocks.find(name);
			// SgvSocketが見つかった
			if(it != m_consocks.end())
				return (*it).second;
			// SgvSocketが見つからなかった
			else
				return NULL;
		}

		//! 接続中のコントローラの数を取得
		int getConnectedControllerNum() {
			return (int)m_consocks.size();
		}

		//! ビューワーがdisconnectしたときに自動的にループから抜ける
		void setAutoExitLoop(bool flag){m_autoExitLoop = flag;}

		//! ビューワーがdisconnectしたときに自動的にプロセスを終了する
		void setAutoExitProc(bool flag){m_autoExitProc = flag;}

		//! 最初に呼び出される
		virtual void onInit(){}

		//! メッセージ受信イベントハンドラ
		virtual void onRecvMsg(RecvMsgEvent &evt) {}

		//! 定期的に呼び出される
		virtual double onAction() { return 0.0;}

	private:
		ViewImage* getDistanceImage(std::string entityName, 
			double offset, 
			double range, 
			int camID,
			int dimension,
			ColorBitType cbit,
			sigverse::ImageDataSize size);


	protected:
		//! socket(for server)
		sigverse::SgvSocket *m_sgvsock;

		//! socket(for viewer)
		sigverse::SgvSocket *m_viewsock;

		//! エンティティの名前とコントローラ用ソケットの対応
		std::map<std::string, SgvSocket*> m_consocks;

		//! 接続中の全エンティティ名
		std::vector<std::string> m_entitiesName;

		//! name
		std::string m_name;

		//! 他のサービスの名前
		std::vector<std::string> m_elseServices;

		//! 接続先サーバホスト名
		std::string m_host;

		//!サービスがスタートしているかどうか
		bool m_start;

		//!コントローラが接続中か
		bool m_connected;

		//! ビューワーと接続中か
		bool m_connectedView;

		//! 自動的にループを抜けるかどうか
		bool m_autoExitLoop;

		//! 自動的にサービスプロバイダを終了するかどうか
		bool m_autoExitProc;

		//! ループ中かどうか
		bool m_onLoop;
	};
}

#endif
