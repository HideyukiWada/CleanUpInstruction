#ifndef __SGVSOCKET__
#define __SGVSOCKET__

#include <winsock2.h>
//#include "SgvLog.h"

namespace sigverse
{
	class SgvSocket
	{
	public:

		SgvSocket();
		SgvSocket(SOCKET sock);

		~SgvSocket();

		//! socket 初期化
		bool initWinsock();

		//! socket 終了
		void finalWinsock(){WSACleanup();}

		//! 接続
		bool connectTo(const char *hostname, int port);

		//! データ送信
		bool sendData( const char *msg, int size);

		//! データ受信
		bool recvData( char *msg, int size);

		//! ソケット取得
		SOCKET getSocket() { return m_sock; }


	private:
		// ソケット
		SOCKET m_sock;

		// ログ
		//Sgv::LogPtr mLog;

	};
}

#endif