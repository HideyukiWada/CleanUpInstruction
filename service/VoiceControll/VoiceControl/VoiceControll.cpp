//#include <sapi.h>
//#include <sphelper.h>
#include <string>
#include <iostream>
#include <fstream>
#include "SIGService.h"

//includes for Shared memory
#include <Windows.h>
#include <tchar.h>
#include <stdio.h>
#include <conio.h>
#include <mmsystem.h>
#include "MaltiSpeak.h"



#pragma comment(lib, "user32.lib")
#pragma comment(lib,"winmm.lib")

bool Enable;

namespace WordCodeConvert{
	/* convUtf8toSJis
	* 文字コード変換のため外部から入手したソース
	* 基本的に使用しない
	*/
	bool convUtf8toSJis(BYTE* pSource, BYTE* pDist, int* pSize){
		*pSize = 0;

		//UTF-8からUTF-16へ変換
		const int nSize = ::MultiByteToWideChar(CP_UTF8, 0, (LPCSTR)pSource, -1, NULL, 0);

		BYTE* buffUtf16 = new BYTE[nSize * 2 + 2];
		::MultiByteToWideChar(CP_UTF8, 0, (LPCSTR)pSource, -1, (LPWSTR)buffUtf16, nSize);

		//UTF-16からShift-JISへ変換
		const int nSizeSJis = ::WideCharToMultiByte(CP_ACP, 0, (LPCWSTR)buffUtf16, -1, NULL, 0, NULL, NULL);
		if (!pDist){
			*pSize = nSizeSJis;
			delete buffUtf16;
			return TRUE;
		}

		BYTE* buffSJis = new BYTE[nSizeSJis * 2];
		ZeroMemory(buffSJis, nSizeSJis * 2);
		::WideCharToMultiByte(CP_ACP, 0, (LPCWSTR)buffUtf16, -1, (LPSTR)buffSJis, nSizeSJis, NULL, NULL);

		*pSize = strlen((const char *)buffSJis);
		memcpy(pDist, buffSJis, *pSize);

		delete buffUtf16;
		delete buffSJis;

		return TRUE;
	}

	/* utf8toSjis
	* 引数 std::string 戻り値 std::string に変更したもの
	* 特別なことが無い限りこの関数を用いる
	* UTF8の文字列をShift-JISに変換する
	*/
	std::string utf8toSjis(std::string utf8_string){
		int nSize = 0;
		const char* constchar = utf8_string.c_str();
		BYTE* pSource = (BYTE*)constchar;
		convUtf8toSJis(pSource, NULL, &nSize);
		BYTE* pDist = new BYTE[nSize + 1];
		ZeroMemory(pDist, nSize + 1);
		convUtf8toSJis(pSource, pDist, &nSize);
		std::string returnString((char*)pDist);
		return returnString;
	}
}

class VoiceRecognition : public sigverse::SIGService
{

public:
	VoiceRecognition(std::string name) : SIGService(name){};
	~VoiceRecognition();
	double onAction();
	void onRecvMsg(sigverse::RecvMsgEvent &evt);
	void  onInit();
	std::string japaneseMessage2englishMessage(std::string japaneseMessage);
	std::string englishMessage2japaneseMessage(std::string englishMessage);
	std::string preview_string;
	MaltiSpeak ms;
	std::string m_pointedObject;
	std::string m_pointedtrash;
	std::string robotName;

	std::string storageSpaceName0;	//テーブル
	std::string storageSpaceName1;	//キッチン
	std::string storageSpaceName2;	//ワゴン
	std::string storageSpaceName3;	//本棚
	std::string storageSpaceName4;	//食器棚
	std::string storageSpaceName5;	//棚
};

VoiceRecognition::~VoiceRecognition()
{

	this->disconnect();
}

/*japaneseMessage2englishMessage
*引数std::string  戻り値std::string
*SIGVerse内で全角が使えないので送信メッセージを日本語から英語に置き換える
*/
std::string VoiceRecognition::japaneseMessage2englishMessage(std::string japaneseMessage){
	std::string ja_str = japaneseMessage;
	std::string en_str;

	//クリーンナップ開始
	if (std::string::npos != ja_str.find("かたづけ") || std::string::npos != ja_str.find("片づけ")) en_str = "go";
	else if (ja_str == "始めます") en_str = "go";

	//把持対象の指定
	else if (std::string::npos != ja_str.find("机") || std::string::npos != ja_str.find("黒")) en_str = storageSpaceName0;
	//else if (ja_str == "これをとって") en_str = "take";

	//捨てるor置く対象の指定
	else if (std::string::npos != ja_str.find("キッチン")) en_str = storageSpaceName1;
	//else if (ja_str == "これに捨てて") en_str = "put";
	//else if (ja_str == "そこに捨てて") en_str = "put";
	
	else if (std::string::npos != ja_str.find("ワゴン")) en_str = storageSpaceName2;

	else if (std::string::npos != ja_str.find("白") || std::string::npos != ja_str.find("本")) en_str = storageSpaceName3;
	//else if (ja_str == "ここにおいて") en_str = "put";

	else if (std::string::npos != ja_str.find("青") || std::string::npos != ja_str.find("食器")) en_str = storageSpaceName4;

	else if (std::string::npos != ja_str.find("茶") || std::string::npos != ja_str.find("棚")) en_str = storageSpaceName5;

	//捨てるor置く対象を間違った場合やり直す
	else if (ja_str == "終わります") en_str = "finish";	

	//特定の命令以外の場合
	else {
		en_str = "error";
		PlaySound("MySound.wav", NULL, SND_FILENAME | SND_ASYNC);
	}
	return en_str;
}

/*englishMessage2japaneseMessage
*引数std::string  戻り値std::string
*OpenJtalkに日本語で発話させたいので受信メッセージを英語から日本語に置き換える
*/
std::string VoiceRecognition::englishMessage2japaneseMessage(std::string englishMessage){
	std::string en_str = englishMessage;
	std::string ja_str;

	if (en_str == "start")
	{
		ja_str = "それでは片づけを始めます";
	}
	else if (en_str == "please pass my hand")
	{
		ja_str = "片付けたい物体を手渡してください";
	}
	/*else if (en_str == "Message is not accepted")
	{
		ja_str = "聞き取れません。";
	}*/
	else if (en_str == "please choose the trashbox")
	{
		ja_str = "持っていく先を教えてください";
	}
	else if (en_str == "Clock") 
	{
		m_pointedObject = "時計";
		ja_str = m_pointedObject + "、を置きます";
	}
	else if (en_str == "Bear") 
	{
		m_pointedObject = "熊のぬいぐるみ";
		ja_str = m_pointedObject + "、を置きます";
	}
	else if (en_str == "Penguin")
	{
		m_pointedObject = "おもちゃのペンギン";
		ja_str = "、" +m_pointedObject + "、を置きます";
	}
	else if (en_str == "Cup")
	{
		m_pointedObject = "カップ";
		ja_str = "、" + m_pointedObject + "、を置きます";
	}
	else if (en_str == "finish")
	{
		ja_str = "綺麗に片付きました";
	}
	else if (en_str == "Please show me which trash box to use")
	{
		ja_str = m_pointedObject + "、を運ぶ先を教えて下さい";
	}
	else if (en_str == storageSpaceName0)
	{
		m_pointedtrash = storageSpaceName0;
		ja_str = "分かりました、" + m_pointedObject + "を、" + "テーブル" + "、に置きに行きます";
	}
	else if (en_str == storageSpaceName1)
	{
		m_pointedtrash = storageSpaceName1;
		ja_str = "分かりました、" + m_pointedObject + "を、" + "キッチン" + "、に置きに行きます";
	}
	else if (en_str == storageSpaceName2)
	{
		m_pointedtrash = storageSpaceName2;
		ja_str = "分かりました、" + m_pointedObject + "を、" + "ワゴン" + "、に置きに行きます";
	}
	else if (en_str == storageSpaceName3)
	{
		m_pointedtrash = storageSpaceName3;
		ja_str = "分かりました、" + m_pointedObject + "を、" + "本棚" + "、に置きに行きます";
	}
	else if (en_str == storageSpaceName4)
	{
		m_pointedtrash = storageSpaceName4;
		ja_str = "分かりました、" + m_pointedObject + "を、" + "食器棚" + "、に置きに行きます";
	}
	else if (en_str == storageSpaceName5)
	{
		m_pointedtrash = storageSpaceName5;
		ja_str = "分かりました、" + m_pointedObject + "を、" + "棚" + "、に置きに行きます";
	}
	return ja_str;
}


void VoiceRecognition::onInit(){
	robotName = "sobit";

	ShellExecute(NULL, "open", "juliusbat\\julius-run.bat", NULL, NULL, SW_SHOW);

	std::ofstream ofs("juliusbat\\sentence.txt");
	ofs.clear(); //前回文字列削除
	ofs.close();

	Enable = true;
	preview_string = "";
	
	storageSpaceName0 = "Table";
	storageSpaceName1 = "Kitchen";
	storageSpaceName2 = "Wagon";
	storageSpaceName3 = "BookShelf";
	storageSpaceName4 = "CupBoard";
	storageSpaceName5 = "Shelf";

	ms.initialize();
}



double VoiceRecognition::onAction()
{
	if (Enable)
	{
		std::ifstream readfile("juliusbat\\sentence.txt"); //julius認識文字列を取得
		std::string str;
		std::getline(readfile, str); //1行読み込み
		readfile.close();

		str = WordCodeConvert::utf8toSjis(str); //文字コードの変換

		size_t pos;
		while ((pos = str.find_first_of(" 　、。")) != std::string::npos){ //文字列内の半角・全角スペース，句読点を削除
			str.erase(pos, 1);
		}
		if (str != preview_string){
			std::string send_msg = japaneseMessage2englishMessage(str);//メッセージの置き換え

			this->sendMsg(robotName, send_msg.c_str());
			preview_string = str;
		}
	}
	return 0.1;
}

void VoiceRecognition::onRecvMsg(sigverse::RecvMsgEvent &evt)
{
	std::string sender = evt.getSender();
	std::string msg = evt.getMsg();
	std::string s = msg;

	printf("Message  : %s  \n", s.c_str());
	std::wstring ws;
	printf("Sender  :  %s  \n", sender.c_str());

	if (strcmp(s.c_str(), "Stop_Reco") == 0)
	{
		Enable = false;
	}
	else if (strcmp(s.c_str(), "Start_Reco") == 0)
	{
		Enable = true;
	}
	ms.noWaitSpeak(englishMessage2japaneseMessage(s));//発話
}

int main(int argc, char** argv)
{
	VoiceRecognition srv("VoiceReco_Service");
	//srv.onInit();
	unsigned short port = (unsigned short)(atoi(argv[2]));
	srv.connect(argv[1], port);
	srv.connectToViewer();
	srv.setAutoExitProc(true);
	//srv.connect("136.187.35.129", 9005);
	srv.startLoop();
	return 0;
}