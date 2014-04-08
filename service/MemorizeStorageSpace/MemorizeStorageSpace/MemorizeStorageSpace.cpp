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



#pragma comment(lib, "user32.lib")

struct cleanUpList{
	//片付ける物体
	std::string objectName;
	//片付け先の個数
	int storageSpaceCount;
	//片付け先の名前
	std::vector<std::string> storageSpaceName;
	//片づけた回数
	std::vector<int> cleanUpCount;
};


class MemorizeStorageSpace : public sigverse::SIGService
{

public:
	MemorizeStorageSpace(std::string name) : SIGService(name){};
	~MemorizeStorageSpace();
	double onAction();
	void onRecvMsg(sigverse::RecvMsgEvent &evt);
	void onInit();
	void readList();
	void writeList();
	bool findObject(std::string objectName);
	void addObject(std::string objectName);
	bool findStorageSpace(std::string storageSpaceName);
	void addStorageSpace(std::string storageSpaceName);
	void incrementCleanUpCount();
	
	//対応表の中身を格納する変数
	std::vector<cleanUpList> cul;

	std::string robotName;

	//対応表の名前
	std::string listName;

	//受信メッセージ用
	//収納場所
	std::string m_storageSpaceName;
	//片付ける物体
	std::string m_objectName;

	//運ぶ物体が何行目に記述されているか
	int objectIndex;
	//運び先が何個目に記述されているか
	int storageSpaceIndex;

	//物体名を受信したか
	bool recieveObject;
	//運び先を受信したか
	bool recieceStorageSpace;

};

MemorizeStorageSpace::~MemorizeStorageSpace()
{
	writeList();
	this->disconnect();
}





void MemorizeStorageSpace::onInit(){
	robotName = "robot_000";
	listName = "StorageSpaceList.txt";

	recieveObject = false;
	recieceStorageSpace = false;

	readList();
	
}



double MemorizeStorageSpace::onAction()
{
	if (recieveObject && recieceStorageSpace){
		incrementCleanUpCount();
	}

	return 0.1;
}

void MemorizeStorageSpace::onRecvMsg(sigverse::RecvMsgEvent &evt)
{
	std::string sender = evt.getSender();
	std::string msg = evt.getMsg();
	std::string s = msg;

	//ヘッダーの取り出し
	int strPos1 = 0;
	int strPos2;

	std::string headss;
	std::string tmpss;
	strPos2 = msg.find(" ", strPos1);
	headss.assign(msg, strPos1, strPos2 - strPos1);
	tmpss.assign(msg, strPos2 + 1, msg.length() - strPos2);

	if (headss == "object"){
		m_objectName = tmpss;
		
		if (!findObject(m_objectName)){
			addObject(m_objectName);
		}
		recieveObject = true;
	}
	else if (headss == "storageSpace"){
		m_storageSpaceName = tmpss;
		
		if (!findStorageSpace(m_storageSpaceName)){
			addStorageSpace(m_storageSpaceName);
		}
		recieceStorageSpace = true;
	}

	printf("Message  : %s  \n", s.c_str());
	std::wstring ws;
	printf("Sender  :  %s  \n", sender.c_str());


}

/*readList
	対応表を読み込む
*/
void MemorizeStorageSpace::readList()
{
	std::ifstream ifs(listName.c_str());

	if (!ifs){
		//ファイルが存在しなかったら作成
		std::ofstream ofs(listName.c_str());
	}
	//対応表の中身を変数に格納
	else{
		while (!ifs.eof()){
			//対応表の1行分を格納する変数
			cleanUpList tmpCul;
			
			ifs >> tmpCul.objectName;
			ifs >> tmpCul.storageSpaceCount;
			
			//片付け先の個数分繰り返す
			for (int i = 0; i < tmpCul.storageSpaceCount; i++){
				//片付け先の名前
				std::string tmpStorageSpaceName;
				//片付けた回数
				int tmpCleanUpCount;

				ifs >> tmpStorageSpaceName;
				ifs >> tmpCleanUpCount;

				tmpCul.storageSpaceName.push_back(tmpStorageSpaceName);
				tmpCul.cleanUpCount.push_back(tmpCleanUpCount);
			}
			cul.push_back(tmpCul);
		}
	}
}

/*writeList
	対応表への書き出し
*/
void MemorizeStorageSpace::writeList()
{
	//対応表を一度空にする
	std::ofstream clear(listName.c_str(), std::ios::trunc);
	
	std::ofstream ofs(listName.c_str(), std::ios::app);
	for (int i = 0; i < cul.size(); i++){
		ofs << cul[i].objectName << " ";
		ofs << cul[i].storageSpaceCount << " ";

		for (int j = 0; j < cul[i].storageSpaceCount; j++){
			ofs << cul[i].storageSpaceName[j] << " ";
			ofs << cul[i].cleanUpCount[j];
			if (j < cul[i].storageSpaceCount - 1){
				ofs << " ";
			}
		}
		if (i < cul.size() - 1){
			ofs << std::endl;
		}
	}
}

/*findObject
	引数
	　std::string objectName:運ぶ物体名
	戻り値 
	　bool
	　存在すればtrue
	　存在しなければfalseを返す

	運ぶ物体が対応表に存在するか確認
	存在する場合、物体が何行目に記述されているかをobjectIndexに格納する
*/
bool MemorizeStorageSpace::findObject(std::string objectName)
{
	for (int i = 0; i < cul.size(); i++){
		if (objectName == cul[i].objectName){
			objectIndex = i;
			return true;
		}
	}
	return false;
}

/*addObject
	引数
	　std::string objectName:運ぶ物体名

	運ぶ物体を対応表に追加
	*/
void MemorizeStorageSpace::addObject(std::string objectName)
{
	cleanUpList blankCul;
	cul.push_back(blankCul);

	objectIndex = cul.size() - 1;

	cul[objectIndex].objectName = objectName;
	cul[objectIndex].storageSpaceCount = 0;
}

/*findStorageSpace
	引数
	　std::string storageSpaceName:運び先
	戻り値
	　bool
	　運ばれたことがあればtrue
	　運ばれたことがなかったらfalseを返す

	現在の運び先に物体が運ばれたことがあるかを確認
	運ばれたことがあれば運び先が何個目に記述されているかをstorageSpaceIndexに格納
*/
bool MemorizeStorageSpace::findStorageSpace(std::string storageSpaceName)
{
	for (int i = 0; i < cul[objectIndex].storageSpaceCount - 1; i++){
		if (storageSpaceName == cul[objectIndex].storageSpaceName[i]){
			storageSpaceIndex = i;
			return true;
		}
	}
	return false;
}

/*addStorageSpace
	引数
	　std::string storageSpaceName：運び先
	物体の新規運び先を追加
*/
void MemorizeStorageSpace::addStorageSpace(std::string storageSpaceName)
{
	cul[objectIndex].storageSpaceCount++;
	cul[objectIndex].storageSpaceName.push_back(storageSpaceName);
	cul[objectIndex].cleanUpCount.push_back(0);
	storageSpaceIndex = cul[objectIndex].storageSpaceCount - 1;
}

/*incrementCleanUpCount
	物体が運ばれた回数を増やす
*/
void MemorizeStorageSpace::incrementCleanUpCount()
{
	cul[objectIndex].cleanUpCount[storageSpaceIndex]++;
	recieveObject = false;
	recieceStorageSpace = false;
}

int main(int argc, char** argv)
{
	MemorizeStorageSpace srv("Memorize_Service");
	//srv.onInit();
	unsigned short port = (unsigned short)(atoi(argv[2]));
	srv.connect(argv[1], port);
	srv.connectToViewer();
	srv.setAutoExitProc(true);
	//srv.connect("136.187.35.129", 9005);
	srv.startLoop();
	return 0;
}