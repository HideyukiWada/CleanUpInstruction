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
	//�Еt���镨��
	std::string objectName;
	//�Еt����̌�
	int storageSpaceCount;
	//�Еt����̖��O
	std::vector<std::string> storageSpaceName;
	//�ЂÂ�����
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
	
	//�Ή��\�̒��g���i�[����ϐ�
	std::vector<cleanUpList> cul;

	std::string robotName;

	//�Ή��\�̖��O
	std::string listName;

	//��M���b�Z�[�W�p
	//���[�ꏊ
	std::string m_storageSpaceName;
	//�Еt���镨��
	std::string m_objectName;

	//�^�ԕ��̂����s�ڂɋL�q����Ă��邩
	int objectIndex;
	//�^�ѐ悪���ڂɋL�q����Ă��邩
	int storageSpaceIndex;

	//���̖�����M������
	bool recieveObject;
	//�^�ѐ����M������
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

	//�w�b�_�[�̎��o��
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
	�Ή��\��ǂݍ���
*/
void MemorizeStorageSpace::readList()
{
	std::ifstream ifs(listName.c_str());

	if (!ifs){
		//�t�@�C�������݂��Ȃ�������쐬
		std::ofstream ofs(listName.c_str());
	}
	//�Ή��\�̒��g��ϐ��Ɋi�[
	else{
		while (!ifs.eof()){
			//�Ή��\��1�s�����i�[����ϐ�
			cleanUpList tmpCul;
			
			ifs >> tmpCul.objectName;
			ifs >> tmpCul.storageSpaceCount;
			
			//�Еt����̌����J��Ԃ�
			for (int i = 0; i < tmpCul.storageSpaceCount; i++){
				//�Еt����̖��O
				std::string tmpStorageSpaceName;
				//�Еt������
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
	�Ή��\�ւ̏����o��
*/
void MemorizeStorageSpace::writeList()
{
	//�Ή��\����x��ɂ���
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
	����
	�@std::string objectName:�^�ԕ��̖�
	�߂�l 
	�@bool
	�@���݂����true
	�@���݂��Ȃ����false��Ԃ�

	�^�ԕ��̂��Ή��\�ɑ��݂��邩�m�F
	���݂���ꍇ�A���̂����s�ڂɋL�q����Ă��邩��objectIndex�Ɋi�[����
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
	����
	�@std::string objectName:�^�ԕ��̖�

	�^�ԕ��̂�Ή��\�ɒǉ�
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
	����
	�@std::string storageSpaceName:�^�ѐ�
	�߂�l
	�@bool
	�@�^�΂ꂽ���Ƃ������true
	�@�^�΂ꂽ���Ƃ��Ȃ�������false��Ԃ�

	���݂̉^�ѐ�ɕ��̂��^�΂ꂽ���Ƃ����邩���m�F
	�^�΂ꂽ���Ƃ�����Ή^�ѐ悪���ڂɋL�q����Ă��邩��storageSpaceIndex�Ɋi�[
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
	����
	�@std::string storageSpaceName�F�^�ѐ�
	���̂̐V�K�^�ѐ��ǉ�
*/
void MemorizeStorageSpace::addStorageSpace(std::string storageSpaceName)
{
	cul[objectIndex].storageSpaceCount++;
	cul[objectIndex].storageSpaceName.push_back(storageSpaceName);
	cul[objectIndex].cleanUpCount.push_back(0);
	storageSpaceIndex = cul[objectIndex].storageSpaceCount - 1;
}

/*incrementCleanUpCount
	���̂��^�΂ꂽ�񐔂𑝂₷
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