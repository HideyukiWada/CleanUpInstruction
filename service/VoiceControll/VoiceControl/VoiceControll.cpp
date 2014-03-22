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
#include "MaltiSpeak.h"



#pragma comment(lib, "user32.lib")

bool Enable;

namespace WordCodeConvert{
	/* convUtf8toSJis
	* �����R�[�h�ϊ��̂��ߊO��������肵���\�[�X
	* ��{�I�Ɏg�p���Ȃ�
	*/
	bool convUtf8toSJis(BYTE* pSource, BYTE* pDist, int* pSize){
		*pSize = 0;

		//UTF-8����UTF-16�֕ϊ�
		const int nSize = ::MultiByteToWideChar(CP_UTF8, 0, (LPCSTR)pSource, -1, NULL, 0);

		BYTE* buffUtf16 = new BYTE[nSize * 2 + 2];
		::MultiByteToWideChar(CP_UTF8, 0, (LPCSTR)pSource, -1, (LPWSTR)buffUtf16, nSize);

		//UTF-16����Shift-JIS�֕ϊ�
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
	* ���� std::string �߂�l std::string �ɕύX��������
	* ���ʂȂ��Ƃ��������肱�̊֐���p����
	* UTF8�̕������Shift-JIS�ɕϊ�����
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
};

VoiceRecognition::~VoiceRecognition()
{

	this->disconnect();
}

/*japaneseMessage2englishMessage
*����std::string  �߂�lstd::string
*SIGVerse���őS�p���g���Ȃ��̂ő��M���b�Z�[�W����{�ꂩ��p��ɒu��������
*/
std::string VoiceRecognition::japaneseMessage2englishMessage(std::string japaneseMessage){
	std::string ja_str = japaneseMessage;
	std::string en_str;

	//�N���[���i�b�v�J�n
	if (ja_str == "�ЂÂ��܂�") en_str = "go";

	//�c���Ώۂ̎w��
	else if (ja_str == "�ΐF�̃S�~��") en_str = "trashbox_0";
	//else if (ja_str == "������Ƃ���") en_str = "take";

	//�̂Ă�or�u���Ώۂ̎w��
	else if (ja_str == "�ԐF�̃S�~��") en_str = "trashbox_1";
	//else if (ja_str == "����Ɏ̂Ă�") en_str = "put";
	//else if (ja_str == "�����Ɏ̂Ă�") en_str = "put";
	
	else if (ja_str == "�F�̃S�~��") en_str = "trashbox_2";
	//else if (ja_str == "�����ɂ�����") en_str = "put";

	else if (ja_str == "���F�̃��S��") en_str = "wagon_0";

	//�̂Ă�or�u���Ώۂ��Ԉ�����ꍇ��蒼��
	else if (ja_str == "�I���܂�") en_str = "finish";	

	//����̖��߈ȊO�̏ꍇ
	else en_str = "error";
	return en_str;
}

/*englishMessage2japaneseMessage
*����std::string  �߂�lstd::string
*OpenJtalk�ɓ��{��Ŕ��b���������̂Ŏ�M���b�Z�[�W���p�ꂩ����{��ɒu��������
*/
std::string VoiceRecognition::englishMessage2japaneseMessage(std::string englishMessage){
	std::string en_str = englishMessage;
	std::string ja_str;

	if (en_str == "start")
	{
		ja_str = "����ł͕ЂÂ����n�߂܂�";
	}
	else if (en_str == "please pass my hand")
	{
		ja_str = "�Еt���������̂��E��ɓn���Ă�������";
	}
	else if (en_str == "Message is not accepted")
	{
		ja_str = "�������܂���B";
	}
	else if (en_str == "please choose the trashbox")
	{
		ja_str = "�����Ă�����������Ă�������";
	}
	else if (en_str == "petbottle_1") 
	{
		m_pointedObject = "�y�b�g�{�g��";
		ja_str = m_pointedObject + "���̂Ă܂�";
	}
	else if (en_str == "kettle") 
	{
		m_pointedObject = "�P�g��";
		ja_str = m_pointedObject + "�����܂�";
	}
	else if (en_str == "can_0")
	{
		m_pointedObject = "��";
		ja_str = "�A" +m_pointedObject + "���̂Ă܂�";
	}
	else if (en_str == "finish")
	{
		ja_str = "�Y��ɕЕt���܂���";
	}
	else if (en_str == "Please show me which trash box to use")
	{
		ja_str = m_pointedObject + "���^�Ԑ�������ĉ�����";
	}
	else if (en_str == "trashbox_0")
	{
		m_pointedtrash = "���T�C�N��";
		ja_str = "������܂����A" + m_pointedObject + "���A" + m_pointedtrash + "�A�Ɏ̂Ăɍs���܂�";
	}
	else if (en_str == "trashbox_1")
	{
		m_pointedtrash = "�R����S�~";
		ja_str = "������܂����A" + m_pointedObject + "���A" + m_pointedtrash + "�A�Ɏ̂Ăɍs���܂�";
	}
	else if (en_str == "trashbox_2")
	{
		m_pointedtrash = "�r�E��";
		ja_str = "������܂����A" + m_pointedObject + "���A" + m_pointedtrash + "�A�Ɏ̂Ăɍs���܂�";
	}
	else if (en_str == "wagon_0")
	{
		m_pointedtrash = "���S��";
		ja_str = "������܂����A" + m_pointedObject + "���A" + m_pointedtrash + "�A�ɒu���ɍs���܂�";
	}
	return ja_str;
}


void VoiceRecognition::onInit(){
	robotName = "robot_000";

	ShellExecute(NULL, "open", "juliusbat\\julius-run.bat", NULL, NULL, SW_SHOW);

	std::ofstream ofs("juliusbat\\sentence.txt");
	ofs.clear(); //�O�񕶎���폜
	ofs.close();

	Enable = true;
	preview_string = "";
	
	ms.initialize();
}



double VoiceRecognition::onAction()
{
	if (Enable)
	{
		std::ifstream readfile("juliusbat\\sentence.txt"); //julius�F����������擾
		std::string str;
		std::getline(readfile, str); //1�s�ǂݍ���
		readfile.close();

		str = WordCodeConvert::utf8toSjis(str); //�����R�[�h�̕ϊ�

		size_t pos;
		while ((pos = str.find_first_of(" �@�A�B")) != std::string::npos){ //��������̔��p�E�S�p�X�y�[�X�C��Ǔ_���폜
			str.erase(pos, 1);
		}
		if (str != preview_string){
			std::string send_msg = japaneseMessage2englishMessage(str);//���b�Z�[�W�̒u������

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
	ms.noWaitSpeak(englishMessage2japaneseMessage(s));//���b
}

int main(int argc, char** argv)
{
	VoiceRecognition srv("VoiceReco_Service");
	//srv.onInit();
	unsigned short port = (unsigned short)(atoi(argv[2]));
	srv.connect(argv[1], port);
	//srv.connect("136.187.35.129", 9005);
	srv.startLoop();
	return 0;
}