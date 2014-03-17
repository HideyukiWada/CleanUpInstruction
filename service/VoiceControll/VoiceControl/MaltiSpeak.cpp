#include "MaltiSpeak.h"

void MaltiSpeak::initialize(){
	this->file_name = "openjtalk\\text.txt";
	this->talkFinishFlag = true;
}

void MaltiSpeak::Speak(std::string msg, int time) {
	if (1){
		//std::lock_guard<std::mutex> lk(m);
		if ((fp = fopen(file_name.c_str(), "w")) == NULL){
			printf("FILE OPEN ERROR!\n");
			exit(1);
		}

		fprintf(fp, msg.c_str());
		fclose(fp);

		ShellExecute(NULL, "open", "openjtalk\\Sobit-text.bat", NULL, NULL, SW_HIDE);
	}
}

void MaltiSpeak::waitFinish(int waitTime){
	Sleep(waitTime);
	this->talkFinishFlag = true;
}

void MaltiSpeak::noWaitSpeak(std::string msg){
	int time = 0;
	this->Speak(msg, time);
}

void MaltiSpeak::waitSpeak(std::string msg, int time){
	this->talkFinishFlag = false;
	this->Speak(msg, time);
	Sleep(time);
	this->talkFinishFlag = true;
}

void MaltiSpeak::joinSpeak(std::string msg, int talkTime){
	//this->talkFinishFlag = false;
	//if (this->timeThread.joinable())
	//	this->timeThread.join();
	//int time = 0;
	//this->Speak(msg, time);
	//this->timeThread = std::thread(std::bind(&MaltiSpeak::waitFinish, this, talkTime));
}

void MaltiSpeak::waitJoin(){
	//if(this->timeThread.joinable())
	//	this->timeThread.join();
}

bool MaltiSpeak::speakState(){
	return this->talkFinishFlag;
}

MaltiSpeak::~MaltiSpeak(){
	//if (this->timeThread.joinable())
	//	this->timeThread.join();
}