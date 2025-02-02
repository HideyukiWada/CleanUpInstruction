#include <Controller.h>
#include <ControllerEvent.h>
#include <Logger.h>
#include <ViewImage.h>
#include <math.h>
#include <iostream>
#include <stdio.h>
#include <stdlib.h>

#define PI 3.141592
#define DEG2RAD(DEG) ( (PI) * (DEG) / 180.0 )

class UserController : public Controller
{
public:

	void moveBodyByKINECT(char* msg);
	//void moveHeadByHMD(const std::string ss);
	void onRecvMsg(RecvMsgEvent &evt);
	void onInit(InitEvent &evt);
	double onAction(ActionEvent &evt);
	void onCollision(CollisionEvent &evt);
	void throwTrash(void);

private:
	RobotObj *m_robotObject;
	std::string m_graspObjectName;
	std::string storageSpaceName;

	std::vector<std::string> objectList;

	//移動速度
	double vel;
	ViewService *m_view;
	BaseService *m_kinect;
	BaseService *m_ors;
	BaseService *m_hmd;

	bool chk_srv;

	//初期位置
	double m_posx, m_posy, m_posz;
	double m_yrot;
	double m_range;

	//データ数（関節数）最大値
	int m_maxsize;

	// 体全体の角度
	double m_qw, m_qy, m_qx, m_qz;

	// 前回送信したyaw, pitch roll
	double pyaw, ppitch, proll;

	// ロボットの名前
	std::string robotName;

	// 物体把持のフラグ
	bool m_grasp;

	//物体開放のフラグ
	bool m_release;
};

void UserController::onInit(InitEvent &evt)
{
	m_robotObject = getRobotObj(myname());
	robotName = "sobit";

	objectList.push_back("Clock");
	objectList.push_back("Bear");
	objectList.push_back("Penguin");
	objectList.push_back("Cup");



	m_kinect = NULL;
	m_hmd = NULL;
	m_ors = NULL;

	vel = 10.0;
	srand(time(NULL));

	//初期位置の設定
	SimObj *my = this->getObj(this->myname());
	m_posx = my->x();
	m_posy = my->y();
	m_posz = my->z();
	m_range = 0.1;
	m_maxsize = 15;
	double qw = my->qw();
	double qy = my->qy();
	m_yrot = acos(fabs(qw))*2;
	if(qw*qy > 0)
	m_yrot = -1*m_yrot;

	// 体全体の向き
	m_qw = 1.0;
	m_qx = 0.0;
	m_qy = 0.0;
	m_qz = 0.0;

	pyaw = ppitch = proll = 0.0;

	m_grasp = false;
	m_release = false;
}

//定期的に呼び出される関数
double UserController::onAction(ActionEvent &evt)
{

	// サービスが使用可能か定期的にチェックする
	bool av_kinect = checkService("SIGKINECT");
	bool ch_ors = checkService("SIGORS");
	

	if (ch_ors && m_ors == NULL) {
		m_ors = connectToService("SIGORS");
	}
	else if (!ch_ors && m_ors != NULL){
		m_ors = NULL;
	}

	// 使用可能
	if(av_kinect && m_kinect == NULL){
	// サービスに接続
	m_kinect = connectToService("SIGKINECT");

	}
	else if (!av_kinect && m_kinect != NULL){
	m_kinect = NULL;
	}

	return 1.5;
}

void UserController::onRecvMsg(RecvMsgEvent &evt)
{

	std::string sender = evt.getSender();

	//自分自身の取得
	SimObj *my = getObj(myname());

	//メッセージ取得
	char *all_msg = (char*)evt.getMsg();

	std::string ss = all_msg;
	//ヘッダーの取り出し
	int strPos1 = 0;
	int strPos2;
	std::string headss;
	std::string tmpss;
	strPos2 = ss.find(" ", strPos1);
	headss.assign(ss, strPos1, strPos2-strPos1);

	

	if (headss == "ORS_DATA") {
		//LOG_MSG(("%s",all_msg));
		//  }
		//  if(headss == "HMD_DATA"){

		double yaw, pitch, roll;
		strPos1 = strPos2 + 1;
		tmpss = "";

		strPos2 = ss.find(",", strPos1);
		tmpss.assign(ss, strPos1, strPos2 - strPos1);
		yaw = atof(tmpss.c_str());

		strPos1 = strPos2 + 1;
		strPos2 = ss.find(",", strPos1);
		tmpss.assign(ss, strPos1, strPos2 - strPos1);
		pitch = atof(tmpss.c_str());

		strPos1 = strPos2 + 1;
		strPos2 = ss.find(",", strPos1);
		tmpss.assign(ss, strPos1, strPos2 - strPos1);
		roll = atof(tmpss.c_str());

		if (yaw == pyaw && pitch == ppitch && roll == proll)  return;
		else {
			pyaw = yaw;
			ppitch = pitch;
			proll = roll;
		}

		dQuaternion qyaw;
		dQuaternion qpitch;
		dQuaternion qroll;

		qyaw[0] = cos(-yaw / 2.0);
		qyaw[1] = 0.0;
		qyaw[2] = sin(-yaw / 2.0);
		qyaw[3] = 0.0;

		qpitch[0] = cos(-pitch / 2.0);
		qpitch[1] = sin(-pitch / 2.0);
		qpitch[2] = 0.0;
		qpitch[3] = 0.0;

		qroll[0] = cos(-roll / 2.0);
		qroll[1] = 0.0;
		qroll[2] = 0.0;
		qroll[3] = sin(-roll / 2.0);
		dQuaternion tmpQ1;
		dQuaternion tmpQ2;

		dQMultiply0(tmpQ1, qyaw, qpitch);
		dQMultiply0(tmpQ2, tmpQ1, qroll);

		dQuaternion bodyQ;
		bodyQ[0] = m_qw;
		bodyQ[1] = m_qx;
		bodyQ[2] = m_qy;
		bodyQ[3] = m_qz;

		dQuaternion tmpQ3;
		dQMultiply1(tmpQ3, bodyQ, tmpQ2);

		my->setJointQuaternion("HEAD_JOINT0", tmpQ3[0], tmpQ3[1], -tmpQ3[2], tmpQ3[3]);
	}
	else if(headss == "KINECT_DATA"){
	//KINECTデータによる頭部以外の体の動き反映
	moveBodyByKINECT(all_msg);
	}
	else if(ss == "go") {
	sendMsg(robotName,"go");
		LOG_MSG(("Starting the clean up task"));
	}

	else if(ss == "take" ) {
	sendMsg(robotName,"take");
		LOG_MSG(("Taking the trash"));
	}

	else if(ss == "put" ) {
	sendMsg(robotName,"put");
		LOG_MSG(("Putting the trash in the trash box"));
	}
	else if (ss == "release") {
		this->throwTrash();
		std::string msg = "release";
		msg += " " + m_graspObjectName;
		sendMsg(robotName, msg);
	}
	




	else if(ss == "init") {
	sendMsg(robotName,"init");
	}

}

//void UserController::moveHeadByHMD(const std::string ss){
//
//	//自分自身の取得
//	SimObj *my = this->getObj(this->myname());
//
//	//ヘッダーの取り出し
//	int strPos1 = 0;
//	int strPos2;
//	std::string headss;
//	std::string tmpss;
//	strPos2 = ss.find(" ", strPos1);
//	headss.assign(ss, strPos1, strPos2-strPos1);
//
//	if(headss == "HMD_DATA"){
//
//	double yaw, pitch, roll;
//	strPos1 = strPos2+1;
//	tmpss = "";
//
//	strPos2 = ss.find(",", strPos1);
//	tmpss.assign(ss, strPos1, strPos2-strPos1);
//	yaw = atof(tmpss.c_str());
//
//	strPos1 = strPos2+1;
//	strPos2 = ss.find(",", strPos1);
//	tmpss.assign(ss, strPos1, strPos2-strPos1);
//	pitch = atof(tmpss.c_str());
//
//	strPos1 = strPos2+1;
//	strPos2 = ss.find(",", strPos1);
//	tmpss.assign(ss, strPos1, strPos2-strPos1);
//	roll = atof(tmpss.c_str());
//
//
//	// 前回送信したyaw pitch roll と同じ場合は送信しない
//	if(yaw == pyaw && pitch == ppitch && roll == proll)  return;
//	else {
//		pyaw = yaw;
//		ppitch = pitch;
//		proll = roll;
//	}
//
//	// yaw pitch roll をクオータニオンに変換
//	dQuaternion qyaw;
//	dQuaternion qpitch;
//	dQuaternion qroll;
//
//	qyaw[0] = cos(-yaw/2.0);
//	qyaw[1] = 0.0;
//	qyaw[2] = sin(-yaw/2.0);
//	qyaw[3] = 0.0;
//
//	qpitch[0] = cos(-pitch/2.0);
//	qpitch[1] = sin(-pitch/2.0);
//	qpitch[2] = 0.0;
//	qpitch[3] = 0.0;
//
//	qroll[0] = cos(-roll/2.0);
//	qroll[1] = 0.0;
//	qroll[2] = 0.0;
//	qroll[3] = sin(-roll/2.0);
//	dQuaternion tmpQ1;
//	dQuaternion tmpQ2;
//
//	// yaw pitch roll の順に回転
//	dQMultiply0(tmpQ1, qyaw, qpitch);
//	dQMultiply0(tmpQ2, tmpQ1, qroll);
//
//	// 体全体の回転
//	dQuaternion bodyQ;
//	bodyQ[0] = m_qw;
//	bodyQ[1] = m_qx;
//	bodyQ[2] = m_qy;
//	bodyQ[3] = m_qz;
//
//	// 体全体の回転との差分をとる
//	dQuaternion tmpQ3;
//	dQMultiply1(tmpQ3, bodyQ, tmpQ2);
//
//	// 回転角度が小さい場合は回転しない
//	double theta = 2*acos(tmpQ3[0]);
//	if(theta <= 0.001) return;
//
//	// 首の関節を回転する
//	my->setJointQuaternion("HEAD_JOINT0", tmpQ3[0], tmpQ3[1], tmpQ3[2], tmpQ3[3]);
//	}
//}


void UserController::moveBodyByKINECT(char* all_msg){

	//自分自身の取得
	SimObj *my = this->getObj(this->myname());
	char* msg = strtok(all_msg," ");
	if (strcmp(msg, "KINECT_DATA") == 0){
		int i = 0;
		while (true){
			i++;
			if (i == m_maxsize + 1) break;
			char *type = strtok(NULL, ":");
			if (strcmp(type, "POSITION") == 0){
				//体の位置
				double x = atof(strtok(NULL, ","));
				double y = atof(strtok(NULL, ","));
				double z = atof(strtok(NULL, " "));
				//エージェント座標からグローバル座標への変換
				double gx = cos(m_yrot)*x - sin(m_yrot)*z;
				double gz = sin(m_yrot)*x + cos(m_yrot)*z;
				my->setPosition(m_posx + gx, m_posy + y, m_posz + gz);
				//limitedSetPosition(my, m_posx+gx,m_posy+y,m_posz+gz);
				continue;
			}
			else if (strcmp(type, "WAIST") == 0){
				//体全体の回転

				double w = atof(strtok(NULL, ","));
				double x = atof(strtok(NULL, ","));
				double y = atof(strtok(NULL, ","));
				double z = atof(strtok(NULL, " "));
				my->setJointQuaternion("ROOT_JOINT0", w, x, y, z);
				m_qw = w;
				m_qx = x;
				m_qy = y;
				m_qz = z;
				continue;
			}
			else if (strcmp(type, "END") == 0){
				break;
			}
			else{
				//関節の回転
				double w = atof(strtok(NULL, ","));
				double x = atof(strtok(NULL, ","));
				double y = atof(strtok(NULL, ","));
				double z = atof(strtok(NULL, " "));
				double angle = acos(w) * 2;
				double tmp = sin(angle / 2);
				double vx = x / tmp;
				double vy = y / tmp;
				double vz = z / tmp;
				double len = sqrt(vx*vx + vy*vy + vz*vz);
				if (len < (1 - m_range) || (1 + m_range) < len) continue;
				// HEAD_JOINT1はHMDにより回転
				if (strcmp(type, "HEAD_JOINT1") != 0){
					my->setJointQuaternion(type, w, x, y, z);
				}
				continue;
			}
		}
	}

}

void UserController::onCollision(CollisionEvent &evt) {
	typedef CollisionEvent::WithC C;
	////触れたエンティティの名前を得ます
	//const std::vector<std::string> & with = evt.getWith();
	//// 衝突した自分のパーツを得ます  
	//const std::vector<std::string> & mparts = evt.getMyParts();
	//// 衝突したエンティティのパーツを得ます
	//const std::vector<std::string> & wparts = evt.getWithParts();
	//std::cout << "withSize:" << with.size() << "  mpartsSIze:" << mparts.size() << "  wpartsSIze:" << wparts.size() << "==============="<< std::endl;
	//for (int i = 0; i < with.size(); i++){
	//	std::cout << "  with[" << i << "]:" << with[i] << std::endl;
	//	std::cout << "mparts[" << i << "]:" << mparts[i] << std::endl;
	//	std::cout << "wparts[" << i << "]:" << wparts[i] << std::endl << std::endl;
	//}
	//std::cout << "==========================================================================="<< std::endl;


	if (m_grasp == false){
		//触れたエンティティの名前を得ます
		const std::vector<std::string> & with = evt.getWith();
		// 衝突した自分のパーツを得ます  
		const std::vector<std::string> & mparts = evt.getMyParts();

		//　衝突したエンティティでループします
		for (int i = 0; i < with.size(); i++){
			for (int j = 0; j < objectList.size(); j++){
				if (objectList[j] == with[i]){
					//右手に衝突した場合
					if (mparts[i] == "RARM_LINK7"){
						//自分を取得
						SimObj *my = getObj(myname());
						//自分の手のパーツを得ます
						CParts * parts = my->getParts("RARM_LINK7");
						if (parts->graspObj(with[i])){
							m_grasp = true;
							m_graspObjectName = objectList[j];
						}
					}
				}
			}
		}
	}
	else {
		//触れたエンティティの名前を得ます
		const std::vector<std::string> & with = evt.getWith();
		// 衝突した自分のパーツを得ます  
		const std::vector<std::string> & mparts = evt.getMyParts();
		// 衝突したエンティティのパーツを得ます
		const std::vector<std::string> & wparts = evt.getWithParts();

		//衝突したエンティティでループします
		for (int i = 0; i < with.size(); i++){
			if (with[i] == robotName){
				//ロボットの右手に衝突した場合
				if (wparts[i] == "RARM_LINK7"){
					if (mparts[i] == "RARM_LINK7"){
						this->throwTrash();
						std::string msg = "release";
						msg += " " + m_graspObjectName;
						sendMsg(robotName, msg);
					}
				}
			}
		}
	}
}

void UserController::throwTrash(void){
	// get the part info. 
	CParts *parts = m_robotObject->getParts("RARM_LINK7");

	// release grasping
	parts->releaseObj();

	// wait a bit
	//sleep(1);

	// set the grasping flag to neutral
	m_grasp = false;
}

extern "C" Controller * createController ()
{
	return new UserController;
}
