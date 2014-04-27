#include "ControllerEvent.h"
#include "Controller.h"
#include "Logger.h"
#include <unistd.h>
#include <algorithm>
#include <stdlib.h>
#include <string>
#include <sstream>

//convert angle unit from degree to radian
#define DEG2RAD(DEG) ( (M_PI) * (DEG) / 180.0 )

class DemoRobotController : public Controller {
public:  
	void onInit(InitEvent &evt);  
	double onAction(ActionEvent&);  
	void onRecvMsg(RecvMsgEvent &evt); 
	void onCollision(CollisionEvent &evt); 

	void stopRobotMove(void);
	double goToObj(Vector3d pos, double range);
	double rotateTowardObj(Vector3d pos);
	void recognizeObjectPosition(Vector3d &pos, std::string &name);
	void throwTrash(void);
	double goGraspingObject(Vector3d &pos);
	void neutralizeArms(double evt_time);
	void prepareThrowing(double evt_time);

private:
	RobotObj *m_robotObject;
	ViewService *m_view;

	int m_state; 
	double refreshRateOnAction;

	std::vector<std::string> objectList;

	std::string m_graspObjectName;
	std::string userName;

	double m_angularVelocity;  // rotation speed of the wheel
	double m_jointVelocity;    // rotation speed around the joint
	double m_radius;           // radius of the wheel
	double m_distance;         // length of wheel-track
	double m_movingSpeed;      // actual velocity of the moving robot

	// times wasted for moving, adjusting or driving
	double m_time;
	double m_time1;
	double m_time4;

	std::string storageSpaceName0;
	std::string storageSpaceName1;
	std::string storageSpaceName2;
	std::string storageSpaceName3;
	std::string storageSpaceName4;
	std::string storageSpaceName5;
	std::string storageSpaceName;

	//収納場所からこの距離だけ離れて止まる
	double stopMargin;

	//positions
	Vector3d m_frontStorageSpace0;	//テーブル	margin=70
	Vector3d m_frontStorageSpace1;	//キッチン
	Vector3d m_frontStorageSpace2;	//ワゴン
	Vector3d m_frontStorageSpace3;	//本棚
	Vector3d m_frontStorageSpace4;	//食器棚
	Vector3d m_frontStorageSpace5;	//棚

	Vector3d frontStorageSpace;

	Vector3d m_throwPosition0;
	Vector3d m_throwPosition1;
	Vector3d m_throwPosition2;
	Vector3d m_throwPosition3;
	Vector3d m_throwPosition4;
	Vector3d m_throwPosition5;

	Vector3d throwPosition;

	Vector3d m_relayPoint1;
	Vector3d m_frontTrash1;
	Vector3d m_frontTrash2;
	Vector3d m_waitPosition;
	Vector3d m_userPosition;

	// condition flag for grasping trash
	bool m_grasp;

	bool m_release;

  // angular parameter used to put robot's hands down
  double thetaA;

  std::stringstream ss;
  int error_count;
};


void DemoRobotController::onInit(InitEvent &evt) {
	// get robot's name
	m_robotObject = getRobotObj(myname());

	// set wheel configuration
	m_radius = 10.0;
	m_distance = 10.0;
	/*RobotObj *m_robotObject = getRobotObj(myname());*/
	m_robotObject->setWheel(m_radius, m_distance);

	m_time = 0.0;
	m_time1 = 0.0;
	m_time4 = 0.0;

	m_state = 100;  // switch of initial behavior
	//m_state = 9140;


	refreshRateOnAction = 0.1;     // refresh-rate for onAction proc.

	// angular velocity of wheel and moving speed of robot
	m_angularVelocity = 1.5;
	m_movingSpeed = m_angularVelocity*m_radius;  // conversion: rad/ms -> m/ms)

	// rotation speed of joint
	m_jointVelocity = 0.5;

	storageSpaceName0 = "Table";
	storageSpaceName1 = "Kitchen";
	storageSpaceName2 = "Wagon";
	storageSpaceName3 = "BookShelf";
	storageSpaceName4 = "CupBoard";
	storageSpaceName5 = "Shelf";

	objectList.push_back("Clock");
	objectList.push_back("Bear");
	objectList.push_back("Penguin");
	objectList.push_back("Cup");


	//m_graspObjectName = m_trashName2;

	userName = "man_000";

	// set positions;
	m_frontStorageSpace0 = Vector3d(-300.0, 0.0, -40.0);	//Table
	m_frontStorageSpace1 = Vector3d(-295.0, 0.0, -185.5);	//Kitchen 
	m_frontStorageSpace2 = Vector3d(-190.0, 0.0, -195.0);	//Wagon
	m_frontStorageSpace3 = Vector3d(-70.0, 0.0, -185.0);	//BookShelf
	m_frontStorageSpace4 = Vector3d(100.0, 0.0, -185.0);	//CupBoard
	m_frontStorageSpace5 = Vector3d(120.0, 0.0, -90.0);	//Sheif
	
	m_throwPosition0 = Vector3d(-300.0, 0.0, 60.0);	//Table
	m_throwPosition1 = Vector3d(-295.0, 0.0, -240.5);	//Kitchen 
	m_throwPosition2 = Vector3d(-200.0, 0.0, -225.0);	//Wagon
	m_throwPosition3 = Vector3d(-75.0, 0.0, -243.0);	//BookShelf
	m_throwPosition4 = Vector3d(100.0, 0.0, -238.0);	//CupBoard
	m_throwPosition5 = Vector3d(400.0, 0.0, -80.0);	//Sheif
	
	m_relayPoint1    = Vector3d(90.0, 0.0, -100.0);
	m_frontTrash1    = Vector3d(273.0, 0.0, -65.0);
	m_frontTrash2    = Vector3d(305.0, 0.0, -80.0);
	m_waitPosition   = Vector3d(70.0, 0.0, 50.0);
	m_userPosition   = Vector3d(180.0, 0.0, 50.0);

	m_grasp = false;
	m_release = true;

	error_count = 0;

	//デバッグ用
	frontStorageSpace = m_frontStorageSpace3;
	storageSpaceName = storageSpaceName3;
	throwPosition = m_throwPosition3;
	stopMargin = 0.0;
}


double DemoRobotController::onAction(ActionEvent &evt) {
	switch (m_state){
	case 0: {
				break;
	}
	case 100: {
				this->neutralizeArms(evt.time());
				m_state = 110;
				LOG_MSG(("m_state:%d\n", m_state));
				sendMsg("VoiceReco_Service", "start");
				break;
	}
	case 110: {

				//RobotObj *m_robotObject = getRobotObj(myname());
				double angleJoint1 = m_robotObject->getJointAngle("RARM_JOINT0")*180.0 / (M_PI);
				double angleJoint4 = m_robotObject->getJointAngle("RARM_JOINT3")*180.0 / (M_PI);

				//LOG_MSG(("\nm_time1:%4f JOINT1 angle:%4f\nm_time4:%4f JOINT4 angle:%4f\n", m_time1, angleJoint1, m_time4, angleJoint4));
				if (evt.time() >= m_time4) m_robotObject->setJointVelocity("RARM_JOINT3", 0.0, 0.0);
				if (evt.time() >= m_time1) m_robotObject->setJointVelocity("RARM_JOINT0", 0.0, 0.0);
				if (evt.time() >= m_time1 && evt.time() >= m_time4){
					this->stopRobotMove();
					sendMsg("VoiceReco_Service", "please pass my hand");
					m_state = 200;
					//m_state = 300;	//デバッグ用 手渡しをスキップ
					//m_time = 6.0;

					LOG_MSG(("m_state:%d\n", m_state));
				}
				break;
	}
		//手渡し対機
	case 200: {
				if (m_grasp == false && m_release == false){
					//自分を取得
					RobotObj *m_robotObject = getRobotObj(myname());
					//自分の手のパーツを得ます
					CParts * parts = m_robotObject->getParts("RARM_LINK7");
					sendMsg("SIGViewer", m_graspObjectName);
					if (parts->graspObj(m_graspObjectName)){
						m_grasp = true;
						sendMsg("VoiceReco_Service", m_graspObjectName);
						m_state = 300;
						LOG_MSG(("m_state:%d\n", m_state));

						//運ぶ物体を対応表に記録
						std::string msg = "object";
						msg += " " + m_graspObjectName;
						sendMsg("Memorize_Service", msg);
					}
				}
				break;
	}
	case 300: {
				double l_moveTime = rotateTowardObj(Vector3d(10.0, 0.0, -90));

				m_time = l_moveTime + evt.time();
				m_state = 310;
				LOG_MSG(("m_state:%d\n", m_state));
				break;
	}
	case 310: {
				if (evt.time() >= m_time){
					this->stopRobotMove();
					sendMsg("VoiceReco_Service", "please choose the trashbox");
					
					//m_state = 320;
					m_state = 400;	//デバッグ用 発話対機をスキップ

					LOG_MSG(("m_state:%d\n", m_state));
				}
				break;
	}
		//発話対機状態
	case 320: {
			  break;
	}
	case 400: {
				double l_moveTime = rotateTowardObj(frontStorageSpace);

				m_time = l_moveTime + evt.time();
				m_state = 410;
				LOG_MSG(("m_state:%d\n", m_state));
				break;
	}
	case 410: {
				if (evt.time() >= m_time){
					this->stopRobotMove();
					double l_moveTime = goToObj(frontStorageSpace, 0.0);
					m_time = l_moveTime + evt.time();
					m_state = 420;
					LOG_MSG(("m_state:%d\n", m_state));
				}
				break;
	}
	case 420: {
				if (evt.time() >= m_time){
					this->stopRobotMove();
					this->prepareThrowing(evt.time());

					m_state = 430;
					LOG_MSG(("m_state:%d\n", m_state));
				}
				break;
	}
	case 430: {
				//RobotObj *m_robotObject = getRobotObj(myname());
				double angleJoint1 = m_robotObject->getJointAngle("RARM_JOINT0")*180.0 / (M_PI);
				double angleJoint4 = m_robotObject->getJointAngle("RARM_JOINT3")*180.0 / (M_PI);

				//LOG_MSG(("\nm_time1:%4f JOINT1 angle:%4f\nm_time4:%4f JOINT4 angle:%4f\n", m_time1, angleJoint1, m_time4, angleJoint4));
				if (evt.time() >= m_time4) m_robotObject->setJointVelocity("RARM_JOINT3", 0.0, 0.0);
				if (evt.time() >= m_time1) m_robotObject->setJointVelocity("RARM_JOINT0", 0.0, 0.0);
				if (evt.time() >= m_time1 && evt.time() >= m_time4){
					this->stopRobotMove();
					//Vector3d l_tpos;
					//this->recognizeObjectPosition(l_tpos, storageSpaceName);
					//double l_moveTime = rotateTowardObj(l_tpos);
					double l_moveTime = rotateTowardObj(throwPosition);
					m_time = l_moveTime + evt.time();

					m_state = 440;
					LOG_MSG(("m_state:%d\n", m_state));
				}
				break;
	}
	case 440: {
				if (evt.time() >= m_time){

					this->stopRobotMove();
					Vector3d l_tpos;
					//this->recognizeObjectPosition(l_tpos, storageSpaceName);
					//double l_moveTime = goToObj(l_tpos, stopMargin);

					double l_moveTime = goToObj(throwPosition, stopMargin);
					m_time = l_moveTime + evt.time();

					m_state = 450;
					LOG_MSG(("m_state:%d\n", m_state));
				}
				break;
	}
	case 450: {  // throw trash and get back a bit
				if (evt.time() >= m_time){
					this->stopRobotMove();
					this->throwTrash();

					//運び先を対応表に記録
					std::string msg = "storageSpace";
					msg += " " + storageSpaceName;
					sendMsg("Memorize_Service", msg);

					sleep(1);

					//RobotObj *m_robotObject = getRobotObj(myname());
					m_robotObject->setWheelVelocity(-m_angularVelocity, -m_angularVelocity);
					m_time = 30.0 / m_movingSpeed + evt.time();

					m_state = 460;
					LOG_MSG(("m_state:%d\n", m_state));
				}
				break;
	}
	case 460: {
				  if (evt.time() >= m_time){
					  this->stopRobotMove();
					  double l_moveTime = rotateTowardObj(frontStorageSpace);
					  m_time = l_moveTime + evt.time();

					  m_state = 470;
					  LOG_MSG(("m_state:%d\n", m_state));
				  }
				  break;
	}
	case 470: {
				  if (evt.time() >= m_time){

					  this->stopRobotMove();
					  //Vector3d l_tpos;
					  //this->recognizeObjectPosition(l_tpos, storageSpaceName);
					  //double l_moveTime = goToObj(l_tpos, stopMargin);

					  double l_moveTime = goToObj(frontStorageSpace, stopMargin);
					  m_time = l_moveTime + evt.time();

					  m_state = 480;
					  LOG_MSG(("m_state:%d\n", m_state));
				  }
				  break;
	}
	case 480: {
				  if (evt.time() >= m_time){
					  this->stopRobotMove();
					  double l_moveTime = rotateTowardObj(m_waitPosition);
					  m_time = l_moveTime + evt.time();

					  m_state = 490;
					  LOG_MSG(("m_state:%d\n", m_state));
				  }
				  break;
	}
	case 490: {
				  if (evt.time() >= m_time){

					  this->stopRobotMove();
					  //Vector3d l_tpos;
					  //this->recognizeObjectPosition(l_tpos, storageSpaceName);
					  //double l_moveTime = goToObj(l_tpos, stopMargin);

					  double l_moveTime = goToObj(m_waitPosition, stopMargin);
					  m_time = l_moveTime + evt.time();

					  m_state = 500;
					  LOG_MSG(("m_state:%d\n", m_state));
				  }
				  break;
	}
	case 500: {
				  if (evt.time() >= m_time){

					  this->stopRobotMove();
					  double l_moveTime = rotateTowardObj(m_userPosition);
					  m_time = l_moveTime + evt.time();

					  m_state = 510;
					  LOG_MSG(("m_state:%d\n", m_state));
				  }
				  break;
	}
	case 510: {
				  if (evt.time() >= m_time){

					  this->stopRobotMove();

					  frontStorageSpace = m_frontStorageSpace2;
					  storageSpaceName = storageSpaceName2;
					  throwPosition = m_throwPosition2;

					  m_state = 100;
					  LOG_MSG(("m_state:%d\n", m_state));
				  }
				  break;
	}
	//case 230: {
	//			  //RobotObj *m_robotObject = getRobotObj(myname());
	//			  double angleJoint1 = m_robotObject->getJointAngle("RARM_JOINT0")*180.0 / (M_PI);
	//			  double angleJoint4 = m_robotObject->getJointAngle("RARM_JOINT3")*180.0 / (M_PI);

	//			  LOG_MSG(("\nm_time1:%4f JOINT1 angle:%4f\nm_time4:%4f JOINT4 angle:%4f\n", m_time1, angleJoint1, m_time4, angleJoint4));

	//			if (evt.time() >= m_time4 && m_state == 230) m_robotObject->setJointVelocity("RARM_JOINT3", 0.0, 0.0);
	//			if (evt.time() >= m_time1 && m_state == 230) m_robotObject->setJointVelocity("RARM_JOINT0", 0.0, 0.0);
	//			if (evt.time() >= m_time1 && evt.time() >= m_time4 && m_state == 230){
	//				double l_moveTime = rotateTowardObj(m_waitPosition);

	//				m_time = l_moveTime + evt.time();
	//				m_state = 231;
	//				LOG_MSG(("m_state:%d\n", m_state));
	//			}
	//			break;
	//}
	//case 231: {
	//			if (evt.time() >= m_time && m_state == 231){
	//				this->stopRobotMove();
	//				double l_moveTime = goToObj(m_waitPosition, 0.0);
	//				m_time = l_moveTime + evt.time();
	//				m_state = 232;
	//				LOG_MSG(("m_state:%d\n", m_state));
	//			}
	//			break;
	//}
	//case 232: {
	//			if (evt.time() >= m_time && m_state == 232){
	//				double l_moveTime = rotateTowardObj(m_userPosition);

	//				m_time = l_moveTime + evt.time();
	//				m_state = 233;
	//				LOG_MSG(("m_state:%d\n", m_state));
	//			}
	//			break;
	//}
	//case 233: {
	//			if (evt.time() >= m_time && m_state == 233){
	//				this->stopRobotMove();
	//				this->neutralizeArms(evt.time());

	//				m_state = 141;
	//				LOG_MSG(("m_state:%d\n", m_state));
	//			}
	//			break;
	//}
	//case 234: {
	//			  //RobotObj *m_robotObject = getRobotObj(myname());
	//			  double angleJoint1 = m_robotObject->getJointAngle("RARM_JOINT0")*180.0 / (M_PI);
	//			  double angleJoint4 = m_robotObject->getJointAngle("RARM_JOINT3")*180.0 / (M_PI);

	//			  LOG_MSG(("\nm_time1:%4f JOINT1 angle:%4f\nm_time4:%4f JOINT4 angle:%4f\n", m_time1, angleJoint1, m_time4, angleJoint4));

	//			if (evt.time() >= m_time4 && m_state == 234) m_robotObject->setJointVelocity("RARM_JOINT3", 0.0, 0.0);
	//			if (evt.time() >= m_time1 && m_state == 234) m_robotObject->setJointVelocity("RARM_JOINT0", 0.0, 0.0);
	//			if (evt.time() >= m_time1 && evt.time() >= m_time4 && m_state == 234){
	//				this->stopRobotMove();
	//				m_state = 141;
	//				LOG_MSG(("m_state:%d\n", m_state));
	//			}
	//			break;
	//}

	////デバッグ用
	//case 9140: {
	//			  this->neutralizeArms(evt.time());
	//			  m_state = 9141;
	//			  LOG_MSG(("m_state:%d\n", m_state));
	//			  sendMsg("VoiceReco_Service", "start");
	//			  break;
	//}
	//case 9141: {

	//			  //RobotObj *m_robotObject = getRobotObj(myname());
	//			  double angleJoint1 = m_robotObject->getJointAngle("RARM_JOINT0")*180.0 / (M_PI);
	//			  double angleJoint4 = m_robotObject->getJointAngle("RARM_JOINT3")*180.0 / (M_PI);

	//			  LOG_MSG(("\nm_time1:%4f JOINT1 angle:%4f\nm_time4:%4f JOINT4 angle:%4f\n", m_time1, angleJoint1, m_time4, angleJoint4));
	//			  if (evt.time() >= m_time4 && m_state == 9141) m_robotObject->setJointVelocity("RARM_JOINT3", 0.0, 0.0);
	//			  if (evt.time() >= m_time1 && m_state == 9141) m_robotObject->setJointVelocity("RARM_JOINT0", 0.0, 0.0);
	//			  if (evt.time() >= m_time1 && evt.time() >= m_time4 && m_state == 9141){
	//				  this->stopRobotMove();
	//				  sendMsg("VoiceReco_Service", "please pass my hand");
	//				  //m_state = 9142;
	//				  m_state = 9161;
	//				  m_time = 6.0;

	//				  LOG_MSG(("m_state:%d\n", m_state));
	//			  }
	//			  break;
	//}
	//case 9142: {
	//			  if (m_grasp == false && m_release == false){
	//				  //自分を取得
	//				  //RobotObj *m_robotObject = getRobotObj(myname());
	//				  //自分の手のパーツを得ます
	//				  CParts * parts = m_robotObject->getParts("RARM_LINK7");
	//				  sendMsg("SIGViewer", m_graspObjectName);
	//				  if (parts->graspObj(m_graspObjectName)){
	//					  m_grasp = true;
	//					  sendMsg("VoiceReco_Service", m_graspObjectName);
	//					  m_state = 9161;
	//					  m_time = evt.time() + 3.0;
	//					  LOG_MSG(("m_state:%d\n", m_state));
	//				  }
	//			  }
	//			  break;
	//}
	//case 9161: {
	//			  if (evt.time() >= m_time && m_state == 9161){
	//				  this->stopRobotMove();
	//				  this->prepareThrowing(evt.time());

	//				  m_state = 9165;
	//				  LOG_MSG(("m_state:%d\n", m_state));
	//			  }
	//			  break;
	//}
	//case 9165: {
	//			  //RobotObj *m_robotObject = getRobotObj(myname());
	//			  double angleJoint1 = m_robotObject->getJointAngle("RARM_JOINT0")*180.0 / (M_PI);
	//			  double angleJoint4 = m_robotObject->getJointAngle("RARM_JOINT3")*180.0 / (M_PI);

	//			  LOG_MSG(("\nm_time1:%4f JOINT1 angle:%4f\nm_time4:%4f JOINT4 angle:%4f\n", m_time1, angleJoint1, m_time4, angleJoint4));
	//			  if (evt.time() >= m_time4 && m_state == 9165) m_robotObject->setJointVelocity("RARM_JOINT3", 0.0, 0.0);
	//			  if (evt.time() >= m_time1 && m_state == 9165) m_robotObject->setJointVelocity("RARM_JOINT0", 0.0, 0.0);
	//			  if (evt.time() >= m_time1 && evt.time() >= m_time4 && m_state == 9165){
	//				  this->stopRobotMove();
	//				  Vector3d l_tpos;
	//				  this->recognizeObjectPosition(l_tpos, trashBoxName);
	//				  double l_moveTime = rotateTowardObj(l_tpos);
	//				  m_time = l_moveTime + evt.time();

	//				  m_state = 9140;
	//				  LOG_MSG(("m_state:%d\n", m_state));
	//			  }
	//			  break;
	//}

	}

	


	return refreshRateOnAction;
}


void DemoRobotController::onRecvMsg(RecvMsgEvent &evt) {
	std::string sender = evt.getSender();


	//メッセージ取得
	char *all_msg = (char*)evt.getMsg();

	std::string str = all_msg;
	//ヘッダーの取り出し
	int strPos1 = 0;
	int strPos2;

	std::string headss;
	std::string tmpss;
	strPos2 = str.find(" ", strPos1);
	headss.assign(str, strPos1, strPos2 - strPos1);
	tmpss.assign(str, strPos2 + 1, str.length() - strPos2);

	if (!m_grasp && headss == "release"){
		m_release = false;
		m_graspObjectName = tmpss;
		sendMsg("SIGViewer", m_graspObjectName);
	}
	else if (m_state == 0){
		if (str == "error"){
			error_count++;
			ss << error_count << ":Recognition Failure!!";
			sendMsg("SIGViewer", ss.str());
			ss.str("");
			ss.clear();
		}
		else if (str == "go"){
			sendMsg("SIGViewer", "Recognition Success!!");
			error_count = 0;
			m_state = 140;
			LOG_MSG(("m_state:%d\n", m_state));
		}
		//else sendMsg("VoiceReco_Service", "Message is not accepted");
	}
	else if (m_state == 145){
		if (str == "error"){
			error_count++;
			ss << error_count << ":Recognition Failure!!";
			sendMsg("SIGViewer", ss.str());
			ss.str("");
			ss.clear();
		}
		else if (str == storageSpaceName0){
			sendMsg("VoiceReco_Service", str);
			storageSpaceName = str;
			frontStorageSpace = m_frontStorageSpace0;
			sendMsg("SIGViewer", "Recognition Success!!");
			error_count = 0;
			m_state = 150;
			LOG_MSG(("m_state:%d\n", m_state));
		}
		else if (str == storageSpaceName1){
			sendMsg("VoiceReco_Service", str);
			storageSpaceName = str;
			frontStorageSpace = m_frontStorageSpace1;
			sendMsg("SIGViewer", "Recognition Success!!");
			error_count = 0;
			m_state = 150;
			LOG_MSG(("m_state:%d\n", m_state));
		}
		else if (str == storageSpaceName2){
			sendMsg("VoiceReco_Service", str);
			storageSpaceName = str;
			frontStorageSpace = m_frontStorageSpace2;
			sendMsg("SIGViewer", "Recognition Success!!");
			error_count = 0;
			m_state = 150;
			LOG_MSG(("m_state:%d\n", m_state));
		}
		else if (str == storageSpaceName3){
			sendMsg("VoiceReco_Service", str);
			storageSpaceName = str;
			frontStorageSpace = m_frontStorageSpace3;
			sendMsg("SIGViewer", "Recognition Success!!");
			error_count = 0;
			m_state = 150;
			LOG_MSG(("m_state:%d\n", m_state));
		}
		else if (str == storageSpaceName4){
			sendMsg("VoiceReco_Service", str);
			storageSpaceName = str;
			frontStorageSpace = m_frontStorageSpace4;
			sendMsg("SIGViewer", "Recognition Success!!");
			error_count = 0;
			m_state = 150;
			LOG_MSG(("m_state:%d\n", m_state));
		}
		else if (str == storageSpaceName5){
			sendMsg("VoiceReco_Service", str);
			storageSpaceName = str;
			frontStorageSpace = m_frontStorageSpace5;
			sendMsg("SIGViewer", "Recognition Success!!");
			error_count = 0;
			m_state = 150;
			LOG_MSG(("m_state:%d\n", m_state));
		}
		//else sendMsg("VoiceReco_Service", "Message is not accepted");
	}
	else if (str == "finish"){
		sendMsg("VoiceReco_Service", str);
		disconnectToService("VoiceReco_Service");
		disconnectToService("SIGKINECT");
	}
}


void DemoRobotController::onCollision(CollisionEvent &evt) {
	//if (m_grasp == false && m_release == false){
	//	typedef CollisionEvent::WithC C;
	//	//触れたエンティティの名前を得ます
	//	const std::vector<std::string> & with = evt.getWith();
	//	// 衝突した自分のパーツを得ます  
	//	const std::vector<std::string> & mparts = evt.getMyParts();

	//	//　衝突したエンティティでループします
	//	for(int i = 0; i < with.size(); i++){
	//		for (int j = 0; j < trashNames.size(); j++){
	//			if (trashNames[j] == with[i]){
	//				//右手に衝突した場合
	//				if (mparts[i] == "RARM_LINK7"){
	//					//自分を取得
	//					SimObj *my = getObj(myname());
	//					//自分の手のパーツを得ます
	//					CParts * parts = my->getParts("RARM_LINK7");
	//					if (parts->graspObj(with[i])){
	//						m_grasp = true;
	//						m_graspObjectName = trashNames[j];
	//						sendMsg("VoiceReco_Service", m_graspObjectName);
	//						m_state = 143;
	//					}
	//				}
	//			}
	//		}
	//	}
	//}
}


void DemoRobotController::stopRobotMove(void) {
	RobotObj *m_robotObject = getRobotObj(myname());
	m_robotObject->setWheelVelocity(0.0, 0.0);
}


/*
void DemoRobotController::stopRobotArmMove(void) {
	m_robotObject->setJointVelocity("RARM_JOINT0", 0.0, 0.0);
	m_robotObject->setJointVelocity("RARM_JOINT3", 0.0, 0.0);
}
*/


double DemoRobotController::goToObj(Vector3d pos, double range) {
	// get own position
	Vector3d robotCurrentPosition;
	//RobotObj *m_robotObject = getRobotObj(myname());
	//m_robotObject->getPosition(robotCurrentPosition);
	m_robotObject->getPartsPosition(robotCurrentPosition,"RARM_LINK2");

	// pointing vector for target
	Vector3d l_pos = pos;
	l_pos -= robotCurrentPosition;

	// ignore y-direction
	l_pos.y(0);

	// measure actual distance
	double distance = l_pos.length() - range;

	// start moving
	m_robotObject->setWheelVelocity(m_angularVelocity, m_angularVelocity);

	// time to be elapsed
	double l_time = distance / m_movingSpeed;

	return l_time;
}


double DemoRobotController::rotateTowardObj(Vector3d pos) {  // "pos" means target position
	// get own position
	Vector3d ownPosition;
	//RobotObj *m_robotObject = getRobotObj(myname());
	m_robotObject->getPartsPosition(ownPosition, "RARM_LINK2");

	// pointing vector for target
	Vector3d l_pos = pos;
	l_pos -= ownPosition;

	// ignore variation on y-axis
	l_pos.y(0);

	// get own rotation matrix
	Rotation ownRotation;
	m_robotObject->getRotation(ownRotation);

	// get angles arround y-axis
	double qw = ownRotation.qw();
	double qy = ownRotation.qy();
	double theta = 2*acos(fabs(qw));

	if(qw*qy < 0) theta = -1.0*theta;

	// rotation angle from z-axis to x-axis
	double tmp = l_pos.angle(Vector3d(0.0, 0.0, 1.0));
	double targetAngle = acos(tmp);

	// 方向
	if(l_pos.x() > 0) targetAngle = -1.0*targetAngle;
	targetAngle += theta;
	LOG_MSG(("targetAngle:%.3f\n", targetAngle));
	if (targetAngle > M_PI){
		targetAngle -= 2 * M_PI;
	}
	else if (targetAngle < -M_PI){
		targetAngle += 2 * M_PI;
	}

	LOG_MSG(("targetAngle:%.3f\n", targetAngle));
	double angVelFac = 3.0;
	double l_angvel = m_angularVelocity/angVelFac;

	if(targetAngle == 0.0){
		return 0.0;
	}
	else {
		// 回転すべき円周距離
		double l_distance = m_distance*M_PI*fabs(targetAngle)/(2.0*M_PI);

		// 回転時間(u秒)
		double l_time = l_distance / (m_movingSpeed/angVelFac);

		// 車輪回転開始
		if(targetAngle > 0.0){
			m_robotObject->setWheelVelocity(l_angvel, -l_angvel);
		}
		else{
			m_robotObject->setWheelVelocity(-l_angvel, l_angvel);
		}

		return l_time;
	}
}

void DemoRobotController::recognizeObjectPosition(Vector3d &pos, std::string &name){
	// get object of trash selected
	SimObj *trash = getObj(name.c_str());

	// get trash's position
	trash->getPosition(pos);
}





void DemoRobotController::throwTrash(void){
	// get the part info. 
	//RobotObj *m_robotObject = getRobotObj(myname());
	CParts *parts = m_robotObject->getParts("RARM_LINK7");

	// release grasping
	parts->releaseObj();

	// wait a bit
	sleep(1);

	// set the grasping flag to neutral
	m_grasp = false;
	m_release = true;
}


double DemoRobotController::goGraspingObject(Vector3d &pos){
	double l_time;
	double thetaJoint4 = 20.0;

	//RobotObj *m_robotObject = getRobotObj(myname());
	m_robotObject->setJointVelocity("RARM_JOINT3", m_jointVelocity, 0.0);

	l_time = DEG2RAD(abs(thetaJoint4))/ m_jointVelocity;

	return l_time;
}


void DemoRobotController::neutralizeArms(double evt_time){

	double vel = 0.5;

	//RobotObj *m_robotObject = getRobotObj(myname());
	double angleJoint1 = m_robotObject->getJointAngle("RARM_JOINT0")*180.0 / (M_PI);
	double angleJoint4 = m_robotObject->getJointAngle("RARM_JOINT3")*180.0/(M_PI);
	double thetaJoint1 = -50 - angleJoint1;
	double thetaJoint4 = -70 - angleJoint4;
	LOG_MSG(("\nthetaJoint1:%f\nthetaJoint3:%f\n", thetaJoint1, thetaJoint4));


	/*if(thetaJoint4<0) m_robotObject->setJointVelocity("RARM_JOINT3", -m_jointVelocity, 0.0);
	else m_robotObject->setJointVelocity("RARM_JOINT3", m_jointVelocity, 0.0);

	if(thetaJoint1<0) m_robotObject->setJointVelocity("RARM_JOINT0", -m_jointVelocity, 0.0);
	else m_robotObject->setJointVelocity("RARM_JOINT0", m_jointVelocity, 0.0);

	m_time4 = DEG2RAD(fabs(thetaJoint4)) / m_jointVelocity + evt_time;
	m_time1 = DEG2RAD(fabs(thetaJoint1))/ m_jointVelocity + evt_time;*/

	if (thetaJoint4<0) m_robotObject->setJointVelocity("RARM_JOINT3", -vel, 0.0);
	else m_robotObject->setJointVelocity("RARM_JOINT3", vel, 0.0);

	if (thetaJoint1<0) m_robotObject->setJointVelocity("RARM_JOINT0", -vel, 0.0);
	else m_robotObject->setJointVelocity("RARM_JOINT0", vel, 0.0);

	m_time4 = DEG2RAD(fabs(thetaJoint4)) / vel + evt_time;
	m_time1 = DEG2RAD(fabs(thetaJoint1)) / vel + evt_time;
	
}

void DemoRobotController::prepareThrowing(double evt_time){
	/*double thetaJoint1 = 50.0;
	m_robotObject->setJointVelocity("RARM_JOINT0", -m_jointVelocity, 0.0);
	m_time1 = DEG2RAD(abs(thetaJoint1))/ m_jointVelocity + evt_time;
	double thetaJoint4 = 20.0;
	m_robotObject->setJointVelocity("RARM_JOINT3", m_jointVelocity, 0.0);
	m_time4 = DEG2RAD(abs(thetaJoint4))/ m_jointVelocity + evt_time;*/
	//double angleJoint1 = m_robotObject->getJointAngle("RARM_JOINT0")*180.0 / (M_PI);
	////LOG_MSG(("angleJoint1: %.1f\n", angleJoint1));
	//double thetaJoint1 = 45.0;
	////LOG_MSG(("thetaJoint1: %.1f\n", thetaJoint1));
	////m_robotObject->setJointVelocity("RARM_JOINT0", -m_jointVelocity, 0.0);
	//if (angleJoint1 < thetaJoint1) m_robotObject->setJointVelocity("RARM_JOINT0", -m_jointVelocity, 0.0);
	//else m_robotObject->setJointVelocity("RARM_JOINT0", m_jointVelocity, 0.0);
	//m_time1 = DEG2RAD(abs(thetaJoint1)) / m_jointVelocity + evt_time;
	//double angleJoint4 = m_robotObject->getJointAngle("RARM_JOINT3")*180.0 / (M_PI);
	////LOG_MSG(("angleJoint4: %.1f\n", angleJoint4));
	//double thetaJoint4 = 70.0;
	////LOG_MSG(("thetaJoint4: %.1f\n", thetaJoint4));
	////m_robotObject->setJointVelocity("RARM_JOINT3", m_jointVelocity, 0.0);
	//if (angleJoint4 > thetaJoint4) m_robotObject->setJointVelocity("RARM_JOINT3", -m_jointVelocity, 0.0);
	//else m_robotObject->setJointVelocity("RARM_JOINT3", m_jointVelocity, 0.0);
	//m_time4 = DEG2RAD(abs(thetaJoint4)) / m_jointVelocity + evt_time;
	//LOG_MSG(("\nthetaJoint1:%f\nthetaJoint4:%f\n", thetaJoint1, thetaJoint4));

	double vel = 0.5;

	//RobotObj *m_robotObject = getRobotObj(myname());
	double angleJoint1 = m_robotObject->getJointAngle("RARM_JOINT0")*180.0 / (M_PI);
	double angleJoint4 = m_robotObject->getJointAngle("RARM_JOINT3")*180.0 / (M_PI);
	double thetaJoint1 = -50 - angleJoint1;
	double thetaJoint4 = -60 - angleJoint4;
	LOG_MSG(("\nthetaJoint1:%f\nthetaJoint3:%f\n", thetaJoint1, thetaJoint4));

	/*if (thetaJoint1<0) m_robotObject->setJointVelocity("RARM_JOINT0", -m_jointVelocity, 0.0);
	else m_robotObject->setJointVelocity("RARM_JOINT0", m_jointVelocity, 0.0);

	if (thetaJoint4<0) m_robotObject->setJointVelocity("RARM_JOINT3", -m_jointVelocity, 0.0);
	else m_robotObject->setJointVelocity("RARM_JOINT3", m_jointVelocity, 0.0);*/

	/*m_time4 = DEG2RAD(fabs(thetaJoint4)) / m_jointVelocity + evt_time;
	m_time1 = DEG2RAD(fabs(thetaJoint1)) / m_jointVelocity + evt_time;*/

	if (thetaJoint1<0) m_robotObject->setJointVelocity("RARM_JOINT0", -vel, 0.0);
	else m_robotObject->setJointVelocity("RARM_JOINT0", vel, 0.0);

	if (thetaJoint4<0) m_robotObject->setJointVelocity("RARM_JOINT3", -vel, 0.0);
	else m_robotObject->setJointVelocity("RARM_JOINT3", vel, 0.0);

	m_time4 = DEG2RAD(fabs(thetaJoint4)) / vel + evt_time;
	m_time1 = DEG2RAD(fabs(thetaJoint1)) / vel + evt_time;
}

//********************************************************************
extern "C" Controller * createController() {  
  return new DemoRobotController;  
}  