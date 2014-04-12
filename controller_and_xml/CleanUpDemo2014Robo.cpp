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
	/*RobotObj *m_robotObject;*/
	ViewService *m_view;

	int m_state; 
	double refreshRateOnAction;

	std::string m_trashName1;
	std::string m_trashName2;
	std::string m_trashName3;
	std::vector<std::string> trashNames;

	std::string m_graspObjectName;

	std::string m_trashBoxName1;
	std::string m_trashBoxName2;
	std::string m_trashBoxName3;
	std::string m_trashBoxName4;
	std::string trashBoxName;

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

	//positions
	Vector3d m_frontTrashBox1;
	Vector3d m_frontTrashBox2;
	Vector3d m_frontTrashBox3;
	Vector3d m_relayPoint1;
	Vector3d m_frontTrash1;
	Vector3d m_frontTrash2;
	Vector3d m_waitPosition;
	Vector3d m_userPosition;
	Vector3d frontTrashBox;

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
	/*m_robotObject = getRobotObj(myname());*/

	// set wheel configuration
	m_radius = 10.0;
	m_distance = 10.0;
	RobotObj *m_robotObject = getRobotObj(myname());
	m_robotObject->setWheel(m_radius, m_distance);

	m_time = 0.0;
	m_time1 = 0.0;
	m_time4 = 0.0;

	m_state = 0;  // switch of initial behavior
	//m_state = 9140;


	refreshRateOnAction = 0.1;     // refresh-rate for onAction proc.

	// angular velocity of wheel and moving speed of robot
	m_angularVelocity = 1.5;
	m_movingSpeed = m_angularVelocity*m_radius;  // conversion: rad/ms -> m/ms)

	// rotation speed of joint
	m_jointVelocity = 0.5;

	m_trashName1 = "petbottle_1";
	m_trashName2 = "can_0";
	m_trashName3 = "kettle";

	m_trashBoxName1 = "trashbox_0";  // for recycle
	m_trashBoxName2 = "trashbox_1";  // for burnable

	trashNames.push_back(m_trashName1);
	trashNames.push_back(m_trashName2);
	trashNames.push_back(m_trashName3);

	//m_graspObjectName = m_trashName2;

	userName = "man_000";

	// set positions;
	m_frontTrashBox1 = Vector3d(-50.0, 0.0, -90);  // for recycle material
	m_frontTrashBox2 = Vector3d(50.0, 0.0, -90);  // for burnable material
	m_frontTrashBox3 = Vector3d(150.0, 0.0, -90);
	m_relayPoint1    = Vector3d(190.0, 0.0, -65.0);
	m_frontTrash1    = Vector3d(273.0, 0.0, -65.0);
	m_frontTrash2    = Vector3d(305.0, 0.0, -80.0);
	m_waitPosition   = Vector3d(185.0, 30.0, -70.0);
	m_userPosition   = Vector3d(280.0, 30.0, -50.0);

	m_grasp = false;
	m_release = true;

	error_count = 0;
}


double DemoRobotController::onAction(ActionEvent &evt) {
	switch (m_state){
	case 0: {
				break;
	}
	case 140: {
				this->neutralizeArms(evt.time());
				m_state = 141;
				LOG_MSG(("m_state:%d\n", m_state));
				sendMsg("VoiceReco_Service", "start");
				break;
	}
	case 141: {

				RobotObj *m_robotObject = getRobotObj(myname());
				double angleJoint1 = m_robotObject->getJointAngle("RARM_JOINT1")*180.0 / (M_PI);
				double angleJoint4 = m_robotObject->getJointAngle("RARM_JOINT4")*180.0 / (M_PI);

				LOG_MSG(("\nm_time1:%4f JOINT1 angle:%4f\nm_time4:%4f JOINT4 angle:%4f\n", m_time1, angleJoint1, m_time4, angleJoint4));
				if (evt.time() >= m_time4 && m_state == 141) m_robotObject->setJointVelocity("RARM_JOINT4", 0.0, 0.0);
				if (evt.time() >= m_time1 && m_state == 141) m_robotObject->setJointVelocity("RARM_JOINT1", 0.0, 0.0);
				if (evt.time() >= m_time1 && evt.time() >= m_time4 && m_state == 141){
					this->stopRobotMove();
					sendMsg("VoiceReco_Service", "please pass my hand");
					m_state = 142;
					//m_state = 161;
					//m_time = 6.0;

					LOG_MSG(("m_state:%d\n", m_state));
				}
				break;
	}
		//手渡し対機
	case 142: {
				if (m_grasp == false && m_release == false){
					//自分を取得
					RobotObj *m_robotObject = getRobotObj(myname());
					//自分の手のパーツを得ます
					CParts * parts = m_robotObject->getParts("RARM_LINK7");
					sendMsg("SIGViewer", m_graspObjectName);
					if (parts->graspObj(m_graspObjectName)){
						m_grasp = true;
						sendMsg("VoiceReco_Service", m_graspObjectName);
						m_state = 143;
						LOG_MSG(("m_state:%d\n", m_state));

						//運ぶ物体を対応表に記録
						std::string msg = "object";
						msg += " " + m_graspObjectName;
						sendMsg("Memorize_Service", msg);
					}
				}
				break;
	}
	case 143: {
				double l_moveTime = rotateTowardObj(Vector3d(10.0, 0.0, -90));

				m_time = l_moveTime + evt.time();
				m_state = 144;
				LOG_MSG(("m_state:%d\n", m_state));
				break;
	}
	case 144: {
				if (evt.time() >= m_time && m_state == 144){
					this->stopRobotMove();
					sendMsg("VoiceReco_Service", "please choose the trashbox");
					m_state = 145;
					LOG_MSG(("m_state:%d\n", m_state));
				}
				break;
	}
		//発話対機状態
	case 145: {
			  break;
	}
	case 150: {
				double l_moveTime = rotateTowardObj(frontTrashBox);

				m_time = l_moveTime + evt.time();
				m_state = 160;
				LOG_MSG(("m_state:%d\n", m_state));
				break;
	}
	case 160: {
				if (evt.time() >= m_time && m_state == 160){
					this->stopRobotMove();
					double l_moveTime = goToObj(frontTrashBox, 0.0);
					m_time = l_moveTime + evt.time();
					m_state = 161;
					LOG_MSG(("m_state:%d\n", m_state));
				}
				break;
	}
	case 161: {
				if (evt.time() >= m_time && m_state == 161){
					this->stopRobotMove();
					this->prepareThrowing(evt.time());

					m_state = 165;
					LOG_MSG(("m_state:%d\n", m_state));
				}
				break;
	}
	case 165: {
				RobotObj *m_robotObject = getRobotObj(myname());
				double angleJoint1 = m_robotObject->getJointAngle("RARM_JOINT1")*180.0 / (M_PI);
				double angleJoint4 = m_robotObject->getJointAngle("RARM_JOINT4")*180.0 / (M_PI);

				LOG_MSG(("\nm_time1:%4f JOINT1 angle:%4f\nm_time4:%4f JOINT4 angle:%4f\n", m_time1, angleJoint1, m_time4, angleJoint4));
				if (evt.time() >= m_time4 && m_state == 165) m_robotObject->setJointVelocity("RARM_JOINT4", 0.0, 0.0);
				if (evt.time() >= m_time1 && m_state == 165) m_robotObject->setJointVelocity("RARM_JOINT1", 0.0, 0.0);
				if (evt.time() >= m_time1 && evt.time() >= m_time4 && m_state == 165){
					this->stopRobotMove();
					Vector3d l_tpos;
					this->recognizeObjectPosition(l_tpos, trashBoxName);
					double l_moveTime = rotateTowardObj(l_tpos);
					m_time = l_moveTime + evt.time();

					m_state = 170;
					LOG_MSG(("m_state:%d\n", m_state));
				}
				break;
	}
	case 170: {
				if (evt.time() >= m_time && m_state == 170){

					this->stopRobotMove();
					Vector3d l_tpos;
					this->recognizeObjectPosition(l_tpos, trashBoxName);
					double l_moveTime = goToObj(l_tpos, 50.0);
					m_time = l_moveTime + evt.time();

					m_state = 180;
					LOG_MSG(("m_state:%d\n", m_state));
				}
				break;
	}
	case 180: {
				if (evt.time() >= m_time && m_state == 180){
					this->stopRobotMove();
					Vector3d l_tpos;
					this->recognizeObjectPosition(l_tpos, trashBoxName);
					double l_moveTime = rotateTowardObj(l_tpos);
					m_time = l_moveTime + evt.time();

					m_state = 200;
					LOG_MSG(("m_state:%d\n", m_state));
				}
				break;
	}
	case 200: {  // throw trash and get back a bit
				if (evt.time() >= m_time && m_state == 200){
					this->stopRobotMove();
					this->throwTrash();

					//運び先を対応表に記録
					std::string msg = "storageSpace";
					msg += " " + trashBoxName;
					sendMsg("Memorize_Service", msg);

					sleep(1);

					RobotObj *m_robotObject = getRobotObj(myname());
					m_robotObject->setWheelVelocity(-m_angularVelocity, -m_angularVelocity);
					m_time = 80.0 / m_movingSpeed + evt.time();

					m_state = 225;
					LOG_MSG(("m_state:%d\n", m_state));
				}
				break;
	}
	case 225: {  // recover robot arms
				if (evt.time() >= m_time && m_state == 225){
					this->stopRobotMove();
					this->neutralizeArms(evt.time());

					m_state = 230;
					LOG_MSG(("m_state:%d\n", m_state));
				}
				break;
	}
	case 230: {
				  RobotObj *m_robotObject = getRobotObj(myname());
				  double angleJoint1 = m_robotObject->getJointAngle("RARM_JOINT1")*180.0 / (M_PI);
				  double angleJoint4 = m_robotObject->getJointAngle("RARM_JOINT4")*180.0 / (M_PI);

				  LOG_MSG(("\nm_time1:%4f JOINT1 angle:%4f\nm_time4:%4f JOINT4 angle:%4f\n", m_time1, angleJoint1, m_time4, angleJoint4));

				if (evt.time() >= m_time4 && m_state == 230) m_robotObject->setJointVelocity("RARM_JOINT4", 0.0, 0.0);
				if (evt.time() >= m_time1 && m_state == 230) m_robotObject->setJointVelocity("RARM_JOINT1", 0.0, 0.0);
				if (evt.time() >= m_time1 && evt.time() >= m_time4 && m_state == 230){
					double l_moveTime = rotateTowardObj(m_waitPosition);

					m_time = l_moveTime + evt.time();
					m_state = 231;
					LOG_MSG(("m_state:%d\n", m_state));
				}
				break;
	}
	case 231: {
				if (evt.time() >= m_time && m_state == 231){
					this->stopRobotMove();
					double l_moveTime = goToObj(m_waitPosition, 0.0);
					m_time = l_moveTime + evt.time();
					m_state = 232;
					LOG_MSG(("m_state:%d\n", m_state));
				}
				break;
	}
	case 232: {
				if (evt.time() >= m_time && m_state == 232){
					double l_moveTime = rotateTowardObj(m_userPosition);

					m_time = l_moveTime + evt.time();
					m_state = 233;
					LOG_MSG(("m_state:%d\n", m_state));
				}
				break;
	}
	case 233: {
				if (evt.time() >= m_time && m_state == 233){
					this->stopRobotMove();
					this->neutralizeArms(evt.time());

					m_state = 141;
					LOG_MSG(("m_state:%d\n", m_state));
				}
				break;
	}
	case 234: {
				  RobotObj *m_robotObject = getRobotObj(myname());
				  double angleJoint1 = m_robotObject->getJointAngle("RARM_JOINT1")*180.0 / (M_PI);
				  double angleJoint4 = m_robotObject->getJointAngle("RARM_JOINT4")*180.0 / (M_PI);

				  LOG_MSG(("\nm_time1:%4f JOINT1 angle:%4f\nm_time4:%4f JOINT4 angle:%4f\n", m_time1, angleJoint1, m_time4, angleJoint4));

				if (evt.time() >= m_time4 && m_state == 234) m_robotObject->setJointVelocity("RARM_JOINT4", 0.0, 0.0);
				if (evt.time() >= m_time1 && m_state == 234) m_robotObject->setJointVelocity("RARM_JOINT1", 0.0, 0.0);
				if (evt.time() >= m_time1 && evt.time() >= m_time4 && m_state == 234){
					this->stopRobotMove();
					m_state = 141;
					LOG_MSG(("m_state:%d\n", m_state));
				}
				break;
	}

	//デバッグ用
	case 9140: {
				  this->neutralizeArms(evt.time());
				  m_state = 9141;
				  LOG_MSG(("m_state:%d\n", m_state));
				  sendMsg("VoiceReco_Service", "start");
				  break;
	}
	case 9141: {

				  RobotObj *m_robotObject = getRobotObj(myname());
				  double angleJoint1 = m_robotObject->getJointAngle("RARM_JOINT1")*180.0 / (M_PI);
				  double angleJoint4 = m_robotObject->getJointAngle("RARM_JOINT4")*180.0 / (M_PI);

				  LOG_MSG(("\nm_time1:%4f JOINT1 angle:%4f\nm_time4:%4f JOINT4 angle:%4f\n", m_time1, angleJoint1, m_time4, angleJoint4));
				  if (evt.time() >= m_time4 && m_state == 9141) m_robotObject->setJointVelocity("RARM_JOINT4", 0.0, 0.0);
				  if (evt.time() >= m_time1 && m_state == 9141) m_robotObject->setJointVelocity("RARM_JOINT1", 0.0, 0.0);
				  if (evt.time() >= m_time1 && evt.time() >= m_time4 && m_state == 9141){
					  this->stopRobotMove();
					  sendMsg("VoiceReco_Service", "please pass my hand");
					  //m_state = 9142;
					  m_state = 9161;
					  m_time = 6.0;

					  LOG_MSG(("m_state:%d\n", m_state));
				  }
				  break;
	}
	case 9142: {
				  if (m_grasp == false && m_release == false){
					  //自分を取得
					  RobotObj *m_robotObject = getRobotObj(myname());
					  //自分の手のパーツを得ます
					  CParts * parts = m_robotObject->getParts("RARM_LINK7");
					  sendMsg("SIGViewer", m_graspObjectName);
					  if (parts->graspObj(m_graspObjectName)){
						  m_grasp = true;
						  sendMsg("VoiceReco_Service", m_graspObjectName);
						  m_state = 9161;
						  m_time = evt.time() + 3.0;
						  LOG_MSG(("m_state:%d\n", m_state));
					  }
				  }
				  break;
	}
	case 9161: {
				  if (evt.time() >= m_time && m_state == 9161){
					  this->stopRobotMove();
					  this->prepareThrowing(evt.time());

					  m_state = 9165;
					  LOG_MSG(("m_state:%d\n", m_state));
				  }
				  break;
	}
	case 9165: {
				  RobotObj *m_robotObject = getRobotObj(myname());
				  double angleJoint1 = m_robotObject->getJointAngle("RARM_JOINT1")*180.0 / (M_PI);
				  double angleJoint4 = m_robotObject->getJointAngle("RARM_JOINT4")*180.0 / (M_PI);

				  LOG_MSG(("\nm_time1:%4f JOINT1 angle:%4f\nm_time4:%4f JOINT4 angle:%4f\n", m_time1, angleJoint1, m_time4, angleJoint4));
				  if (evt.time() >= m_time4 && m_state == 9165) m_robotObject->setJointVelocity("RARM_JOINT4", 0.0, 0.0);
				  if (evt.time() >= m_time1 && m_state == 9165) m_robotObject->setJointVelocity("RARM_JOINT1", 0.0, 0.0);
				  if (evt.time() >= m_time1 && evt.time() >= m_time4 && m_state == 9165){
					  this->stopRobotMove();
					  Vector3d l_tpos;
					  this->recognizeObjectPosition(l_tpos, trashBoxName);
					  double l_moveTime = rotateTowardObj(l_tpos);
					  m_time = l_moveTime + evt.time();

					  m_state = 9140;
					  LOG_MSG(("m_state:%d\n", m_state));
				  }
				  break;
	}

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
		else if (str == "trashbox_0"){
			sendMsg("VoiceReco_Service", str);
			trashBoxName = str;
			frontTrashBox = m_frontTrashBox1;
			sendMsg("SIGViewer", "Recognition Success!!");
			error_count = 0;
			m_state = 150;
			LOG_MSG(("m_state:%d\n", m_state));
		}
		else if (str == "trashbox_1"){
			sendMsg("VoiceReco_Service", str);
			trashBoxName = str;
			frontTrashBox = m_frontTrashBox2;
			sendMsg("SIGViewer", "Recognition Success!!");
			error_count = 0;
			m_state = 150;
			LOG_MSG(("m_state:%d\n", m_state));
		}
		else if (str == "trashbox_2"){
			sendMsg("VoiceReco_Service", str);
			trashBoxName = str;
			frontTrashBox = m_frontTrashBox3;
			sendMsg("SIGViewer", "Recognition Success!!");
			error_count = 0;
			m_state = 150;
			LOG_MSG(("m_state:%d\n", m_state));
		}
		else if (str == "wagon_0"){
			sendMsg("VoiceReco_Service", str);
			trashBoxName = str;
			frontTrashBox = m_frontTrashBox3;
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
	m_robotObject->setJointVelocity("RARM_JOINT1", 0.0, 0.0);
	m_robotObject->setJointVelocity("RARM_JOINT4", 0.0, 0.0);
}
*/


double DemoRobotController::goToObj(Vector3d pos, double range) {
	// get own position
	Vector3d robotCurrentPosition;
	RobotObj *m_robotObject = getRobotObj(myname());
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
	RobotObj *m_robotObject = getRobotObj(myname());
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


void DemoRobotController::prepareThrowing(double evt_time){
	/*double thetaJoint1 = 50.0;
	m_robotObject->setJointVelocity("RARM_JOINT1", -m_jointVelocity, 0.0);
	m_time1 = DEG2RAD(abs(thetaJoint1))/ m_jointVelocity + evt_time;

	double thetaJoint4 = 20.0;
	m_robotObject->setJointVelocity("RARM_JOINT4", m_jointVelocity, 0.0);
	m_time4 = DEG2RAD(abs(thetaJoint4))/ m_jointVelocity + evt_time;*/

	//double angleJoint1 = m_robotObject->getJointAngle("RARM_JOINT1")*180.0 / (M_PI);
	////LOG_MSG(("angleJoint1: %.1f\n", angleJoint1));
	//double thetaJoint1 = 45.0;
	////LOG_MSG(("thetaJoint1: %.1f\n", thetaJoint1));
	////m_robotObject->setJointVelocity("RARM_JOINT1", -m_jointVelocity, 0.0);
	//if (angleJoint1 < thetaJoint1) m_robotObject->setJointVelocity("RARM_JOINT1", -m_jointVelocity, 0.0);
	//else m_robotObject->setJointVelocity("RARM_JOINT1", m_jointVelocity, 0.0);
	//m_time1 = DEG2RAD(abs(thetaJoint1)) / m_jointVelocity + evt_time;

	//double angleJoint4 = m_robotObject->getJointAngle("RARM_JOINT4")*180.0 / (M_PI);
	////LOG_MSG(("angleJoint4: %.1f\n", angleJoint4));
	//double thetaJoint4 = 70.0;
	////LOG_MSG(("thetaJoint4: %.1f\n", thetaJoint4));
	////m_robotObject->setJointVelocity("RARM_JOINT4", m_jointVelocity, 0.0);
	//if (angleJoint4 > thetaJoint4) m_robotObject->setJointVelocity("RARM_JOINT4", -m_jointVelocity, 0.0);
	//else m_robotObject->setJointVelocity("RARM_JOINT4", m_jointVelocity, 0.0);
	//m_time4 = DEG2RAD(abs(thetaJoint4)) / m_jointVelocity + evt_time;
	//LOG_MSG(("\nthetaJoint1:%f\nthetaJoint4:%f\n", thetaJoint1, thetaJoint4));

	RobotObj *m_robotObject = getRobotObj(myname());
	double angleJoint1 = m_robotObject->getJointAngle("RARM_JOINT1")*180.0 / (M_PI);
	double angleJoint4 = m_robotObject->getJointAngle("RARM_JOINT4")*180.0 / (M_PI);
	double thetaJoint1 = -50 - angleJoint1;
	double thetaJoint4 = -60 - angleJoint4;
	LOG_MSG(("\nthetaJoint1:%f\nthetaJoint4:%f\n", thetaJoint1, thetaJoint4));


	

	if (thetaJoint1<0) m_robotObject->setJointVelocity("RARM_JOINT1", -m_jointVelocity, 0.0);
	else m_robotObject->setJointVelocity("RARM_JOINT1", m_jointVelocity, 0.0);

	if (thetaJoint4<0) m_robotObject->setJointVelocity("RARM_JOINT4", -m_jointVelocity, 0.0);
	else m_robotObject->setJointVelocity("RARM_JOINT4", m_jointVelocity, 0.0);

	m_time4 = DEG2RAD(fabs(thetaJoint4)) / m_jointVelocity + evt_time;
	m_time1 = DEG2RAD(fabs(thetaJoint1)) / m_jointVelocity + evt_time;
}


void DemoRobotController::throwTrash(void){
	// get the part info. 
	RobotObj *m_robotObject = getRobotObj(myname());
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

	RobotObj *m_robotObject = getRobotObj(myname());
	m_robotObject->setJointVelocity("RARM_JOINT4", m_jointVelocity, 0.0);

	l_time = DEG2RAD(abs(thetaJoint4))/ m_jointVelocity;

	return l_time;
}


void DemoRobotController::neutralizeArms(double evt_time){
	RobotObj *m_robotObject = getRobotObj(myname());
	double angleJoint1 = m_robotObject->getJointAngle("RARM_JOINT1")*180.0 / (M_PI);
	double angleJoint4 = m_robotObject->getJointAngle("RARM_JOINT4")*180.0/(M_PI);
	double thetaJoint1 = -50 - angleJoint1;
	double thetaJoint4 = -70 - angleJoint4;
	LOG_MSG(("\nthetaJoint1:%f\nthetaJoint4:%f\n", thetaJoint1, thetaJoint4));


	if(thetaJoint4<0) m_robotObject->setJointVelocity("RARM_JOINT4", -m_jointVelocity, 0.0);
	else m_robotObject->setJointVelocity("RARM_JOINT4", m_jointVelocity, 0.0);

	if(thetaJoint1<0) m_robotObject->setJointVelocity("RARM_JOINT1", -m_jointVelocity, 0.0);
	else m_robotObject->setJointVelocity("RARM_JOINT1", m_jointVelocity, 0.0);

	m_time4 = DEG2RAD(fabs(thetaJoint4)) / m_jointVelocity + evt_time;
	m_time1 = DEG2RAD(fabs(thetaJoint1))/ m_jointVelocity + evt_time;
	
}


//********************************************************************
extern "C" Controller * createController() {  
  return new DemoRobotController;  
}  
