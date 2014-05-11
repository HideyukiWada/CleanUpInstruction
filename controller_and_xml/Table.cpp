#include "ControllerEvent.h"  
#include "Controller.h"  
#include "Logger.h"  
#include <sstream>

class MyController : public Controller {
public:
	void onInit(InitEvent &evt);
	double onAction(ActionEvent&);
	void onRecvMsg(RecvMsgEvent &evt);
	void onCollision(CollisionEvent &evt);

private:
	SimObj *m_my;
	std::vector<std::string> m_entities;

	// �S�~���̃T�C�Y(���͈̔͂�release���Ȃ���΃S�~���̂Ă��Ȃ�)
	double tableSize_x, tableSize_z, tableHeight;

	// �S�~���������Ƃ���鍂�������͈̔�(y����)
	double tableMin_y, tableMax_y;

	int trash_num;

};

void MyController::onInit(InitEvent &evt) {
	m_my = getObj(myname());
	getAllEntities(m_entities);

	// �S�~�̑傫��
	// ���͈̔͂ŃS�~��release����ƃS�~���̂Ă��Ɣ���
	tableSize_x = 85;
	tableSize_z = 140;
	tableHeight = 75.1 / 2. + 25;
	tableMin_y = 40.0;
	tableMax_y = 1000.0;
	trash_num = 5;
}

double MyController::onAction(ActionEvent &evt)
{
	// �����̈ʒu�擾
	Vector3d myPos;
	m_my->getPosition(myPos);
	//LOG_MSG(("table position [x:%.2lf y:%.2lf z:%.2lf]", myPos.x(), myPos.y(), myPos.z()));

	int entSize = m_entities.size();
	//�S�~���ւ̑Ή�
	for (int i = 0; i < entSize; i++){

		// �͂߂�I�u�W�F�N�g�ȊO�͏���
		if (!(m_entities[i] == "Clock"   ||
			  m_entities[i] == "Bear"    ||
			  m_entities[i] == "Penguin" ||
			  m_entities[i] == "Cup"     )){
			continue;
		}
		double objHeight;
		if (m_entities[i] == "Clock"){
			objHeight = 10;
		}
		if (m_entities[i] == "Bear"){
			objHeight = 19.8;
		}
		if (m_entities[i] == "Penguin"){
			objHeight = 12.3;
		}
		if (m_entities[i] == "Cup"){
			objHeight = 7.42;
		}
		// �G���e�B�e�B�擾
		SimObj *ent = getObj(m_entities[i].c_str());

		// �ʒu�擾
		Vector3d tpos;
		ent->getPosition(tpos);

		// �S�~������S�~�����ԃx�N�g��
		Vector3d vec(tpos.x() - myPos.x(), tpos.y() - myPos.y(), tpos.z() - myPos.z());

		// �S�~���S�~���̒��ɓ��������ǂ�������
		if (abs(vec.x()) < tableSize_x / 2.0 &&
			abs(vec.z()) < tableSize_z / 2.0 &&
			tpos.y() < tableMax_y     &&
			tpos.y() > tableMin_y){
			if (!ent->getIsGrasped()){
				ent->setPosition(Vector3d(tpos.x(), tableHeight + objHeight/2., tpos.z()));
			}
		}
	}
	return 0.1;
}

void MyController::onRecvMsg(RecvMsgEvent &evt) {
}

void MyController::onCollision(CollisionEvent &evt) {
}

extern "C" Controller * createController() {
	return new MyController;
}

