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

	// ゴミ箱のサイズ(この範囲でreleaseしなければゴミを捨てられない)
	double tableSize_x, tableSize_z, tableHeight;

	// ゴミが入ったとされる高さ方向の範囲(y方向)
	double tableMin_y, tableMax_y;

	int trash_num;

};

void MyController::onInit(InitEvent &evt) {
	m_my = getObj(myname());
	getAllEntities(m_entities);

	// ゴミの大きさ
	// この範囲でゴミをreleaseするとゴミを捨てたと判定
	tableSize_x = 85;
	tableSize_z = 140;
	tableHeight = 75.1 / 2. + 25;
	tableMin_y = 40.0;
	tableMax_y = 1000.0;
	trash_num = 5;
}

double MyController::onAction(ActionEvent &evt)
{
	// 自分の位置取得
	Vector3d myPos;
	m_my->getPosition(myPos);
	//LOG_MSG(("table position [x:%.2lf y:%.2lf z:%.2lf]", myPos.x(), myPos.y(), myPos.z()));

	int entSize = m_entities.size();
	//ゴミ箱への対応
	for (int i = 0; i < entSize; i++){

		// 掴めるオブジェクト以外は除く
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
		// エンティティ取得
		SimObj *ent = getObj(m_entities[i].c_str());

		// 位置取得
		Vector3d tpos;
		ent->getPosition(tpos);

		// ゴミ箱からゴミを結ぶベクトル
		Vector3d vec(tpos.x() - myPos.x(), tpos.y() - myPos.y(), tpos.z() - myPos.z());

		// ゴミがゴミ箱の中に入ったかどうか判定
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

