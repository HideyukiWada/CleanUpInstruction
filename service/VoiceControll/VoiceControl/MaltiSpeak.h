#include<stdio.h>
#include<stdlib.h>
#include<string>
#include<time.h>
#include<windows.h>
#include<vector>
//#include<thread>
//#include<mutex>

class MaltiSpeak{
private:
	void Speak(std::string message, int time);
	void waitFinish(int waitTime);
	bool talkFinishFlag;
	FILE *fp;
	//std::mutex m;
	//std::thread timeThread;

public:
	~MaltiSpeak();

	/**
	 * initialize
	 *
	 * 初期化関数　最初に呼ぶ
	 */
	void initialize();

	/**
	 * noWaitSpeak
	 *
	 * 発話関数　関数を呼び出すと発話を開始するが，関数はすぐに抜けだす．発話の終わりは検知出来ない．
	 * 引数
	 *　message 発話する文字列
	 */
	void noWaitSpeak(std::string message);

	/**
	 * waitSpeak
	 *
	 * 発話関数　発話中指定した秒数関数内で待つ．　関数内でSleepを呼んでいる為，発話中何もできない．
	 *
	 * 引数
	 *　message 発話する文字列
	 *　waitTime 発話の間待つ時間　文字列によって異なる為，調整する必要あり
	 */
	void waitSpeak(std::string message, int waitTime);

	/**
	 * joinSpeak
	 *
	 * 発話関数　関数を呼び出すと発話する．スレッドを独立して実行．関数はすぐ抜け出すが，waitJoin関数で発話の終了待ちを後から呼ぶことが出来る．
	 * 　　　　　またwaitJoin関数を呼ぶ前に再度joinSpeakが実行された場合前に実行された発話が完了した後に発話する．
	 *
	 * 引数
	 *　message 発話する文字列
	 *　talkTime 発話する時間　文字列によって異なる為，調整する必要あり
	 */
	void joinSpeak(std::string message, int talkTime);

	/**
	 * waitJoin
	 *
	 * joinSpeak関数で実行された発話の終了を待つ関数．　発話を別スレッドで実行後合流時に使用
	 * joinSpeak関数が発話中の場合発話終了まで待ち，発話が終わっていた場合はすぐに終了．
	 * 動作と同期させたいときなどに用いてください．
	 */
	void waitJoin();

	/**
	 * speakState
	 *
	 * 現在の発話状態を戻り値とする関数
	 * waitSpeak関数とjoinSpeak関数のみ有効．　noWaitSpeakを使用した場合必ずtrueになるので注意
	 *
	 * 戻り値
	 *　bool
	 *　　発話中の場合false
	 *　　発話が完了している場合true
	 */
	bool speakState();

	/**
	 * file_name
	 *
	 * OpenJTalkの発話内容が書いてあるテキストファイルのパス
	 */
	std::string file_name;
};