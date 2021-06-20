#ifndef ROBOT_ROLE_H
#define ROBOT_ROLE_H

// クラス定義
typedef struct robotrole
{
  // メンバ変数
  // 言語レベルでは隠蔽できないため命名規則でカプセル化を実現
  int bottype;

}RobotRole;

// public メンバ関数はヘッダに定義します
enum BOTTYPE {
  NEST,
  FOOD,
  ROBOT
};

// 接頭辞としてクラス名をつけることでスコープを分けることができます

// 第一引数に必ず自身の構造体のポインタ型を定義してthis(インスタンスへのポインタ)として扱います
// メンバ関数の引数は第二引数以降に定義
void RobotRole_construct( RobotRole* const p_this );
void RobotRole_setbottype( RobotRole* const p_this, int num );
int  RobotRole_getbottype( const RobotRole* const p_this );

#endif