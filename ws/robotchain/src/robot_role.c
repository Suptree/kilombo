#include "robot_role.h"

// private メンバ関数はstaticでスコープ隠蔽
static void robot_role_clear( RobotRole* const p_this );

// また、private関数の接頭辞は小文字で始めるなどのルールにより
// public/privateを分かりやすくすることができます
static void robot_role_clear( RobotRole* const p_this )
{
    // 第一引数に自分自身のインスタンスへのポインタ(this)が格納されるように呼び出すので
    // 第一引数のポインタを経由してメンバ変数へアクセスします
    p_this->privateNumber = 0;
}

// コンストラクタは自動生成されないため必ず定義します
void robot_role_construct(RobotRole* const p_this)
{
    robot_role_clear(p_this);
}

void robot_role_setNumber( RobotRole* const p_this, int num )
{
    p_this->privateNumber = num;
}

int robot_role_getNumber( const RobotRole* const p_this )
{
    return p_this->privateNumber;
}