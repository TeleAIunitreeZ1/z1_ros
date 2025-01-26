#include "unitree_arm_sdk/control/unitreeArm.h"

int main()
{   //创建了一个名为arm的unitreeArm类的实例，构造函数中传递的true可能是用于初始化或配置机械臂
    UNITREE_ARM::unitreeArm arm(true);
    //启动了机械臂的发送接收线程，这可能是用于与机械臂通信的线程。
    arm.sendRecvThread->start();
    //调用backToStart函数，可能是将机械臂移动到起始位置或重置其状态。
    arm.backToStart();
    //设置机械臂的状态跟踪为JOINTCTRL，即关节控制模式
    arm.startTrack(UNITREE_ARM::ArmFSMState::JOINTCTRL);

    double duration = 1000.;
    Vec6 targetPos, lastPos;
    lastPos = arm.lowstate->getQ();
    targetPos << 0.0, 1.5, -1.0, -0.54, 0.0, 0.0;

    UNITREE_ARM::Timer timer(arm._ctrlComp->dt);
    for(int i=0; i<duration; i++)
    {
        arm.q = lastPos*(1-i/duration) + targetPos*(i/duration);
        arm.qd = (targetPos-lastPos)/(duration*arm._ctrlComp->dt);
        arm.setArmCmd(arm.q, arm.qd);
        timer.sleep();
    }
    //在移动完成后，再次调用backToStart函数，可能是将机械臂移动回起始位置或进行状态重置。
    arm.backToStart();
    //设置机械臂的有限状态机为PASSIVE，即被动模式。
    arm.setFsm(UNITREE_ARM::ArmFSMState::PASSIVE);
    //关闭发送接收线程，结束与机械臂的通信。
    arm.sendRecvThread->shutdown();
    return 0;
}
