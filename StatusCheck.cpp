/*
 * StatusCheck.cpp
 *
 *  Created on: 2011. 1. 21.
 *      Author: zerom
 */

#include <stdio.h>
#include <unistd.h>

#include "StatusCheck.h"
#include "Head.h"
#include "Action.h"
#include "Walking.h"
#include "MotionStatus.h"
#include "MotionManager.h"
#include "LinuxActionScript.h"

using namespace Robot;

void StatusCheck::Check(CM730 &cm730)
{
    if(MotionStatus::FALLEN != STANDUP)
    {
        Walking::GetInstance()->Stop();
        while(Walking::GetInstance()->IsRunning() == 1) usleep(8000);

        Action::GetInstance()->m_Joint.SetEnableBody(true, true);

        if(MotionStatus::FALLEN == FORWARD)
            Action::GetInstance()->Start(10);   // FORWARD GETUP
        else if(MotionStatus::FALLEN == BACKWARD)
            Action::GetInstance()->Start(11);   // BACKWARD GETUP

        while(Action::GetInstance()->IsRunning() == 1) usleep(8000);

        Head::GetInstance()->m_Joint.SetEnableHeadOnly(true, true);
        Walking::GetInstance()->m_Joint.SetEnableBodyWithoutHead(true, true);
    }

}
