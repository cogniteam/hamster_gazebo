
#include "Wandering.h"



FSM(Wandering){
FSM_STATES
{
    TurnRandom,
    TurnLeft,
    TurnRight,
    Drive,
    Pause,
    DriveBackward,
    Stop
}
//FSM_START(Drive);
FSM_START(Pause);
FSM_BGN
{
    FSM_STATE(TurnRandom)
    {

        FSM_CALL_TASK(TurnRandom)
        FSM_TRANSITIONS
        {
            FSM_ON_EVENT("/PAUSE", FSM_NEXT(Pause))
            FSM_ON_EVENT("/TIMEOUT_TURN", FSM_NEXT(Drive))
            FSM_ON_EVENT("/LEFT_OBSTACLE", FSM_NEXT(Drive))
            FSM_ON_EVENT("/RIGHT_OBSTACLE", FSM_NEXT(Drive))
            FSM_ON_EVENT("/FRONT_TOO_CLOSE", FSM_NEXT(DriveBackward))
        }
    }
    FSM_STATE(TurnLeft)
    {
        FSM_CALL_TASK(TurnLeft)
        FSM_TRANSITIONS
        {
            FSM_ON_EVENT("/PAUSE", FSM_NEXT(Pause))
            FSM_ON_EVENT("/TIMEOUT_TURN", FSM_NEXT(Drive))
            FSM_ON_EVENT("/LEFT_OBSTACLE", FSM_NEXT(Drive))
            FSM_ON_EVENT("/FRONT_TOO_CLOSE", FSM_NEXT(DriveBackward))
        }
    }
    FSM_STATE(TurnRight)
    {
        FSM_CALL_TASK(TurnRight)
        FSM_TRANSITIONS
        {
            FSM_ON_EVENT("/PAUSE", FSM_NEXT(Pause))
            FSM_ON_EVENT("/TIMEOUT_TURN", FSM_NEXT(Drive))
            FSM_ON_EVENT("/RIGHT_OBSTACLE", FSM_NEXT(Drive))
            FSM_ON_EVENT("/FRONT_TOO_CLOSE", FSM_NEXT(DriveBackward))
        }
    }
    FSM_STATE(Drive)
    {
        FSM_CALL_TASK(Drive)

        FSM_TRANSITIONS
        {
            FSM_ON_EVENT("/PAUSE", FSM_NEXT(Pause))
			FSM_ON_EVENT("/FRONT_AND_BACK_OBSTACLE", FSM_NEXT(Stop))
            FSM_ON_EVENT("/RIGHT_OBSTACLE", FSM_NEXT(TurnLeft))
            FSM_ON_EVENT("/LEFT_OBSTACLE", FSM_NEXT(TurnRight))
            FSM_ON_EVENT("/FRONT_OBSTACLE", FSM_NEXT(TurnRandom))
            FSM_ON_EVENT("/TIMEOUT_DRIVE", FSM_NEXT(TurnRandom))
            FSM_ON_EVENT("/FRONT_TOO_CLOSE", FSM_NEXT(DriveBackward))
        }
    }
    FSM_STATE(DriveBackward)
    {
        FSM_CALL_TASK(DriveBackward)

        FSM_TRANSITIONS
        {
        	FSM_ON_EVENT("/FRONT_AND_BACK_OBSTACLE", FSM_NEXT(Stop))
            FSM_ON_EVENT("/TIMEOUT_BACKWARD", FSM_NEXT(TurnRandom))
			FSM_ON_EVENT("/BACK_TOO_CLOSE", FSM_NEXT(TurnRandom))
            FSM_ON_EVENT("/PAUSE", FSM_NEXT(Pause))
        }
    }
    FSM_STATE(Pause)
    {

        FSM_CALL_TASK(StopRobot)

        FSM_TRANSITIONS
        {
            FSM_ON_EVENT("/RESUME", FSM_NEXT(TurnRandom))
        }
    }
    FSM_STATE(Stop)
        {

            FSM_CALL_TASK(StopRobotForSec)

            FSM_TRANSITIONS
            {
                FSM_ON_EVENT("/TIMEOUT_STOP", FSM_NEXT(TurnRandom))
            }
        }
}
FSM_END
}
