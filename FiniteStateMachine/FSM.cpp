#include "FSM.h"
#include <iostream>

timur::StateMachine::StateMachine()
{
	_fsmTable = {
	{
	{ &StateMachine::toState2        , &StateMachine::toUndefinedState, &StateMachine::toUndefinedState },
	{ &StateMachine::toUndefinedState, &StateMachine::toUndefinedState, &StateMachine::toState3         },
	{ &StateMachine::toUndefinedState, &StateMachine::toState4        , &StateMachine::toUndefinedState },
	{ &StateMachine::toUndefinedState, &StateMachine::toState4        , &StateMachine::toState5         },
	{ &StateMachine::toUndefinedState, &StateMachine::toUndefinedState, &StateMachine::toUndefinedState },
	{ &StateMachine::toUndefinedState, &StateMachine::toUndefinedState, &StateMachine::toUndefinedState }
	}
	};
	_currentState = 0;
}

ptrdiff_t timur::StateMachine::interpreter()
{
	switch (preSignal)
	{
	case '+':
		return 0;
	case 'a':
		return 1;
	case 'b':
		return 2;
	default:
		return -1;
	}
}

void timur::StateMachine::startWorking()
{
	const ptrdiff_t signal = interpreter();
	if (signal != -1)
	{
		(this->*_fsmTable[_currentState][signal])();
	}
	else
	{
		return;
	}
}


void timur::StateMachine::toState1()
{
	_currentState = 0;
}

void timur::StateMachine::toState2()
{
	_currentState = 1;
}

void timur::StateMachine::toState3()
{
	_currentState = 2;
}

void timur::StateMachine::toState4()
{
	_currentState = 3;
}

void timur::StateMachine::toState5()
{
	_currentState = 4;
}

void timur::StateMachine::toUndefinedState()
{
	_currentState = -1;
}
