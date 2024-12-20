#include <FSM/FSM.h>
#include <iostream>

FSM::FSM(ControlFSMData *data)
    : _data(data)
{

    _stateList.invalid = nullptr;
    _stateList.passive = new FSMState_Passive(_data);
    _stateList.pdstand = new FSMState_PDStand(_data);
    _stateList.qpstand = new FSMState_QPStand(_data);
    _stateList.walking = new FSMState_Walking(_data);
    _stateList.manipulation = new FSMState_Manipulation(_data);
    // add other FSM states later

    initialize();
}

FSM::~FSM()
{
    _stateList.deletePtr();
}

void FSM::initialize()
{
    count = 0;
    _currentState = _stateList.passive;
    _currentState->enter();
    _nextState = _currentState;
    _mode = FSMMode::NORMAL;
}

void FSM::run()
{
    if(count ==1)
    {
        for(int i = 0; i < 4; i++)
        {
            _data->_lowState->Init_FootForce[i] = _data->_lowState->FootForce[i];
        }
    }
    
    if (!checkSafty())
    {
        _data->_interface->setPassive();
    }
    if (_mode == FSMMode::NORMAL)
    {
        _currentState->run();
        _nextStateName = _currentState->checkTransition();
        if (_nextStateName != _currentState->_stateName)
        {
            _mode = FSMMode::CHANGE;
            _nextState = getNextState(_nextStateName);
        }
    }
    else if (_mode == FSMMode::CHANGE)
    {
        // std::cout << "change state" << std::endl;
        _currentState->exit();
        _currentState = _nextState;
        _currentState->enter();
        _mode = FSMMode::NORMAL;
        _currentState->run();
    }

    for (int i = 0; i < 3; i++)
    {
        _data->_lowState->position[i] = _data->_stateEstimator->getResult().position(i);
    }
    count++;
}

FSMState *FSM::getNextState(FSMStateName stateName)
{
    switch (stateName)
    {
    case FSMStateName::INVALID:
        return _stateList.invalid;
        break;
    case FSMStateName::PASSIVE:
        return _stateList.passive;
        break;
    case FSMStateName::PDSTAND:
        return _stateList.pdstand;
        break;
    case FSMStateName::QPSTAND:
        return _stateList.qpstand;
        break;
    case FSMStateName::WALKING:
        return _stateList.walking;
        break;
    case FSMStateName::MANIPULATION:
        return _stateList.manipulation;
    default:
        return _stateList.invalid;
        break;
    }
}

bool FSM::checkSafty()
{
    if (_data->_stateEstimator->getResult().rBody(2, 2) < 0.5)
    {
        return false;
    }
    else
    {
        return true;
    }
}