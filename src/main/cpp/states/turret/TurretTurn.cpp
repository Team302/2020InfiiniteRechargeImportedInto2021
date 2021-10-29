//====================================================================================================================================================
// Copyright 2020 Lake Orion Robotics FIRST Team 302
//
// Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), 
// to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, 
// and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF 
// MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, 
// DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE 
// OR OTHER DEALINGS IN THE SOFTWARE.
//====================================================================================================================================================

#include "states/turret/TurretTurn.h"
#include "subsys/MechanismFactory.h"
#include "subsys/IMechanism.h"
#include "subsys/Turret.h"
#include "controllers/ControlData.h"
#include "subsys/MechanismTypes.h"
#include "frc/smartdashboard/SmartDashboard.h"

using namespace std;
TurretTurn::TurretTurn(ControlData* controlData, double target): m_controlData(controlData),
                                                                 m_turret( MechanismFactory::GetMechanismFactory()->GetIMechanism( MechanismTypes::TURRET) ),
                                                                 m_targetAngle(0.0),
                                                                 m_targetSpeed(target),
                                                                 m_currentAngle(0.0),
                                                                 m_startAngle(0.0),
                                                                 m_isDone(false)   
{
}

void TurretTurn::Init()
{
    m_turret->SetControlConstants(m_controlData);
    m_targetAngle = 0.0;
    m_startAngle = m_turret->GetCurrentPosition();
    m_currentAngle = m_startAngle;
    m_isDone = false;
}

void TurretTurn::SetTarget(double targetAngle)
{
    m_targetAngle = targetAngle;
    m_isDone = false;
}

void TurretTurn::Run()
{
    //if (m_isDone == false)
    //{
             m_turret->SetOutput(ControlModes::PERCENT_OUTPUT, m_targetSpeed);
        //std::cout << "Target speed: " + to_string(m_targetSpeed) << endl;
        std::cout << "Current Position: " + to_string(m_turret->GetCurrentPosition()) << endl;
        //std::cout << "Target angle: " + to_string(m_targetAngle) << endl;
        //if ( abs(m_targetAngle - m_turret->GetCurrentPosition()) < 5)
        if (m_turret->GetCurrentPosition() >= (m_targetAngle + m_startAngle))
        {
            Done();
            m_isDone = true;
            //cout << "TurretTurn is done, setting turret to 0%" << endl;
        }
        m_isDone = false;
    //}
}

bool TurretTurn::AtTarget() const
{ 
    return m_isDone;
}

void TurretTurn::Done()
{
    m_targetSpeed = 0.0;
}