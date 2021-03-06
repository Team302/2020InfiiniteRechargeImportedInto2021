//====================================================================================================================================================
/// Copyright 2019 Lake Orion Robotics FIRST Team 302
///
/// Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"),
/// to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense,
/// and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:
///
/// The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
///
/// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
/// MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
/// DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE
/// OR OTHER DEALINGS IN THE SOFTWARE.
//====================================================================================================================================================


// C++ Includes

// FRC includes

// Team 302 includes
#include <states/hookdelivery/HookDeliveryManualState.h>
#include <states/MechanismState.h>
#include <subsys/IMechanism.h>
#include <subsys/MechanismFactory.h>
#include <controllers/MechanismTargetData.h>
#include <gamepad/TeleopControl.h>

// Third Party Includes

using namespace std;


HookDeliveryManualState::HookDeliveryManualState
(
    ControlData*                    control,
    double                          target,
    MechanismTargetData::SOLENOID   solState
) : MechanismState( MechanismFactory::GetMechanismFactory()->GetIMechanism(MechanismTypes::MECHANISM_TYPE::HOOK_DELIVERY), control, target, solState )
{
    auto ctl = TeleopControl::GetInstance();
    if (ctl != nullptr)
    {
        ctl->SetAxisProfile( TeleopControl::FUNCTION_IDENTIFIER::MANUAL_HOOK_CONTROL_UP, IDragonGamePad::AXIS_PROFILE::CUBED );
    }
}


void HookDeliveryManualState::Run()
{
    auto mech = MechanismFactory::GetMechanismFactory()->GetIMechanism(MechanismTypes::MECHANISM_TYPE::HOOK_DELIVERY);
    auto ctl = TeleopControl::GetInstance();
    if ( mech != nullptr && ctl != nullptr )
    {
        auto up = ctl->GetAxisValue( TeleopControl::FUNCTION_IDENTIFIER::MANUAL_HOOK_CONTROL_UP );
        if ( abs(up) < 0.1)
        {
            up = 0.0;
        }
        //up *= 0.5;
        //auto down = ctl->GetAxisValue( TeleopControl::FUNCTION_IDENTIFIER::MANUAL_HOOK_CONTROL_DOWN );
        //mech->SetOutput( ControlModes::CONTROL_TYPE::PERCENT_OUTPUT, (up-down) );
        mech->SetOutput( ControlModes::CONTROL_TYPE::PERCENT_OUTPUT, up );

    }
}