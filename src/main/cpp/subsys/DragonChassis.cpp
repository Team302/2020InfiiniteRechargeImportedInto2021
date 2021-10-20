
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
#include <memory>
#include <iostream>

// FRC includes
#include <frc/geometry/Pose2d.h>
#include <frc/kinematics/ChassisSpeeds.h>
#include <frc/kinematics/MecanumDriveWheelSpeeds.h>
#include <frc2/Timer.h>

#include <units/angle.h>
#include <units/length.h>
#include <units/velocity.h>

// Team 302 includes
#include <subsys/DragonChassis.h>
#include <hw/interfaces/IDragonMotorController.h>
#include <hw/DragonPigeon.h>
#include <hw/factories/PigeonFactory.h>
#include <subsys/IChassis.h>
#include <subsys/IMechanism.h>
#include <controllers/ControlModes.h>
#include <subsys/MechanismTypes.h>
#include <controllers/ControlData.h>
#include <subsys/DriveTrainSide.h>
#include <utils/Logger.h>

// Third Party Includes

using namespace std;
using namespace frc;

/// @class DragonChassis
/// @brief This is a differential chassis.  Each side of the robot will be treated as a separate mechanism

/// @brief Create the subobjects and initialize the DragonChassis subsystem
DragonChassis::DragonChassis
(
    double 						                            wheelDiameter,		
    double 					    	                        wheelBase,			
    double 						                            track,				
    std::shared_ptr<IDragonMotorController>                 leftMaster,
    std::shared_ptr<IDragonMotorController>                 rightMaster,
    std::shared_ptr<IDragonMotorController>                 leftFollower,
    std::shared_ptr<IDragonMotorController>                 rightFollower
) : m_leftSide( new DriveTrainSide( leftMaster, leftFollower, wheelDiameter ) ),
    m_rightSide( new DriveTrainSide( rightMaster, rightFollower, wheelDiameter ) ),
    m_wheelBase( wheelBase ),
    m_wheelTrack( track ),
    m_wheelDiameter( wheelDiameter ),
    m_pigeon(PigeonFactory::GetFactory()->GetPigeon()),
    m_pose(),
    m_timer(),
    m_kinematics(units::length::inch_t(track))
{
    m_timer.Reset();
    m_timer.Start();
}

/// @brief clean up memory when this object gets deleted
DragonChassis::~DragonChassis()
{
    delete m_leftSide;
    delete m_rightSide;
}


/// @brief      Run chassis 
/// @param [in] ControlModes::CONTROL_TYPE   controlType:  How are the item(s) being controlled
/// @param [in] double                                     leftValue:   Left side target (units are based on the controlType)
/// @param [in] double                                     rightValue:   Right side target (units are based on the controlType)
/// @return     void
void DragonChassis::SetOutput
(
    ControlModes::CONTROL_TYPE controlType,
    double                                   leftValue,
    double                                   rightValue     
)
{
    auto left = 0.75*leftValue;
    auto right = 0.75*rightValue;
    m_leftSide->SetOutput( controlType, left * 0.99 ); //0.99 accounts for shooter side being slightly faster than right side
    m_rightSide->SetOutput( controlType, right );
}

void DragonChassis::SetOutput
(
        frc::ChassisSpeeds  chassisSpeeds
)
{
    auto diffDriveSpeeds = m_kinematics.ToWheelSpeeds(chassisSpeeds);
    units::velocity::feet_per_second_t fpsLeft = diffDriveSpeeds.left;
    units::velocity::feet_per_second_t fpsRight = diffDriveSpeeds.right;
    double ipsLeft = fpsLeft.to<double>() * 12.0;
    double ipsRight = fpsRight.to<double>() * 12.0;
    cout << "chassis speeds: " << chassisSpeeds.vx.to<double>() << " " <<chassisSpeeds.vy.to<double>() << " " <<chassisSpeeds.omega.to<double>() << " " <<  endl;
    cout << "chassis left: " << ipsLeft << endl;
    cout << "chassis right " << ipsRight << endl;
    SetOutput(ControlModes::CONTROL_TYPE::VELOCITY_INCH, ipsLeft, ipsRight);
    /**
    units::velocity::feet_per_second_t vel {chassisSpeeds.vx};

    auto throttle = vel.to<double>() * 12.0;
    auto steer = (m_wheelTrack / 2.0) * chassisSpeeds.omega.to<double>();

    auto left = throttle - steer;
    auto right = throttle + steer;

    cout << "chassis speeds: " << left << endl;
    cout << "chassis left: " << left << endl;
    cout << "chassis right " << right << endl;

    SetOutput(ControlModes::CONTROL_TYPE::VELOCITY_INCH, left, right);
    **/
}


/// @brief  Return the current position of the center of the DragonChassis in inches.  
/// @return double  position in inches of the center of the chassis
double DragonChassis::GetCurrentPosition() const
{
    return ( m_leftSide->GetCurrentPosition() + m_rightSide->GetCurrentPosition() ) / 2.0;
}

/// @brief  Return the current position of the left side of the DragonChassis in inches.  
/// @return double  position in inches of the left side of the chassis
double DragonChassis::GetCurrentLeftPosition() const
{
    return m_leftSide->GetCurrentPosition();
}

/// @brief  Return the current position of the right side of the DragonChassis in inches.  
/// @return double  position in inches of the right side of the chassis
double DragonChassis::GetCurrentRightPosition() const
{
    return m_rightSide->GetCurrentPosition();
}

frc::Pose2d DragonChassis::GetPose() const
{
    return m_pose;
}

void DragonChassis::ResetPose
(
    const frc::Pose2d&      pose
)
{
    auto trans = pose - m_pose;
    m_pose += trans;
}

void DragonChassis::UpdatePose() 
{

	if ( m_pigeon != nullptr )
	{

        auto startX = m_pose.X();
        auto startY = m_pose.Y();
        
        units::degree_t yaw{m_pigeon->GetYaw()};
        Rotation2d rot2d {yaw};
        units::angle::radian_t rads = yaw;

        auto deltaT = m_timer.Get();
        m_timer.Reset();    

        double cosAngle = cos(rads.to<double>());
        double sinAngle = sin(rads.to<double>());

        units::velocity::feet_per_second_t speed = units::velocity::feet_per_second_t(GetCurrentSpeed()/12.0);
        units::length::meter_t currentX = startX + (speed * cosAngle) * deltaT;
        units::length::meter_t currentY = startY + (speed * sinAngle) * deltaT;

        Pose2d currentPose {currentX, currentY, rot2d};
        ResetPose(currentPose);
    }
}


bool DragonChassis::IsMoving() const
{
    return (abs(GetCurrentSpeed() > 0.0001));
}



/// @brief  Return the current speed of the center of the DragonChassis in inches per second.  
/// @return double  speed in inches per second of the center of the chassis
double DragonChassis::GetCurrentSpeed() const
{
    return ( m_leftSide->GetCurrentSpeed() + m_rightSide->GetCurrentSpeed() ) / 2.0;
}

/// @brief  Return the current speed of the left side of the DragonChassis in inches per second.  
/// @return double  speed in inches per second of the left side of the chassis
double DragonChassis::GetCurrentLeftSpeed() const
{
    return m_leftSide->GetCurrentSpeed();
}

/// @brief  Return the current speed of the right side of the DragonChassis in inches per second.  
/// @return double  speed in inches per second of the right side of the chassis
double DragonChassis::GetCurrentRightSpeed() const
{
    return m_rightSide->GetCurrentSpeed();
}




/// @brief  Set the control constants (e.g. PIDF values).
/// @param [in] ControlData*   pid - the control constants
/// @return void
void DragonChassis::SetControlConstants
(
    ControlData*                                 pid               
)
{
    m_leftSide->SetControlConstants( pid );
    m_rightSide->SetControlConstants( pid );
}

double DragonChassis::GetWheelDiameter() const
{ 
    return m_wheelDiameter; 
}





