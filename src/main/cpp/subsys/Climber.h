#pragma once

//C++ Includes
#include <memory>

//Team 302 includes
#include <hw/interfaces/IDragonMotorController.h>
#include <subsys/IMechanism.h>
#include <controllers/ControlModes.h>
#include <controllers/ControlData.h>
#include <subsys/MechanismTypes.h>

///@class Climber
///@brief This is the climber sub-system
class Climber : public IMechanism
{
    public:

        ///@brief   Create and initialize the Climber sub-mechanism
        ///@param [in] std::shared_ptr<IDragonMotorController>   masterMotor - master motor for the Climber
        ///@param [in] double   **
        Climber
        (
            std::shared_ptr<IDragonMotorController>          masterMotor
        );
        Climber() = delete;
        ///@brief Clean up memory when this object gets deleted
        virtual ~Climber() = default;

        ///@brief Indicates that this is Climber
        ///@return IMechanism::MECHANISM_TYPE::CLIMBER
        MechanismTypes::MECHANISM_TYPE GetType() const override;

        ///@brief Run the Climber as defined
        ///@param [in] ControlModes::CONTROL_TYPE   controlType
        ///@param [in] double   value
        ///@return void
        void SetOutput
        (
            ControlModes::CONTROL_TYPE controlType,
            double                                  value
        ) override;

        /// @brief      Activate/deactivate pneumatic solenoid
        /// @param [in] bool - true == extend, false == retract
        /// @return     void 
        void ActivateSolenoid
        (
            bool     activate
        ) override;

        /// @brief      Check if the pneumatic solenoid is activated
        /// @return     bool - true == extended, false == retract
        bool IsSolenoidActivated() override;



        ///@brief Return the current position of the climber in inches (positive is forward, negative is backward)
        ///@return  double position in inches
        double GetCurrentPosition() const override;

        ///@brief Return the current position of the climber in inches (positive is forward, negative is backward)
        ///@return double speed in inches
        double GetCurrentSpeed() const override;


        ///@brief Set the control constants (e.g. PIDF values)
        ///@param [in] ControlData*    pid - the control constants
        ///@return void
        void SetControlConstants
        (
            ControlData*    pid
        ) override;

    private:
        std::shared_ptr<IDragonMotorController>             m_motorMaster;
        double                                              m_target;
};
