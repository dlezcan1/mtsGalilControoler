#include "mtsGalilController/mtsGalilController.h"
#include "mtsGalilController/GalilControllerInterface.h"


CMN_IMPLEMENT_SERVICES_DERIVED(mtsGalilController, mtsTaskPeriodic);

#define MTS_ADD_COMMAND_CHECK( AddCommandFunc, interface, function, object, name) \
    if (!interface->AddCommandFunc(function, object, name)) \
    { \
       CMN_LOG_CLASS_INIT_ERROR << "Failed to add mtsGalilController::" << name << " to \""  \
                                 << interface->GetFullName() \
                                 << "\"" << std::endl; \
    }
#define MTS_ADD_COMMAND_READ_CHECK(...) MTS_ADD_COMMAND_CHECK(AddCommandRead, ##__VA_ARGS__)
#define MTS_ADD_COMMAND_VOID_CHECK(...) MTS_ADD_COMMAND_CHECK(AddCommandVoid, ##__VA_ARGS__)
#define MTS_ADD_COMMAND_WRITE_CHECK(...) MTS_ADD_COMMAND_CHECK(AddCommandWrite, ##__VA_ARGS__)

mtsGalilController::mtsGalilController(const std::string & componentName, double period_secs) : mtsTaskPeriodic(componentName, period_secs), m_StateTable(50, "GalilState")
{
    SetupInterfaces();
}

mtsGalilController::mtsGalilController(const mtsTaskPeriodicConstructorArg & arg) : mtsTaskPeriodic(arg), m_StateTable(50, "GalilState")
{
    SetupInterfaces();
}

void mtsGalilController::Configure(const std::string & fileName)
{
    m_galilController = GalilControllerInterfaceFactory::GetControllerInterface<DemoGalilControllerInterface>(fileName);
    
}

void mtsGalilController::SetupInterfaces()
{
    // Configure the Galil Controller State Table
    m_StateTable.AddData(m_ActuatorState, "ActuatorState");

    // Add the interface
    mtsInterfaceProvided* intfProvided = this->AddInterfaceProvided("GalilController");
    if (!intfProvided)
    {
        CMN_LOG_CLASS_INIT_ERROR << "Error adding \"GalilController\" provided interface \"" 
                                 << this->GetName()
                                 << "\"!" << std::endl;
        return;
    }

    // Add command to control galil controller
    MTS_ADD_COMMAND_WRITE_CHECK(intfProvided, &mtsGalilController::SendCommand, this, "SendCommand");
    if (!intfProvided->AddCommandWriteReturn(&mtsGalilController::SetTimeout, this, "SetTimeout"))
    {
        CMN_LOG_CLASS_INIT_ERROR << "Failed to add mtsGalilController::SetTimeout to \"" 
                                 << intfProvided->GetFullName()
                                 << "\"" << std::endl;
    }
    if (!intfProvided->AddCommandReadState(m_StateTable, m_ActuatorState, "GetActuatorState"))
    {
        CMN_LOG_CLASS_INIT_ERROR << "Failed to add mtsGalilController::GetActuatorState to \"" 
                                 << intfProvided->GetFullName()
                                 << "\"" << std::endl;
    }

    // Galil commands
    MTS_ADD_COMMAND_VOID_CHECK(intfProvided,  &GalilControllerInterface::AbortProgram,  m_galilController, "AbortProgram");
    MTS_ADD_COMMAND_VOID_CHECK(intfProvided,  &GalilControllerInterface::AbortMotion,   m_galilController, "AbortMotion");
    MTS_ADD_COMMAND_VOID_CHECK(intfProvided,  &GalilControllerInterface::Reset,         m_galilController, "Reset");
    MTS_ADD_COMMAND_WRITE_CHECK(intfProvided, &GalilControllerInterface::Home,          m_galilController, "Home");
    MTS_ADD_COMMAND_WRITE_CHECK(intfProvided, &GalilControllerInterface::UnHome,        m_galilController, "UnHome");

    MTS_ADD_COMMAND_WRITE_CHECK(intfProvided, &GalilControllerInterface::SetAbsolutePosition, m_galilController, "SetAbsolutePosition");
    MTS_ADD_COMMAND_WRITE_CHECK(intfProvided, &GalilControllerInterface::SetAcceleration,     m_galilController, "SetAccleration");
    MTS_ADD_COMMAND_WRITE_CHECK(intfProvided, &GalilControllerInterface::SetDeceleration,     m_galilController, "SetDecleration");
    MTS_ADD_COMMAND_WRITE_CHECK(intfProvided, &GalilControllerInterface::SetPositionMove,     m_galilController, "SetPositionMove");
    MTS_ADD_COMMAND_WRITE_CHECK(intfProvided, &GalilControllerInterface::SetVelocity,         m_galilController, "SetVelocity");
    MTS_ADD_COMMAND_WRITE_CHECK(intfProvided, &GalilControllerInterface::SetVelocityMove,     m_galilController, "SetVelocityMove");
    
    MTS_ADD_COMMAND_WRITE_CHECK(intfProvided, &GalilControllerInterface::EnableMotorPower,     m_galilController, "EnableMotorPower");
    MTS_ADD_COMMAND_WRITE_CHECK(intfProvided, &GalilControllerInterface::DisableMotorPower,    m_galilController, "DisableMotorPower");
    MTS_ADD_COMMAND_VOID_CHECK(intfProvided,  &GalilControllerInterface::EnableAllMotorPower,  m_galilController, "EnableAllMotorPower");
    MTS_ADD_COMMAND_VOID_CHECK(intfProvided,  &GalilControllerInterface::DisableAllMotorPower, m_galilController, "DisableAllMotorPower");
    
    MTS_ADD_COMMAND_WRITE_CHECK(intfProvided, &GalilControllerInterface::StopMotion,    m_galilController, "StopMotion");
    MTS_ADD_COMMAND_VOID_CHECK(intfProvided,  &GalilControllerInterface::StopMotionAll, m_galilController, "StopMotionAll");
    
    MTS_ADD_COMMAND_WRITE_CHECK(intfProvided, &mtsGalilController::StopMovement, this, "StopMovement");
    MTS_ADD_COMMAND_WRITE_CHECK(intfProvided, &mtsGalilController::WaitMotion,   this, "WaitMotion");

    MTS_ADD_COMMAND_READ_CHECK(intfProvided, &GalilControllerInterface::GetAnalogInputs, m_galilController, "GetAnalogInputs");

    delete intfProvided;
}


void mtsGalilController::Startup(){
    // TODO
}


void mtsGalilController::Run(){
    this->ProcessQueuedCommands();
    this->ProcessQueuedEvents();
    
    if (!m_galilController)
        return;


    // Get the Galil State
    m_StateTable.Start();
    m_galilController->GetActuatorState(m_ActuatorState);
    m_StateTable.Advance();

}


void mtsGalilController::Cleanup(){
    // TODO
    try {
        m_galilController->Close();
        CMN_LOG_CLASS_RUN_VERBOSE << "Closed Galil Controller." << std::endl;
    }
    catch (GalilControllerInterface::ExcpSystemError e)
    {
        throw e;    
    }
}

void mtsGalilController::SetTimeout(const mtsDouble& timeout, mtsBool& success){ 
    success.Data = false;
    if (timeout > 0) 
    {
        success.Data = true;
        m_timeout    = timeout;
    }
}