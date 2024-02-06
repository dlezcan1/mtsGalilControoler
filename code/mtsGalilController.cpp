#include "mtsGalilController/mtsGalilController.h"

#include "cisstCommon/cmnDataFormat.h"

CMN_IMPLEMENT_SERVICES(mtsGalilController);

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

mtsGalilController::mtsGalilController(const std::string & componentName, double period_secs) : mtsTaskPeriodic(componentName, period_secs), m_StateTable(10000, "GalilState")
{
    SetupInterfaces();
}

mtsGalilController::mtsGalilController(const mtsTaskPeriodicConstructorArg & arg) : mtsTaskPeriodic(arg), m_StateTable(10000, "GalilState")
{
    SetupInterfaces();
}

mtsGalilController::~mtsGalilController()
{
}

void mtsGalilController::Configure(const std::string& fileName)
{
    std::string deviceName = "demo";
    auto deviceInterface   = GalilControllerInterfaceFactory::Interface::DEMO;

    std::string dmcStartupFile;
    
    try
    {
        std::ifstream jsonStream;
        Json::Value   jsonConfig;
        Json::Reader  jsonReader;

        jsonStream.open(fileName.c_str());

        if (!jsonReader.parse(jsonStream, jsonConfig)) {
            CMN_LOG_CLASS_INIT_ERROR << "Configure " << this->GetName()
                                     << ": failed to parse galil controller configuration file \""
                                     << fileName << "\"\n"
                                     << jsonReader.getFormattedErrorMessages();
            return;
        }

        CMN_LOG_CLASS_INIT_VERBOSE << "Configure: " << this->GetName()
                                   << " using file \"" << fileName << "\"" << std::endl
                                   << "----> content of galil controller configuration file: " << std::endl
                                   << jsonConfig << std::endl
                                   << "<----" << std::endl;

    if (jsonConfig.isMember("IP_Address"))
        deviceName = jsonConfig["IP_Address"].asString();


    if (jsonConfig.isMember("Controller_Interface"))
    {
        std::string controllerInterface = jsonConfig["Controller_Interface"].asString();
        std::transform(
            controllerInterface.begin(),
            controllerInterface.end(),
            controllerInterface.begin(),
            [] (unsigned char c) {return std::tolower(c);}

        );

        deviceInterface = GalilControllerInterfaceFactory::CONTROLLER; // if present value, assume it is a real controller
        if ( controllerInterface == "demo")
            deviceInterface = GalilControllerInterfaceFactory::Interface::DEMO;
    }

    
    if (jsonConfig.isMember("DMC_Startup_Program"))
    {
        std::cout << "Before dmcStartupFile addition\n";
        dmcStartupFile = jsonConfig["DMC_Startup_Program"].asString();
        std::cout << "After dmcStartupFile addition: " << dmcStartupFile << std::endl;
    }

    }
    catch (...)
    {
        CMN_LOG_CLASS_INIT_ERROR << "Configure " << this->GetName() 
                                 << ": make sure the file \""
                                 << fileName << "\" is in JSON format"
                                 << std::endl;
    }


    // create the controller interface
    m_galilController = GalilControllerInterfaceFactory::GetControllerInterface(
        deviceName,
        deviceInterface
    );


    // upload a dmc program file if available
    if (dmcStartupFile.length() > 0)
    {
        std::ifstream testFile(dmcStartupFile);
        if (testFile.good())
        {
            std::cout << "After testFile is good\n";
            m_galilController->ProgramUploadFile(dmcStartupFile);
        }
        else
        {
            CMN_LOG_CLASS_INIT_ERROR << "Configure " << this->GetName() << ": "
                                     << "No dmc program file exists: \""
                                     << dmcStartupFile
                                     << "\""
                                     << std::endl;;
        }
        testFile.close();
    }
    
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
    MTS_ADD_COMMAND_VOID_CHECK(intfProvided,  &GalilControllerInterface::AbortProgram,  m_galilController.get(), "AbortProgram");
    MTS_ADD_COMMAND_VOID_CHECK(intfProvided,  &GalilControllerInterface::AbortMotion,   m_galilController.get(), "AbortMotion");
    MTS_ADD_COMMAND_VOID_CHECK(intfProvided,  &GalilControllerInterface::Reset,         m_galilController.get(), "Reset");
    MTS_ADD_COMMAND_WRITE_CHECK(intfProvided, &GalilControllerInterface::Home,          m_galilController.get(), "Home");
    MTS_ADD_COMMAND_WRITE_CHECK(intfProvided, &GalilControllerInterface::UnHome,        m_galilController.get(), "UnHome");

    MTS_ADD_COMMAND_WRITE_CHECK(intfProvided, &GalilControllerInterface::SetAcceleration,     m_galilController.get(), "SetAccleration");
    MTS_ADD_COMMAND_WRITE_CHECK(intfProvided, &GalilControllerInterface::SetDeceleration,     m_galilController.get(), "SetDecleration");
    MTS_ADD_COMMAND_WRITE_CHECK(intfProvided, &GalilControllerInterface::SetSpeed,            m_galilController.get(), "SetSpeed");
    
    MTS_ADD_COMMAND_WRITE_CHECK(intfProvided, &GalilControllerInterface::SetAbsolutePosition, m_galilController.get(), "SetAbsolutePosition");
    MTS_ADD_COMMAND_WRITE_CHECK(intfProvided, &GalilControllerInterface::SetPositionMove,     m_galilController.get(), "SetPositionMove");
    MTS_ADD_COMMAND_WRITE_CHECK(intfProvided, &GalilControllerInterface::SetVelocityMove,     m_galilController.get(), "SetVelocityMove");
    
    MTS_ADD_COMMAND_WRITE_CHECK(intfProvided, &GalilControllerInterface::EnableMotorPower,     m_galilController.get(), "EnableMotorPower");
    MTS_ADD_COMMAND_WRITE_CHECK(intfProvided, &GalilControllerInterface::DisableMotorPower,    m_galilController.get(), "DisableMotorPower");
    MTS_ADD_COMMAND_VOID_CHECK(intfProvided,  &GalilControllerInterface::EnableAllMotorPower,  m_galilController.get(), "EnableAllMotorPower");
    MTS_ADD_COMMAND_VOID_CHECK(intfProvided,  &GalilControllerInterface::DisableAllMotorPower, m_galilController.get(), "DisableAllMotorPower");
    
    MTS_ADD_COMMAND_WRITE_CHECK(intfProvided, &GalilControllerInterface::StopMotion,    m_galilController.get(), "StopMotion");
    MTS_ADD_COMMAND_VOID_CHECK(intfProvided,  &GalilControllerInterface::StopMotionAll, m_galilController.get(), "StopMotionAll");
    
    MTS_ADD_COMMAND_WRITE_CHECK(intfProvided, &mtsGalilController::StopMovement, this, "StopMovement");
    MTS_ADD_COMMAND_WRITE_CHECK(intfProvided, &mtsGalilController::WaitMotion,   this, "WaitMotion");

    MTS_ADD_COMMAND_READ_CHECK(intfProvided, &GalilControllerInterface::GetAnalogInputs, m_galilController.get(), "GetAnalogInputs");

    delete intfProvided;
}


void mtsGalilController::Startup(){
    // TODO
}


void mtsGalilController::Run(){
    this->ProcessQueuedEvents();
    // this->ProcessQueuedCommands();

    
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
        m_galilController.reset();
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