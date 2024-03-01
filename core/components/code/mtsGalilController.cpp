/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  (C) Copyright 2024 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/

#include <cisstCommon/cmnDataFormat.h>
#include <cisstCommon/cmnUnits.h>
#include <cisstCommon/cmnPath.h>
#include <cisstOSAbstraction/osaSleep.h>
#include <cisstOSAbstraction/osaStopwatch.h>
#include <cisstMultiTask/mtsInterfaceProvided.h>

#include <sawGalilController/mtsGalilController.h>

CMN_IMPLEMENT_SERVICES_DERIVED_ONEARG(mtsGalilController, mtsTaskPeriodic, mtsTaskPeriodicConstructorArg)

#define MTS_ADD_COMMAND_CHECK( AddCommandFunc, interface, function, object, name) \
    if (!interface->AddCommandFunc(function, object, name)) \
    { \
       CMN_LOG_CLASS_INIT_ERROR << "Failed to add mtsGalilController::" << name << " to \""  \
                                 << interface->GetFullName() \
                                 << "\"" << std::endl; \
    }

#if 0
// Some compilers do not support following
#define MTS_ADD_COMMAND_READ_CHECK(...) MTS_ADD_COMMAND_CHECK(AddCommandRead, ##__VA_ARGS__)
#define MTS_ADD_COMMAND_VOID_CHECK(...) MTS_ADD_COMMAND_CHECK(AddCommandVoid, ##__VA_ARGS__)
#define MTS_ADD_COMMAND_WRITE_CHECK(...) MTS_ADD_COMMAND_CHECK(AddCommandWrite, ##__VA_ARGS__)

#else

#define MTS_ADD_COMMAND_READ_CHECK(interface, function, object, name)   \
        MTS_ADD_COMMAND_CHECK(AddCommandRead, interface, function, object, name)
#define MTS_ADD_COMMAND_VOID_CHECK(interface, function, object, name) \
        MTS_ADD_COMMAND_CHECK(AddCommandVoid, interface, function, object, name)
#define MTS_ADD_COMMAND_WRITE_CHECK(interface, function, object, name) \
        MTS_ADD_COMMAND_CHECK(AddCommandWrite, interface, function, object, name)
#endif

mtsGalilController::mtsGalilController(const std::string & componentName, double period_secs) :
                    mtsTaskPeriodic(componentName, period_secs), m_StateTable(10000, "GalilState")
{
}

mtsGalilController::mtsGalilController(const mtsTaskPeriodicConstructorArg & arg) :
                    mtsTaskPeriodic(arg), m_StateTable(10000, "GalilState")
{
}

mtsGalilController::~mtsGalilController()
{
    Close();
}

void mtsGalilController::Configure(const std::string& fileName)
{
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

        m_DeviceName = jsonConfig["IP_Address"].asString();
        // Configure the axes
        Json::Value axesConfig = jsonConfig["Axes"];
        m_EncoderCountsPerUnit.SetSize(axesConfig.size());
        m_AxisToGalilChannelMappings.SetSize(axesConfig.size());
        m_GalilToAxisChannelMappings.SetSize(axesConfig.size());
        for (auto it = axesConfig.begin(); it != axesConfig.end(); it++)
        {
            unsigned int axisIndex = it.index();

            unsigned int galilIndex  = it->get("Galil_Channel", axisIndex).asUInt();
            double encoderConversion = it->get("Encoder_Conversion", 1.0).asDouble();

            m_EncoderCountsPerUnit[axisIndex]               = encoderConversion;
            m_AxisToGalilChannelMappings[axisIndex]         = galilIndex;
            m_GalilToAxisChannelMappings.Data()[galilIndex] = axisIndex;
            m_GalilToAxisChannelMappings.Mask()[galilIndex] = true;
        }

        // Galil Startup Program
        if (jsonConfig.isMember("DMC_Startup_Program"))
        {
            std::cout << "Before dmcStartupFile addition\n";
            mDmcFile = jsonConfig["DMC_Startup_Program"].asString();
            std::cout << "After dmcStartupFile addition: " << mDmcFile << std::endl;
        }
    }
    catch (...)
    {
        CMN_LOG_CLASS_INIT_ERROR << "Configure " << this->GetName() 
                                 << ": make sure the file \""
                                 << fileName << "\" is in JSON format"
                                 << std::endl;
    }

    // Galil Controller Command config

    // set the size of the vectors:
    m_ActuatorState.SetSize(GetNumberActuators());
    // prmActuatorState does not initialize its members
    // after calling SetSize
    m_ActuatorState.Position().SetAll(0.0);
    m_ActuatorState.Velocity().SetAll(0.0);

    m_IsHomed.SetSize(GetNumberActuators());
    m_IsHomed.SetAll(false); // this state is not available on the controller

    m_AnalogInput.SetSize(GetNumberActuators());
    m_AnalogInput.Zeros();

    // PK: size set above
    // m_EncoderCountsPerUnit.SetSize(GetNumberActuators());
    // m_EncoderCountsPerUnit.SetAll(1.0); // default to use encoder counts

    // Disha- encoder
    unsigned int Encoder_pins[] = {9, 12, 15, 11, 14, 10, 13, 8, 1, 2};
    const size_t NumberEncoderPins = sizeof(Encoder_pins) / sizeof(Encoder_pins[0]);
    m_NumberEncoderPins = NumberEncoderPins;
    m_DigitalInput.SetSize(NumberEncoderPins);
    m_DigitalInput.Zeros();

    m_Galil = nullptr;
    // max number of actuator is 8 here
    char analogStr[7];
    // Disha-encoder
    char digitalStr[NumberEncoderPins];
    memset(digitalStr, 0, sizeof(char) * NumberEncoderPins);
    memset(analogStr, 0, sizeof(char) * 7);

    m_SoftRevLimitHit.SetSize(GetNumberActuators());
    m_SoftFwdLimitHit.SetSize(GetNumberActuators());

    const char *letter = "ABCDEFGHIJKLMNOPQRSTUVWYZ";
    // create commands for all desired variable commands
    for (unsigned int idxAxis = 0; idxAxis < GetNumberActuators(); idxAxis++)
    {
        unsigned int idxGalil = RemapAxisIndex(idxAxis);
        TP.push_back(std::string("_TP").append(1, letter[idxGalil]));
        TV.push_back(std::string("_TV").append(1, letter[idxGalil])); // vel (filtered) counts/sec
        BG.push_back(std::string("_BG").append(1, letter[idxGalil])); // axis moving
        HM.push_back(std::string("_HM").append(1, letter[idxGalil])); // home
        LF.push_back(std::string("_LF").append(1, letter[idxGalil])); // for limit
        LR.push_back(std::string("_LR").append(1, letter[idxGalil])); // rev limit
        MO.push_back(std::string("_MO").append(1, letter[idxGalil])); // Motor off
        SC.push_back(std::string("_SC").append(1, letter[idxGalil])); // stop code
        ZA.push_back(std::string("_ZA").append(1, letter[idxGalil])); // user variable   can be used to store state of home

        // analog inputs, generate appropriate strings  -- begins with 1
        // std::string  analog("@AN[1]");
        // itoa(i+1,digit,10);
        // analog.replace(4,1,digit);
        sprintf(analogStr, "@AN[%d]", idxGalil + 2);
        AN.push_back(std::string(analogStr));
        // std::cout<<analogStr<<std::endl;
    }

    // Disha-encoder
    for (unsigned int i = 0; i < m_NumberEncoderPins; i++)
    {
        sprintf(digitalStr, "@IN[%02d]", Encoder_pins[i]);
        DI.push_back(std::string(digitalStr));
    }

    // Call SetupInterfaces after Configure because we need to know the correct sizes of
    // the dynamic vectors, which are based on the number of configured axes.
    SetupInterfaces();
}

void mtsGalilController::SetupInterfaces()
{
    // Configure the Galil Controller State Table
    m_StateTable.AddData(m_ActuatorState, "ActuatorState");

    // Add the interface
    mtsInterfaceProvided* intfProvided = this->AddInterfaceProvided("control");
    if (!intfProvided)
    {
        CMN_LOG_CLASS_INIT_ERROR << "Error adding \"GalilController\" provided interface \"" 
                                 << this->GetName()
                                 << "\"!" << std::endl;
        return;
    }

    // Add command to control galil controller
    MTS_ADD_COMMAND_WRITE_CHECK(intfProvided, &mtsGalilController::SendCommand, this, "SendCommand");
    if (!intfProvided->AddCommandWriteReturn(&mtsGalilController::SendCommandRet, this, "SendCommandRet"))
    {
        CMN_LOG_CLASS_INIT_ERROR << "Failed to add mtsGalilController::SendCommandRet to \""
                                 << intfProvided->GetFullName()
                                 << "\"" << std::endl;
    }
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
    MTS_ADD_COMMAND_VOID_CHECK(intfProvided,  &mtsGalilController::AbortProgram,  this, "AbortProgram");
    MTS_ADD_COMMAND_VOID_CHECK(intfProvided,  &mtsGalilController::AbortMotion,   this, "AbortMotion");
    MTS_ADD_COMMAND_VOID_CHECK(intfProvided,  &mtsGalilController::Reset,         this, "Reset");
    MTS_ADD_COMMAND_WRITE_CHECK(intfProvided, &mtsGalilController::Home,          this, "Home");
    MTS_ADD_COMMAND_WRITE_CHECK(intfProvided, &mtsGalilController::UnHome,        this, "UnHome");

    MTS_ADD_COMMAND_WRITE_CHECK(intfProvided, &mtsGalilController::SetAcceleration,     this, "SetAccleration");
    MTS_ADD_COMMAND_WRITE_CHECK(intfProvided, &mtsGalilController::SetDeceleration,     this, "SetDecleration");
    MTS_ADD_COMMAND_WRITE_CHECK(intfProvided, &mtsGalilController::SetSpeed,            this, "SetSpeed");

    MTS_ADD_COMMAND_WRITE_CHECK(intfProvided, &mtsGalilController::SetAbsolutePosition, this, "SetAbsolutePosition");
    MTS_ADD_COMMAND_WRITE_CHECK(intfProvided, &mtsGalilController::SetPositionMove,     this, "SetPositionMove");
    MTS_ADD_COMMAND_WRITE_CHECK(intfProvided, &mtsGalilController::SetVelocityMove,     this, "SetVelocityMove");

    MTS_ADD_COMMAND_WRITE_CHECK(intfProvided, &mtsGalilController::EnableMotorPower,     this, "EnableMotorPower");
    MTS_ADD_COMMAND_WRITE_CHECK(intfProvided, &mtsGalilController::DisableMotorPower,    this, "DisableMotorPower");
    MTS_ADD_COMMAND_VOID_CHECK(intfProvided,  &mtsGalilController::EnableAllMotorPower,  this, "EnableAllMotorPower");
    MTS_ADD_COMMAND_VOID_CHECK(intfProvided,  &mtsGalilController::DisableAllMotorPower, this, "DisableAllMotorPower");

    MTS_ADD_COMMAND_WRITE_CHECK(intfProvided, &mtsGalilController::StopMotion,    this, "StopMotion");
    MTS_ADD_COMMAND_VOID_CHECK(intfProvided,  &mtsGalilController::StopMotionAll, this, "StopMotionAll");

    MTS_ADD_COMMAND_WRITE_CHECK(intfProvided, &mtsGalilController::StopMovement, this, "StopMovement");
    MTS_ADD_COMMAND_WRITE_CHECK(intfProvided, &mtsGalilController::WaitMotion,   this, "WaitMotion");

    MTS_ADD_COMMAND_READ_CHECK(intfProvided, &mtsGalilController::GetAnalogInputs, this, "GetAnalogInputs");
    MTS_ADD_COMMAND_READ_CHECK(intfProvided, &mtsGalilController::GetConnected, this, "GetConnected");

    // PK: not correct to delete following
    //delete intfProvided;
}


void mtsGalilController::Startup(){
    ConnectToGalilController(m_DeviceName);

    // upload a DMC program file if available
    if (mDmcFile.length() > 0)
    {
        if (cmnPath::Exists(mDmcFile))
        {
            std::cout << "After testFile is good\n";
            ProgramUploadFile(mDmcFile);
        }
        else
        {
            CMN_LOG_CLASS_INIT_ERROR << "Configure " << this->GetName() << ": "
                                     << "No dmc program file exists: \""
                                     << mDmcFile
                                     << "\""
                                     << std::endl;;
        }
    }

    m_StateTable.Start();
}


void mtsGalilController::Run()
{
    if (m_Galil) {
        // Get the Galil State
        GetActuatorState(m_ActuatorState);
        m_StateTable.Advance();
    }

    // Call any connected components
    RunEvent();

    ProcessQueuedCommands();
}

void mtsGalilController::Cleanup(){
    // TODO
    try {
        Close();
        CMN_LOG_CLASS_RUN_VERBOSE << "Closed Galil Controller." << std::endl;
    }
    catch (const std::runtime_error &e)
    {
        throw e;
    }
}

void mtsGalilController::SetTimeout(const double& timeout, bool& success){
    success = false;
    if (timeout > 0)
    {
        success = true;
        m_timeout = timeout;
    }
}

/* ================== Galil Controller Interface ============================== */

void mtsGalilController::AbortProgram()
{
    this->SendCommandString("AB");
}

void mtsGalilController::AbortMotion()
{
    this->SendCommandString("AB 1");
}

void mtsGalilController::Close()
{

    if (m_Galil)
    {
        CMN_LOG_CLASS_RUN_VERBOSE << "Closing Galil controller" << std::endl;
        SendCommandString("ST");
        DisableAllMotorPower();
        GClose(m_Galil);
        // PK: I do not think it is necessary or correct to delete m_Galil
        //     you could set it to nullptr if you want
        // delete m_Galil;
    }
}

//////////////////////////////////////////////////////////////////////
// Public member functions
//////////////////////////////////////////////////////////////////////

// Init:  initialize robot (must be called)
// This function establishes communication with the Galil controller,
// configures it, downloads the application program and then starts execution.
void mtsGalilController::ConnectToGalilController(const std::string &deviceName)
{
    try
    {
        std::cout << "Hello Galil" << std::endl;

        // -d for direct connection (not using gcaps)
        std::string GalilString = std::string("-d ") + deviceName;
        GReturn ret = GOpen(GalilString.c_str(), &m_Galil);
        if (ret != G_NO_ERROR) {
            CMN_LOG_CLASS_INIT_ERROR << "Galil GOpen: error opening " << deviceName
                                     << ": " << ret << std::endl;
            return;
        }
        std::cout << "Galil connected!" << std::endl;

        // Stop motors and halt Galil Controller program execution
        // PK: Following does not seem to work
        // this->StopMotionAll();

        m_ServoLoopTime = this->SendCommandDouble("TM?");
        // Loop time is used to correct for speed scaling due to TM250
        m_TMVelocityMultiplier = 1000.0 / m_ServoLoopTime;

        CMN_LOG_CLASS_INIT_VERBOSE << "Galil Servo Loop Time is " << m_ServoLoopTime / 1000.0 << " ms" << std::endl;
    }
    catch (const std::string &e) // error
    {
        CMN_LOG_CLASS_INIT_ERROR << e << std::endl;
        if (std::string::npos != e.find("COMMAND ERROR"))
            CMN_LOG_CLASS_INIT_ERROR << "a command error occurred" << std::endl; // special processing for command errors
    }
    catch (const std::runtime_error &e) // error
    {
        CMN_LOG_CLASS_RUN_ERROR << e.what() << std::endl;
    }
}

// EnableMotorPower:  turns on the motor servos
void mtsGalilController::EnableAllMotorPower()
{
    CMN_LOG_CLASS_RUN_VERBOSE << "EnableMotorPower " << std::endl;
    // VerifyStatus("EnableMotorPower", ESTOP_MASK);
    try
    {
        this->SendCommandString("SH");
    }
    catch (const std::string &e)
    {
        CMN_LOG_CLASS_RUN_ERROR << "Enable motor failed  \"" << e << "\"" << std::endl;
        cmnThrow(std::runtime_error("Enable motor failed "));
    }
}

// DisableMotorPower:  turns off the motor servos
// MO not valid while running, need to issue ST first!!!! annoying.
void mtsGalilController::DisableAllMotorPower()
{
    CMN_LOG_CLASS_RUN_VERBOSE << "DisableMotorPower " << std::endl;
    try
    {
        this->StopMotionAll();
        SendCommandString("MO");
    }
    catch (const std::string &e)
    {
        CMN_LOG_CLASS_RUN_ERROR << "Disable motor failed  \"" << e << "\"" << std::endl;
        cmnThrow(std::runtime_error("Disable motor failed "));
    }
}

void mtsGalilController::EnableMotorPower(const vctBoolVec &mask)
{
    CMN_LOG_CLASS_RUN_VERBOSE << "EnableMotorPower \"" << mask << "\"" << std::endl;
    try
    {
        char buffer[G_SMALL_BUFFER];
        CreateCommandForAxis(buffer, "SH", mask);
        SendCommandString(buffer);
    }
    catch (const std::string &e)
    {
        CMN_LOG_CLASS_RUN_ERROR << "Enable motor failed  \"" << e << "\"" << std::endl;
        cmnThrow(std::runtime_error("Enable motor failed "));
    }
}

// MO not valid while running, need to issue ST first!!!! annoying.
void mtsGalilController::DisableMotorPower(const vctBoolVec &mask)
{
    CMN_LOG_CLASS_RUN_VERBOSE << "DisableMotorPower \"" << mask << "\"" << std::endl;
    try
    {
        StopMotion(mask);
        char buffer[G_SMALL_BUFFER];
        CreateCommandForAxis(buffer, "MO", mask);
        SendCommandString(buffer);
    }
    catch (const std::string &e)
    {
        CMN_LOG_CLASS_RUN_ERROR << "Disable motor Failed  \"" << e << "\"" << std::endl;
        cmnThrow(std::runtime_error("Disable motor Failed "));
    }
}

// Stop:  stop robot motion (do not disable motor power)
void mtsGalilController::StopMotionAll()
{
    try
    {
        CMN_LOG_CLASS_RUN_VERBOSE << "Stop ALL" << std::endl;
        vctBoolVec all(GetNumberActuators());
        all.SetAll(true);
        StopMotion(all);
    }
    catch (const std::string &e)
    {
        CMN_LOG_CLASS_RUN_ERROR << "Stop Motion failed  \"" << e << "\"" << std::endl;
        cmnThrow(std::runtime_error("Stop Motion error "));
    }
}

void mtsGalilController::StopMotion(const vctBoolVec &mask)
{
    CMN_LOG_CLASS_RUN_VERBOSE << "Stop \"" << mask << "\"" << std::endl;

    try
    {
        if (mask.Equal(false))
        { // otherwise all axes on???
            CMN_LOG_CLASS_RUN_ERROR << "StopMotion: Error  \" Mask empty " << std::endl;
            cmnThrow(std::runtime_error("StopMotion: Error - Mask Empty "));
        }
        else
        { // else stop motion.
            char buffer[G_SMALL_BUFFER];
            CreateCommandForAxis(buffer, "ST", mask);
            SendCommandString(buffer);
        }
    }
    catch (const std::string &e)
    {
        CMN_LOG_CLASS_RUN_ERROR << "Stop Motion failed  \"" << e << "\"" << std::endl;
        cmnThrow(std::runtime_error("Stop Motion  error "));
    }
}

// this is a bit tricky. the settings in the config file determine in which
// direction the homing procedure will start.
// last known velocity will be used for homing.
// THIS IS A BLOCKING COMMAND!!!!!!!!
// Start homing with the stage to the negative side of the home switch

void mtsGalilController::Home(const vctBoolVec &mask)
{
    CMN_LOG_CLASS_RUN_VERBOSE << "Homing \"" << mask << "\"" << std::endl;

    // I should move -50000 first in order to be in the correct zone for homing
    // but that can be done by the user
    char buffer[G_SMALL_BUFFER];
    // HOME ALL,
    try
    {
        StopMotion(mask); // stop all relavant
        SendCommandString("HM");

        // Now start the desired axes.
        CreateCommandForAxis(buffer, "BG", mask);
        SendCommandString(buffer);
        UnHome(mask);
        // wait to see if everything is finished
        WaitMotion(mask, 60);

        // if home is found, lets save the status in a variable.
        vctDoubleVec homeValue;
        homeValue.SetSize(mask.size());
        homeValue.SetAll(1);

        CreateCommand(buffer, "ZA", mask, homeValue);
        SendCommandString(buffer);
    }

    catch (const std::runtime_error &e)
    {
        CMN_LOG_CLASS_RUN_ERROR << "Homing failed: Error  \"" << e.what() << "\"" << std::endl;
        cmnThrow(std::runtime_error("Homing error "));
    }
    catch (const std::string &e)
    {
        CMN_LOG_CLASS_RUN_ERROR << "Homing failed: Error  \"" << e << "\"" << std::endl;
        cmnThrow(std::runtime_error("Homing error "));
    }
}

// UnHome:  Unhome the robot.  Besides clearing the IsHomed flag, this function turns off the forward and reverse
//          software travel limits, i.e., sets them to +/- 100mm
void mtsGalilController::UnHome(const vctBoolVec &mask)
{
    CMN_LOG_CLASS_RUN_VERBOSE << "UnHoming \"" << mask << "\"" << std::endl;

    vctDoubleVec homeValue;
    homeValue.SetSize(mask.size());
    homeValue.SetAll(0);

    char buffer[G_SMALL_BUFFER];
    try
    {
        CreateCommand(buffer, "ZA", mask, homeValue);
        SendCommandString(buffer);
    }
    catch (const std::string &e)
    {
        CMN_LOG_CLASS_RUN_ERROR << "UnHoming failed: Error  \"" << e << "\"" << std::endl;
        cmnThrow(std::runtime_error("UnHoming error "));
    }
}

void mtsGalilController::GetActuatorState(prmActuatorState &state)
{
    // InMotion returns the motion profile finished, typically there is some hysteresis in the control
    // algorithm, so the profile finishes assuming perfect tracking, and the servo loop tries to
    // catch up.
    state.InMotion().Zeros();
    state.MotorOff().Zeros();
    state.SoftFwdLimitHit().Zeros();
    state.SoftRevLimitHit().Zeros();
    state.HardFwdLimitHit().Zeros();
    state.HardRevLimitHit().Zeros();
    state.HomeSwitchOn().Zeros();

    m_SoftRevLimitHit.Zeros();
    m_SoftFwdLimitHit.Zeros();

    try
    {
        // Disha-encoder
        m_DecPosition = 0;
        for (unsigned int i = 0; i < m_NumberEncoderPins; i++)
        {
            m_DigitalInput[i] = this->SendCommandInt("MG " + DI[i]);
            m_DigitalInput[i] = !m_DigitalInput[i];

            m_DecPosition += m_DigitalInput[i] * pow(2, (9 - i));
        }

        for (unsigned int i = m_DecPosition >> 1; i != 0; i = i >> 1)
        {
            m_DecPosition = m_DecPosition ^ i;
        }

        // DegRotation= (dec_position/1024.0)*360.0;
        //      std::cout<<"Absolute encoder reading for Z axis is:"<<dec_position<<std::endl;

        // double =(galil->sourceValue(m_data, "_TM"]));
        for (unsigned int i = 0; i < GetNumberActuators(); i++)
        {
            // the encoders are in long
            state.Position()[i] = (long)this->SendCommandInt("MG " + TP[i]);

            // In the new version: This only applies to setting values, e.g., sp.
            // velocity still returns the actual Velocity * (TM/1000) so multiply it by 1000/tm
            // The TV command is computed using a special averaging filter (over approximately 0.25 sec for
            // TM1000). Therefore, TV will return average velocity, not instantaneous velocity.
            // this might be different for TMi250, not sure what the value will be
            // it might be 1/4 of actual velocity due to faster sampling

            state.Velocity()[i] = this->SendCommandDouble("MG " + TV[i]) * m_TMVelocityMultiplier;

            // analog inputs are not part of the actuator state??
            m_AnalogInput[i] = this->SendCommandDouble("MG " + AN[i]);

            // be careful, this is a motion profile variable not actual motion (there is a lag where the servo loop
            // catches up to the profile position)
            //  state.Velocity()[i]=(galil->sourceValue(m_data, TV[i]));
            if (this->SendCommandDouble("MG " + BG[i]) != 0.0)
            {
                state.InMotion()[i] = true;
            }

            // motor off
            if (this->SendCommandDouble("MG " + MO[i]) != 0.0)
            {
                state.MotorOff()[i] = true;
            }

            if (this->SendCommandDouble("MG " + ZA[i]) != 0.0)
            {
                state.IsHomed()[i] = true;
            }

            // TODO: double check the active ihigh/low configuration
            // here we assume that homeCFG=-1
            if (this->SendCommandDouble("MG " + HM[i]) != 0.0)
                state.HomeSwitchOn()[i] = true;

            // Decelerating or stopped by FWD limit switch OR soft limit FL
            if (this->SendCommandDouble("MG " + SC[i]) == 2)
            {
                state.SoftFwdLimitHit()[i] = true;
                m_SoftFwdLimitHit = true;
            }
            if (this->SendCommandDouble("MG " + SC[i]) == 3)
            {
                state.SoftRevLimitHit()[i] = true;
                m_SoftRevLimitHit = true;
            }
            /*
                0 Motors are running, independent mode
                10 Stopped after homing (HM)
                1 Motors decelerating or stopped at commanded independent position
                11 Stopped by Selective Abort Input
                2 Decelerating or stopped by FWD limit switch or soft limit FL
                12 Decelerating or stopped by encoder failure (OA1)
                3 Decelerating or stopped by REV limit switch or soft limit BL
                50 Contour running
                4 Decelerating or stopped by Stop Command (ST)
                51 Contour Stop
                6 Stopped by Abort input 99 MC timeout
                7 Stopped by Abort command (AB) 100 Motors are running, vector sequence
                8 Decelerating or stopped by Off on Error (OE1)
                101 Motors stopped at commanded vector
                9 Stopped after Finding Edge (FE)
            */

            // TODO: check, this might be differnt for CN-1, or CN1

            if (this->SendCommandDouble("MG " + LF[i]) == 0)
            {
                state.HardFwdLimitHit()[i] = true;
            }

            if (this->SendCommandDouble("MG " + LR[i]) == 0)
            {
                state.HardFwdLimitHit()[i] = true;
            }

            // user variable used to store state of home
            if (this->SendCommandDouble("MG " + ZA[i]) == 1)
            {
                state.IsHomed()[i] = true;
            }
            else
                state.IsHomed()[i] = false;
        }

        // ESTOP input is connected to Input 01 (first one) - ready state IN[01]=1, estop hit it goes low = 0

        if (this->SendCommandDouble("MG _TA3") == 3)
            state.EStopON() = true;
        else
            state.EStopON() = false;

        // Get the time
        // create a time index for the state
        // down cast from double to u long, i think the time is in controller is of cycles anyway.
        // it is 64K samples, so it only spans a minute or so
        // The TIME operand returns the value of the internal free running, real time clock.
        // The returned value represents the number of servo loop updates and is based on the TM command.
        // default value for the TM command is 1000. With this update rate, the operand TIME will
        // increase by 1 count every up date of approximately 1000usec. Note that a value of 1000 for
        // the update rate (TM command) will actually set an update rate of 976 microseconds. Thus the
        // value returned by the TIME operand will be off by 2.4% of the actual time.

        state.Timestamp() = this->SendCommandDouble("MG TIME");

        // TC is the error code;
        // 1 Unrecognized command 56 Array index invalid or out of range
        // 2 Command only valid from program 57 Bad function or array
        // 3 Command not valid in program 58 Bad command response (i.e._GNX)
        // 4 Operand error 59 Mismatched parentheses
        // 5 Input buffer full 60 Download error - line too long or too many lines
        // 6 Number out of range 61 Duplicate or bad label
        // 7 Command not valid while running 62 Too many labels
        // 8 Command not valid when not running 63 IF statement without ENDIF
        // 9 Variable error 65 IN command must have a comma
        // 10 Empty program line or undefined label 66 Array space full
        // 11 Invalid label or line number 67 Too many arrays or variables
        // 12 Subroutine more than 16 deep 71 IN only valid in task #0
        // 13 JG only valid when running in jog mode
        // 80 Record mode already running
        // 14 EEPROM check sum error 81 No array or source specified
        // 15 EEPROM write error 82 Undefined Array
        // 16 IP incorrect sign during position move
        // or IP given during forced deceleration
        // 83 Not a valid number
        // 17 ED, BN and DL not valid while program running
        // 84 Too many elements
        // 18 Command not valid when contouring 90 Only A B C D valid operand
        // 19 Application strand already executing 98 Binary Commands not valid in application program
        // 20 Begin not valid with motor off
        // 99 Bad binary command number
        // 21 Begin not valid while running 100 Not valid when running ECAM
        // 22 Begin not possible due to Limit Switch 101 Improper index into ET (must be 0-256)
    }
    catch (const std::string &e)
    {
        CMN_LOG_CLASS_RUN_ERROR << "GetActuatorState: Error  \"" << e << "\"" << std::endl;
        cmnThrow(std::runtime_error("GetActuatorState: Error "));
    }

    // unit conversions
    state.Position() = ConvertEncoderCountsToAxisUnit(state.Position());
    state.Velocity() = ConvertEncoderCountsToAxisUnit(state.Velocity());
}

void mtsGalilController::GetAnalogInputs(vctDoubleVec &ain) const
{

    if (ain.size() == m_AnalogInput.size())
    {
        ain = m_AnalogInput;
    }
    else
    {
        CMN_LOG_CLASS_RUN_ERROR << "GetAnalogInputs: Size of vectors mismatched" << std::endl;
        cmnThrow(std::runtime_error("GetAnalogInputs: Size of vectors mismatched "));
    }
}

// Disha-encoder
void mtsGalilController::GetToolZEncoder(int &toolZencoder) const
{
    toolZencoder = m_DecPosition;
}

// The expected values are in counts.
// Set the desired motion goals.
// this sets the desired actuator goal, also sets the velocity
// in PT - POSITION TRACKING MODE (PT1,1..) SP, AC, DC commands are allowed while the robot is moving.
// If in a different mode, ST (stop) command has to be called first.
void mtsGalilController::SetPositionMove(const prmMaskedDoubleVec &goalPosition)
{
    CMN_LOG_CLASS_RUN_VERBOSE << "SetPositionMove \"" << goalPosition << "\"" << std::endl;
    if (goalPosition.Mask().Equal(false))
    { // otherwise all axes on???
        CMN_LOG_CLASS_RUN_WARNING << "SetPositionMove: Error  \" Mask empty " << std::endl;
        cmnThrow(std::runtime_error("SetPositionMove: Error - Mask Empty "));
    }
    // this sets the mode on the controller in order to allow changes in goal absolute
    // position while moving. It also allows changing the velocity and accelerations on the fly.
    // if in jog mode then PT will shut it off and turn position tracking without BGA
    prmMaskedDoubleVec goalPositionEnc = ConvertAxisUnitToEncoderCounts(goalPosition);
    char buffer[G_SMALL_BUFFER];
    // create a command that will print PT1,,1,,,, etc.
    prmMaskedDoubleVec cmdPT(goalPosition.Mask().size());
    cmdPT.Data().SetAll(1);
    CreateCommandLong(buffer, "PT", goalPosition.Mask(), cmdPT.Data());
    SendCommandString(buffer);
    // if in motion then BG is not required

    CreateCommand(buffer, "PA", goalPosition.Mask(), goalPosition.Data());
    SendCommandString(buffer);

    // set only the ones that are desired
}

// does jog change the velocity for position move commands?
// Can also be used to stop the motion (set vel to zero)
// this sets the velocity rather then position
// in JG - JOG MODE (JGPT1,1..) SP, AC, DC commands are allowed while the robot is moving.
// This mode can only be entered when in JG mode, or after STOP. So if in TP mode, then
// the user has to //    vctBoolVec  startJogMask(NumberActuators);
//     for( unsigned int  i =0 ; i <NumberActuators; i++) {
//         if (!IsMoving[i] && goalVelocity.Mask()[i])
//             startJogMask = true;
//         else
//             startJogMask = false;
//      }call ST first.
void mtsGalilController::SetVelocityMove(const prmMaskedDoubleVec &goalVelocity)
{
    CMN_LOG_CLASS_RUN_VERBOSE << "SetVelocityMove \"" << goalVelocity << "\"" << std::endl;
    // set only the ones that are desired
    if (goalVelocity.Mask().Equal(false))
    {
        CMN_LOG_CLASS_RUN_ERROR << "SetVelocityMove: Error  \" Mask empty " << std::endl;
        cmnThrow(std::runtime_error("SetVelocityMove: Error - Mask Empty "));
    }

    // correct for TM2500
    prmMaskedDoubleVec goalVelocityEnc = ConvertAxisUnitToEncoderCounts(goalVelocity);
    // std::cerr << "Joint velocities being set by JG command:" << v <<std::endl;
    char buffer[G_SMALL_BUFFER];
    CreateCommand(buffer, "JG", goalVelocityEnc.Mask(), goalVelocityEnc.Data());
    SendCommandString(buffer);

    // Note when a soft limit switch is hit, it takes two ms to notice, so if we constantly call BG then the profile is started from scratch
    //  and the soft limit is only check on the following loop cycle in the galil controller.
    // so try to avoid calling BG after JG
    // just in case they are not moving.
    vctBoolVec startJogMask(goalVelocityEnc.Mask());

    for (unsigned int i = 0; i < GetNumberActuators(); i++)
    {
        if (m_SoftRevLimitHit[i] && (goalVelocityEnc.Data()[i] < 0))
            startJogMask[i] = false;

        if (m_SoftFwdLimitHit[i] && (goalVelocityEnc.Data()[i] > 0))
            startJogMask[i] = false;
    }

    // Check if we are in a profile mode?Stop code?
    CreateCommandForAxis(buffer, "BG", startJogMask); // this required if the robot is not moving in JG mode yet.
    // CreateCommandForAxis(buffer,"BG", goalVelocity.Mask());   //this required if the robot is not moving in JG mode yet.
    SendCommandString(buffer);
};

// Note: while the robot is moving we cannot change the acceleration or deceleration.
// this sets the velocity rather than position
void mtsGalilController::SetSpeed(const prmMaskedDoubleVec &speed)
{
    CMN_LOG_CLASS_RUN_VERBOSE << "Setting velocity \"" << speed << "\"" << std::endl;
    char buffer[G_SMALL_BUFFER];

    prmMaskedDoubleVec speedEnc = ConvertAxisUnitToEncoderCounts(speed);
    CreateCommand(buffer, "SP", speedEnc.Mask(), speedEnc.Data());
    SendCommandString(buffer);
}
void mtsGalilController::SetAcceleration(const prmMaskedDoubleVec &acceleration)
{
    CMN_LOG_CLASS_RUN_VERBOSE << "Setting acceleration \"" << acceleration << "\"" << std::endl;
    char buffer[G_SMALL_BUFFER];

    prmMaskedDoubleVec accelerationEnc = ConvertAxisUnitToEncoderCounts(acceleration);
    CreateCommand(buffer, "AC", accelerationEnc.Mask(), accelerationEnc.Data());
    SendCommandString(buffer);
}
void mtsGalilController::SetDeceleration(const prmMaskedDoubleVec &deceleration)
{
    CMN_LOG_CLASS_RUN_VERBOSE << "Setting deceleration \"" << deceleration << "\"" << std::endl;
    char buffer[G_SMALL_BUFFER];

    prmMaskedDoubleVec decelerationEnc = ConvertAxisUnitToEncoderCounts(deceleration);
    CreateCommand(buffer, "DC", decelerationEnc.Mask(), decelerationEnc.Data());
    SendCommandString(buffer);
}

void mtsGalilController::SetAbsolutePosition(const prmMaskedDoubleVec &position)
{
    CMN_LOG_CLASS_RUN_VERBOSE << "Setting absolute Position \"" << position << "\"" << std::endl;
    // unset
    //  UnHome(position.Mask());
    char buffer[G_SMALL_BUFFER];
    // Define Position
    prmMaskedDoubleVec positionEnc = ConvertAxisUnitToEncoderCounts(position);
    CreateCommand(buffer, "DP", positionEnc.Mask(), positionEnc.Data());
    SendCommandString(buffer);
}

void mtsGalilController::StopMovement(const vctBoolVec &mask, double timeout)
{
    osaStopwatch timer;
    timer.Reset();
    timer.Start();

    unsigned int ii;
    bool moving = true;
    char buffer[G_SMALL_BUFFER];
    for (ii = 0; ii < mask.size(); ii++)
    {
        if (mask[ii])
        {
            sprintf(buffer, "ST %c", 'A' + ii);
            this->SendCommandString(buffer);
        }
    }
    try
    {
        this->WaitMotion(mask, timeout);
    }
    catch (const std::runtime_error &e)
    {
        CMN_LOG_CLASS_RUN_ERROR << "Stop movement failed: " << e.what() << std::endl;
        cmnThrow(std::runtime_error(std::string("Stop movement failed with error") + e.what()));
    }
}

// WaitMotion:  Wait for the robot to stop the current motion
// waits for the specified axes
// specified timeout in seconds
void mtsGalilController::WaitMotion(const vctBoolVec &mask, double timeout)
{
    osaStopwatch timer;
    timer.Reset();
    timer.Start();

    unsigned int ii;
    bool moving = true;
    char buffer[G_SMALL_BUFFER];
    try
    {
        while (timer.GetElapsedTime() < timeout && moving)
        {
            moving = false;
            // std::vector<char>  m_data=galil->record("DR"); //reads DR packet
            // TODO: The is moving flag does not work very well.
            // for example, on a short move, the flag will be on for a impulse part of the move
            // and then when the integral component kicks in the moving flag is off...
            for (ii = 0; ii < mask.size(); ii++)
            {
                if (mask[ii])
                {
                    sprintf(buffer, "MG_BG%c", 'A' + ii);
                    if (this->SendCommandDouble(buffer) != 0.0)
                        moving = true;
                }
            }
            osaSleep(0.002);
        }
        if (timer.GetElapsedTime() > timeout)
            cmnThrow(std::runtime_error(std::string("Timeout reached:")));
    }
    catch (const std::string &e)
    {
        CMN_LOG_CLASS_RUN_ERROR << "Wait for Motion failed: " << e << std::endl;
        cmnThrow(std::runtime_error(std::string("Wait for motion error") + e));
    }
}

// SendCommand:  send a command to the Galil controller.
// Returns response string.
std::string mtsGalilController::SendCommandString(const std::string &cmd)
{
    if (!m_Galil)
    {
        cmnThrow(std::runtime_error("SendCommandString: ( No Controller Handle = Not Connected )"));
        return " ";
    }
    CMN_LOG_CLASS_RUN_DEBUG << "Sending to Galil [" << cmd << "]" << std::endl;

    char response[G_SMALL_BUFFER];
    try
    {
        CheckErrorGCommand(
            GCmdT(m_Galil, cmd.c_str(), response, G_SMALL_BUFFER, NULL));
    }
    catch (const GReturn &rc)
    {
        CMN_LOG_CLASS_RUN_ERROR << "Error for command \\" << cmd << "\\" << std::endl;
        CMN_LOG_CLASS_RUN_ERROR << "Galil error code: " << rc << std::endl;
        cmnThrow(std::runtime_error(std::string("SendCommandString:  Error sending command--") + std::to_string(rc)), CMN_LOG_LOD_RUN_ERROR);

        //  int t = atoi( (e.substr(0,1)).c_str() ); // get int for type, first digit of code
        //  int f = atoi( (e.substr(1,2)).c_str() ); // int for function, middle two digits of code
        //  int u = atoi( (e.substr(3,1)).c_str() ); // int for unique serial, last digit of code
        // ints can now be used in switch statements to handle all variations of error
        /*Throws:
        7020 INVALID COMMAND ERROR.  DL, UL, ED, and QD are not allowed from Galil::command()
        1010 TIMEOUT ERROR.  Galil::command("...") took longer than ... ms to write
        1011 TIMEOUT ERROR.  Galil::command("...") took longer than ... ms to read : response...
        2010 COMMAND ERROR.  Galil::command("...") got ? instead of : response...
        3010 MONITOR ERROR.  Galil::command("...") got > instead of : response.  Got...
        9020 UNINITIALIZED OBJECT ERROR.  Galil::command() called without Galil::address set*/
    }

    CMN_LOG_CLASS_RUN_VERBOSE << "Galil response to " << cmd << " is [" << response << "]" << std::endl;
    return std::string(response);
}

int mtsGalilController::SendCommandInt(const std::string &cmd)
{
    int response;
    if (!m_Galil)
    {
        cmnThrow(std::runtime_error("SendCommandInt: ( No Controller Handle = Not Connected )"));
    }
    CMN_LOG_CLASS_RUN_DEBUG << "Sending to Galil [" << cmd << "]" << std::endl;

    try
    {
        CheckErrorGCommand(
            GCmdI(m_Galil, cmd.c_str(), &response));
    }
    catch (const GReturn &rc)
    {
        CMN_LOG_CLASS_RUN_ERROR << "Error for command \\" << cmd << "\\" << std::endl;
        CMN_LOG_CLASS_RUN_ERROR << "Galil error code: " << rc << std::endl;
        cmnThrow(std::runtime_error(std::string("SendCommandInt:  Error sending command--") + std::to_string(rc)), CMN_LOG_LOD_RUN_ERROR);

        //  int t = atoi( (e.substr(0,1)).c_str() ); // get int for type, first digit of code
        //  int f = atoi( (e.substr(1,2)).c_str() ); // int for function, middle two digits of code
        //  int u = atoi( (e.substr(3,1)).c_str() ); // int for unique serial, last digit of code
        // ints can now be used in switch statements to handle all variations of error
        /*Throws:
        7020 INVALID COMMAND ERROR.  DL, UL, ED, and QD are not allowed from Galil::command()
        1010 TIMEOUT ERROR.  Galil::command("...") took longer than ... ms to write
        1011 TIMEOUT ERROR.  Galil::command("...") took longer than ... ms to read : response...
        2010 COMMAND ERROR.  Galil::command("...") got ? instead of : response...
        3010 MONITOR ERROR.  Galil::command("...") got > instead of : response.  Got...
        9020 UNINITIALIZED OBJECT ERROR.  Galil::command() called without Galil::address set*/
    }

    CMN_LOG_CLASS_RUN_VERBOSE << "Galil response to " << cmd << " is [" << response << "]" << std::endl;

    return response;
}

double mtsGalilController::SendCommandDouble(const std::string &cmd)
{
    double response;
    if (!m_Galil)
    {
        cmnThrow(std::runtime_error("SendCommandDouble: ( No Controller Handle = Not Connected )"));
    }
    CMN_LOG_CLASS_RUN_DEBUG << "Sending to Galil [" << cmd << "]" << std::endl;

    try
    {
        CheckErrorGCommand(
            GCmdD(m_Galil, cmd.c_str(), &response));
    }
    catch (const GReturn &rc)
    {
        CMN_LOG_CLASS_RUN_ERROR << "Error for command \\" << cmd << "\\" << std::endl;
        CMN_LOG_CLASS_RUN_ERROR << "Galil error code: " << rc << std::endl;
        cmnThrow(std::runtime_error(std::string("SendCommandDouble:  Error sending command--") + std::to_string(rc)), CMN_LOG_LOD_RUN_ERROR);

        //  int t = atoi( (e.substr(0,1)).c_str() ); // get int for type, first digit of code
        //  int f = atoi( (e.substr(1,2)).c_str() ); // int for function, middle two digits of code
        //  int u = atoi( (e.substr(3,1)).c_str() ); // int for unique serial, last digit of code
        // ints can now be used in switch statements to handle all variations of error
        /*Throws:
        7020 INVALID COMMAND ERROR.  DL, UL, ED, and QD are not allowed from Galil::command()
        1010 TIMEOUT ERROR.  Galil::command("...") took longer than ... ms to write
        1011 TIMEOUT ERROR.  Galil::command("...") took longer than ... ms to read : response...
        2010 COMMAND ERROR.  Galil::command("...") got ? instead of : response...
        3010 MONITOR ERROR.  Galil::command("...") got > instead of : response.  Got...
        9020 UNINITIALIZED OBJECT ERROR.  Galil::command() called without Galil::address set*/
    }

    CMN_LOG_CLASS_RUN_VERBOSE << "Galil response to " << cmd << " is [" << response << "]" << std::endl;

    return response;
}

// Helper functions for formatting the string commands sent to the controller.
void mtsGalilController::CreateCommand(char *buffer,
                                             const char *galilCmd,
                                             const vctBoolVec &mask,
                                             const vctDoubleVec &cmdParam)
{
    long bufpos;
    bufpos = sprintf(buffer, "%s", galilCmd);

    prmMaskedDoubleVec cmdParamAxis(cmdParam, mask);
    prmMaskedDoubleVec cmdParamGalil = RemapAxisValues(cmdParamAxis); // remap to galil space

    unsigned int ii;
    for (ii = 0; ii < cmdParamGalil.Mask().size() - 1; ii++)
    {
        if (cmdParamGalil.Mask()[ii])
            bufpos += sprintf(buffer + bufpos, "%f,", cmdParamGalil.Data()[ii]);
        else
            bufpos += sprintf(buffer + bufpos, ",");
    }
    if (mask[ii])
    {
        sprintf(buffer + bufpos, "%f", cmdParamGalil.Data()[cmdParamGalil.Data().size() - 1]);
        // else don't add anything so it is not set.
    }
}

// Helper functions for formatting the string commands sent to the controller.
void mtsGalilController::CreateCommandLong(char *buffer,
                                                 const char *galilCmd,
                                                 const vctBoolVec &mask,
                                                 const vctDoubleVec &cmdParam)
{
    long bufpos;
    bufpos = sprintf(buffer, "%s", galilCmd);
    unsigned int ii;
    prmMaskedDoubleVec cmdParamAxis(cmdParam, mask);
    prmMaskedDoubleVec cmdParamGalil = RemapAxisValues(cmdParamAxis); // remap to galil space

    for (ii = 0; ii < cmdParamGalil.Mask().size() - 1; ii++)
    {
        if (cmdParamGalil.Mask()[ii])
            bufpos += sprintf(buffer + bufpos, "%ld,", (long)cmdParamGalil.Data()[ii]);
        else
            bufpos += sprintf(buffer + bufpos, ",");
    }
    if (cmdParamGalil.Mask()[ii])
    {
        sprintf(buffer + bufpos, "%ld", (long)cmdParamGalil.Data()[cmdParamGalil.Data().size() - 1]);
        // else don't add anything so it is not set.
    }
}

void mtsGalilController::CreateCommandForAxis(char *buffer,
                                                    const char *galilCmd,
                                                    const vctBoolVec &mask)
{
    //"CMD ABCDEFGH"
    // ascii A is 65
    // Note a blank mask calls the command for all axis. (has some undesirable side effects for ST command)
    long bufpos;
    bufpos = sprintf(buffer, "%s ", galilCmd);

    vctBoolVec maskGalil = RemapAxisMask(mask);
    
    for (unsigned int ii = 0; ii < maskGalil.size(); ii++)
    {
        if (maskGalil[ii])
        {
            bufpos += sprintf(buffer + bufpos, "%c", 'A' + ii); // A+next char
        }
    }
}

GDataRecord mtsGalilController::RecordData(const DataRecordMethod &method)
{
    GDataRecord record;
    // DR -> Non-blocking asynchronous | QR -> Query-based
    GRecord(
        m_Galil,
        &record,
        method == DataRecordMethod::QR ? G_QR : G_DR);

    return record;
}

// This should be never used.
void mtsGalilController::Reset()
{
    this->SendCommandString("RS");
}

void mtsGalilController::ProgramUploadFile(const std::string &filepath)
{
    GProgramUploadFile(m_Galil, filepath.c_str());
}

void mtsGalilController::SetEncoderCountConversionFactors(const prmMaskedDoubleVec& conversionFactors)
{
    for (size_t i = 0; i < m_EncoderCountsPerUnit.size(); i++)
    {
        // set only conversion factors that have a mask
        if (!conversionFactors.Mask()[i])
            continue;

        m_EncoderCountsPerUnit[i] = conversionFactors.Data()[i];
    }
}

prmMaskedDoubleVec mtsGalilController::ConvertAxisUnitToEncoderCounts(const prmMaskedDoubleVec &axisUnits) const
{
    prmMaskedDoubleVec result(axisUnits.Data().size());
    auto axisConversionFactors = GetEncoderCountConversionFactors();

    axisUnits.GetMask(result.Mask()); // copy over the result mask

    for (size_t i = 0; i < axisUnits.Data().size(); i++)
    {
        if (!axisUnits.Mask()[i])
            continue;

        result.Data()[i] = axisUnits.Data()[i] * axisConversionFactors[i];
    }

    return result;
}

prmMaskedIntVec mtsGalilController::ConvertAxisUnitToEncoderCountsRounded(const prmMaskedDoubleVec &axisUnits) const
{
    prmMaskedDoubleVec resultFull = ConvertAxisUnitToEncoderCounts(axisUnits);
    prmMaskedIntVec resultRounded;

    resultFull.GetMask(resultRounded.Mask());

    for (size_t i = 0; i < resultFull.Data().size(); i++)
    {
        resultRounded.Data()[i] = (int)std::round(resultFull.Data()[i]);
    }

    return resultRounded;
}

prmMaskedDoubleVec mtsGalilController::ConvertEncoderCountsToAxisUnit(const prmMaskedDoubleVec &encoderCounts) const
{
    prmMaskedDoubleVec result(encoderCounts.Data().size());
    auto axisConversionFactors = GetEncoderCountConversionFactors();

    encoderCounts.GetMask(result.Mask()); // copy over the result mask

    for (size_t i = 0; i < encoderCounts.Data().size(); i++)
    {
        if (!encoderCounts.Mask()[i])
            continue;

        result.Data()[i] = (int)std::round(encoderCounts.Data()[i] / axisConversionFactors[i]);
    }

    return result;
}

prmMaskedDoubleVec mtsGalilController::ConvertEncoderCountsToAxisUnit(const prmMaskedIntVec &encoderCounts) const
{
    prmMaskedDoubleVec encoderCountsDbl(encoderCounts.Data().size());

    for (size_t i = 0; i < encoderCountsDbl.Data().size(); i++)
    {
        encoderCountsDbl.Data()[i] = (double)encoderCounts.Data()[i];
        encoderCountsDbl.Mask()[i] = encoderCounts.Mask()[i];
    }

    return ConvertEncoderCountsToAxisUnit(encoderCountsDbl);
}

unsigned int mtsGalilController::RemapAxisIndex(const unsigned int index)
{
    return m_AxisToGalilChannelMappings[index];
}

unsigned int mtsGalilController::RemapGalilIndex(const unsigned int index, bool& valid)
{
    valid = m_GalilToAxisChannelMappings.Mask()[index];
    return m_GalilToAxisChannelMappings.Data()[index];
}

template <typename T> prmMaskedVector<T> mtsGalilController::RemapAxisValues(const vctDynamicVector<T>& axisValues)
{
    prmMaskedVector<T> galilValues(m_AxisToGalilChannelMappings.MaxElement());

    for (size_t idxGalil = 0; idxGalil < galilValues.Data().size(); idxGalil++)
    {
        bool   idxGalilValid;
        size_t idxAxis = RemapGalilIndex(idxGalil, idxGalilValid);

        if (!idxGalilValid)
            continue;

        galilValues.Data()[idxGalil] = axisValues[idxAxis];
        galilValues.Mask()[idxGalil] = true;
    }

    return galilValues;
}

template <typename T> prmMaskedVector<T> mtsGalilController::RemapAxisValues(const prmMaskedVector<T>& axisValues)
{
    prmMaskedVector<T> galilValues(m_AxisToGalilChannelMappings.MaxElement());

    for (size_t idxGalil = 0; idxGalil < galilValues.Data().size(); idxGalil++)
    {
        bool   idxGalilValid;
        size_t idxAxis = RemapGalilIndex(idxGalil, idxGalilValid);

        if (!idxGalilValid)
            continue;

        galilValues.Data()[idxGalil] = axisValues.Data()[idxAxis];
        galilValues.Mask()[idxGalil] = axisValues.Mask()[idxAxis];
    }

    return galilValues;
}

vctBoolVec mtsGalilController::RemapAxisMask(const vctBoolVec& axisMask)
{
    vctBoolVec galilMask(m_AxisToGalilChannelMappings.MaxElement());

    for (size_t idxGalil = 0; idxGalil < galilMask.size(); idxGalil++)
    {
        bool   idxGalilValid;
        size_t idxAxis = RemapGalilIndex(idxGalil, idxGalilValid);

        if (!idxGalilValid)
            continue;

        galilMask[idxGalil] = axisMask[idxGalil];
    }
    return galilMask;   // PK TODO: Added to compile
}
