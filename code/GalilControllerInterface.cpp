#include "mtsGalilController/GalilControllerInterface.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <bitset>
#include <cisstOSAbstraction.h>

// The class is registered with a default level of detail 5 for the logs.
// Levels of details are
// 1: Errors during init
// 2: Warnings during init,
// 3: Messages during init,
// 5: Error during normal operations,
// 6: Warnings during normal operations
// 7: Messages during normal operations
// 8: Very verbose, i.e. log all connections to the Galil controller.

std::unordered_map<std::string, std::shared_ptr<GalilControllerInterface>> GalilControllerInterfaceFactory::s_instances = {};

CMN_IMPLEMENT_SERVICES(GalilControllerInterface);

// disable sprintf warnings
#pragma warning(disable : 4996)

//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////

GalilControllerInterface::GalilControllerInterface()
{

    // set the size of the vectors:
    m_IsHomed.SetSize(NumberActuators);
    m_IsHomed.SetAll(false); // this state is not available on the controller

    m_AnalogInput.SetSize(NumberActuators);
    m_AnalogInput.Zeros();

    m_EncoderCountsPerUnit.SetSize(NumberActuators);
    m_EncoderCountsPerUnit.SetAll(1.0); // default to use encoder counts

    // Disha- encoder
    unsigned int Encoder_pins[] = {9, 12, 15, 11, 14, 10, 13, 8, 1, 2};
    m_NumberEncoderPins = sizeof(Encoder_pins) / sizeof(Encoder_pins[0]);
    m_DigitalInput.SetSize(m_NumberEncoderPins);
    m_DigitalInput.Zeros();

    m_galil = nullptr;
    // max number of actuator is 8 here
    char analogStr[7];
    // Disha-encoder
    char digitalStr[m_NumberEncoderPins];
    memset(digitalStr, 0, sizeof(char) * m_NumberEncoderPins);
    memset(analogStr, 0, sizeof(char) * 7);

    m_SoftRevLimitHit.SetSize(NumberActuators);
    m_SoftFwdLimitHit.SetSize(NumberActuators);

    const char *letter = "ABCDEFGHIJKLMNOPQRSTUVWY";
    // create commands for all desired variable commands
    for (unsigned int i = 0; i < NumberActuators; i++)
    {
        TP.push_back(std::string("_TP").append(1, letter[i]));
        TV.push_back(std::string("_TV").append(1, letter[i])); // vel (filtered) counts/sec
        BG.push_back(std::string("_BG").append(1, letter[i])); // axis moving
        HM.push_back(std::string("_HM").append(1, letter[i])); // home
        LF.push_back(std::string("_LF").append(1, letter[i])); // for limit
        LR.push_back(std::string("_LR").append(1, letter[i])); // rev limit
        MO.push_back(std::string("_MO").append(1, letter[i])); // Motor off
        SC.push_back(std::string("_SC").append(1, letter[i])); // stop code
        ZA.push_back(std::string("_ZA").append(1, letter[i])); // user variable   can be used to store state of home

        // analog inputs, generate appropriate strings  -- begins with 1
        // std::string  analog("@AN[1]");
        // itoa(i+1,digit,10);
        // analog.replace(4,1,digit);
        sprintf(analogStr, "@AN[%d]", i + 2);
        AN.push_back(std::string(analogStr));
        // std::cout<<analogStr<<std::endl;
    }

    // Disha-encoder
    for (unsigned int i = 0; i < m_NumberEncoderPins; i++)
    {
        sprintf(digitalStr, "@IN[%02d]", Encoder_pins[i]);
        DI.push_back(std::string(digitalStr));
    }
}

GalilControllerInterface::GalilControllerInterface(const GalilControllerInterface &controller)
{
    *this = controller;
}

GalilControllerInterface::~GalilControllerInterface()
{
    Close();
}

GCStringOut GalilControllerInterface::BufferToGCStringOut(char *buffer, unsigned int buffer_size)
{
    GCStringOut stringout = new char[G_SMALL_BUFFER];

    memcpy(stringout, buffer, buffer_size);

    return stringout;
}

void GalilControllerInterface::AbortProgram()
{
    this->SendCommand("AB");
}

void GalilControllerInterface::AbortMotion()
{
    this->SendCommand("AB 1");
}

void GalilControllerInterface::Close()
{

    if (m_galil)
    {
        CMN_LOG_CLASS_RUN_VERBOSE << "Closing Galil controller" << std::endl;
        SendCommand("ST");
        DisableAllMotorPower();
        GClose(m_galil);
        delete m_galil;
    }
}

// VerifyStatus:  check the robot status and throw an exception for any set status bit
//                that matches the specified mask (statusMask).
//
// Note:  This function will silently attempt to enable the motor power if it is not
//        already on and statusMask has the MOTOR_OFF_MASK bit set.
// TODO: added ESTOP input to PIN 1 , it is possible to test for that
// here
long GalilControllerInterface::VerifyStatus(const char * /*callerName*/, long /*statusMask*/)
{
    return 0;
}

//////////////////////////////////////////////////////////////////////
// Public member functions
//////////////////////////////////////////////////////////////////////

// Init:  initialize robot (must be called)
// This function establishes communication with the Galil controller,
// configures it, downloads the application program and then starts execution.
bool GalilControllerInterface::Init(const std::string &deviceName)
{
    if (m_galil)
        return true;

    try
    {
        std::cout << "Hello Galil" << std::endl;

        GOpen(deviceName.c_str(), &m_galil);

        // Stop motors and halt Galil Controller program execution
        this->StopMotionAll();

        m_ServoLoopTime = this->SendCommandDouble("TM?");
        // Loop time is used to correct for speed scaling due to TM250
        m_TMVelocityMultiplier = 1000.0 / m_ServoLoopTime;

        CMN_LOG_CLASS_INIT_VERBOSE << "Galil Servo Loop Time is " << m_ServoLoopTime / 1000.0 << " ms" << std::endl;
    }
    catch (std::string e) // error
    {
        CMN_LOG_CLASS_INIT_ERROR << e << std::endl;
        if (std::string::npos != e.find("COMMAND ERROR"))
            CMN_LOG_CLASS_INIT_ERROR << "a command error occurred" << std::endl; // special processing for command errors

        return false;
    }
    catch (ExcpSystemError e) // error
    {
        CMN_LOG_CLASS_RUN_ERROR << e.what() << std::endl;
        return false;
    }

    // Add the instances
    GalilControllerInterfaceFactory::s_instances.insert({deviceName, std::shared_ptr<GalilControllerInterface>(this)});

    // Enable motor power.  This will fail if the EStop switch is pressed,
    // but that isn't considered an error at this point.
    return true;
}

// EnableMotorPower:  turns on the motor servos
void GalilControllerInterface::EnableAllMotorPower() throw(ExcpPowerOff, ExcpSystemError)
{
    CMN_LOG_CLASS_RUN_VERBOSE << "EnableMotorPower " << std::endl;
    //	VerifyStatus("EnableMotorPower", ESTOP_MASK);
    try
    {
        this->SendCommand("SH");
    }
    catch (std::string e)
    {
        CMN_LOG_CLASS_RUN_ERROR << "Enable motor failed  \"" << e << "\"" << std::endl;
        cmnThrow(ExcpSystemError("Enable motor failed "));
    }
}

// DisableMotorPower:  turns off the motor servos
// MO not valid while running, need to issue ST first!!!! annoying.
void GalilControllerInterface::DisableAllMotorPower()
{
    CMN_LOG_CLASS_RUN_VERBOSE << "DisableMotorPower " << std::endl;
    try
    {
        this->StopMotionAll();
        SendCommand("MO");
    }
    catch (std::string e)
    {
        CMN_LOG_CLASS_RUN_ERROR << "Disable motor failed  \"" << e << "\"" << std::endl;
        cmnThrow(ExcpSystemError("Disable motor failed "));
    }
}

void GalilControllerInterface::EnableMotorPower(const mtsBoolVec &mask)
{
    CMN_LOG_CLASS_RUN_VERBOSE << "EnableMotorPower \"" << mask << "\"" << std::endl;
    try
    {
        char buffer[G_SMALL_BUFFER];
        CreateCommandForAxis(buffer, "SH", mask);
        SendCommand(buffer);
    }
    catch (std::string e)
    {
        CMN_LOG_CLASS_RUN_ERROR << "Enable motor failed  \"" << e << "\"" << std::endl;
        cmnThrow(ExcpSystemError("Enable motor failed "));
    }
}

// MO not valid while running, need to issue ST first!!!! annoying.
void GalilControllerInterface::DisableMotorPower(const mtsBoolVec &mask)
{
    CMN_LOG_CLASS_RUN_VERBOSE << "DisableMotorPower \"" << mask << "\"" << std::endl;
    try
    {
        StopMotion(mask);
        char buffer[G_SMALL_BUFFER];
        CreateCommandForAxis(buffer, "MO", mask);
        SendCommand(buffer);
    }
    catch (std::string e)
    {
        CMN_LOG_CLASS_RUN_ERROR << "Disable motor Failed  \"" << e << "\"" << std::endl;
        cmnThrow(ExcpSystemError("Disable motor Failed "));
    }
}

// Stop:  stop robot motion (do not disable motor power)
void GalilControllerInterface::StopMotionAll()
{
    try
    {
        CMN_LOG_CLASS_RUN_VERBOSE << "Stop ALL" << std::endl;
        // SendCommand("ST ABCDEFGH");
        mtsBoolVec all(NumberActuators);
        all.SetAll(true);
        StopMotion(all);
    }
    catch (std::string e)
    {
        CMN_LOG_CLASS_RUN_ERROR << "Stop Motion failed  \"" << e << "\"" << std::endl;
        cmnThrow(ExcpSystemError("Stop Motion error "));
    }
}

void GalilControllerInterface::StopMotion(const mtsBoolVec &mask)
{
    CMN_LOG_CLASS_RUN_VERBOSE << "Stop \"" << mask << "\"" << std::endl;

    try
    {
        if (mask.Equal(false))
        { // otherwise all axes on???
            CMN_LOG_CLASS_RUN_ERROR << "StopMotion: Error  \" Mask empty " << std::endl;
            cmnThrow(ExcpSystemError("StopMotion: Error - Mask Empty "));
        }
        else
        { // else stop motion.
            char buffer[G_SMALL_BUFFER];
            CreateCommandForAxis(buffer, "ST", mask);
            SendCommand(buffer);
        }
    }
    catch (std::string e)
    {
        CMN_LOG_CLASS_RUN_ERROR << "Stop Motion failed  \"" << e << "\"" << std::endl;
        cmnThrow(ExcpSystemError("Stop Motion  error "));
    }
}

// this is a bit tricky. the settings in the config file determine in which
// direction the homing procedure will start.
// last known velocity will be used for homeing.
// THIS IS A BLOCKING COMMAND!!!!!!!!
// Start homing with the stage to the negative side of the home switch

void GalilControllerInterface::Home(const mtsBoolVec &mask)
{
    CMN_LOG_CLASS_RUN_VERBOSE << "Homing \"" << mask << "\"" << std::endl;

    // I should move -50000 first in order to be in the correct zone for homing
    // but that can be done by the user
    char buffer[G_SMALL_BUFFER];
    // HOME ALL,
    try
    {
        StopMotion(mask); // stop all relavant
        SendCommand("HM");

        // Now start the desired axes.
        CreateCommandForAxis(buffer, "BG", mask);
        SendCommand(buffer);
        UnHome(mask);
        // wait to see if everything is finished
        WaitMotion(mask, 60);

        // if home is found, lets save the status in a variable.
        mtsDoubleVec homeValue;
        homeValue.SetSize(mask.size());
        homeValue.SetAll(1);

        CreateCommand(buffer, "ZA", mask, homeValue);
        SendCommand(buffer);
    }

    catch (ExcpWaitMotion e)
    {
        CMN_LOG_CLASS_RUN_ERROR << "Homing failed: Error  \"" << e.what() << "\"" << std::endl;
        cmnThrow(ExcpSystemError("Homing error "));
    }
    catch (ExcpSystemError e)
    {
        CMN_LOG_CLASS_RUN_ERROR << "Homing failed: Error  \"" << e.what() << "\"" << std::endl;
        cmnThrow(ExcpSystemError("Homing error "));
    }
    catch (std::string e)
    {
        CMN_LOG_CLASS_RUN_ERROR << "Homing failed: Error  \"" << e << "\"" << std::endl;
        cmnThrow(ExcpSystemError("Homing error "));
    }
}

// UnHome:  Unhome the robot.  Besides clearing the IsHomed flag, this function turns off the forward and reverse
//          software travel limits i.e sets them to +/- 100mm
void GalilControllerInterface::UnHome(const mtsBoolVec &mask)
{
    CMN_LOG_CLASS_RUN_VERBOSE << "UnHoming \"" << mask << "\"" << std::endl;

    mtsDoubleVec homeValue;
    homeValue.SetSize(mask.size());
    homeValue.SetAll(0);

    char buffer[G_SMALL_BUFFER];
    try
    {
        CreateCommand(buffer, "ZA", mask, homeValue);
        SendCommand(buffer);
    }
    catch (std::string e)
    {
        CMN_LOG_CLASS_RUN_ERROR << "UnHoming failed: Error  \"" << e << "\"" << std::endl;
        cmnThrow(ExcpSystemError("UnHoming error "));
    }
}

void GalilControllerInterface::GetActuatorState(prmActuatorState &state) throw(ExcpSystemError, RobotException)
{
    // InMotion returns the motion profile finished, typically there is some histersis in the control
    // algorithm, so the profile finishes assuming perfect tracking, and the servo loop tries to
    // catch up.
    state.SetSize(NumberActuators);

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
            m_DigitalInput[i] = this->SendCommandInt("MG_" + DI[i]);
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
        for (unsigned int i = 0; i < NumberActuators; i++)
        {
            // the encoders are in long
            state.Position()[i] = (long)this->SendCommandInt("MG_" + TP[i]);

            // In the new version: This only applies to setting values ,eg sp.
            // velocity still returns the actual Velocity * (TM/1000) so multiply it by 1000/tm
            // The TV command is computed using a special averaging filter (over approximately 0.25 sec for
            // TM1000). Therefore, TV will return average velocity, not instantaneous velocity.
            // this might be different for TMi250, not sure what the value will be
            // it might be 1/4 of actual velocity due to faster sampling

            state.Velocity()[i] = this->SendCommandDouble("MG_" + TV[i]) * m_TMVelocityMultiplier;

            // analog inputs are not part of the actuator state??
            m_AnalogInput[i] = this->SendCommandDouble("MG_" + AN[i]);

            // be careful, this is a motion profile variable not actual motion (there is a lag where the servo loop
            // catches up to the profile position)
            //  state.Velocity()[i]=(galil->sourceValue(m_data, TV[i]));
            if (this->SendCommandDouble("MG _" + BG[i]) != 0.0)
            {
                state.InMotion()[i] = true;
            }

            // motor off
            if (this->SendCommandDouble("MG _" + MO[i]) != 0.0)
            {
                state.MotorOff()[i] = true;
            }

            if (this->SendCommandDouble("MG _" + ZA[i]) != 0.0)
            {
                state.IsHomed()[i] = true;
            }

            // TODO: double check the active ihigh/low configuration
            // here we assume that homeCFG=-1
            if (this->SendCommandDouble("MG _" + HM[i]) != 0.0)
                state.HomeSwitchOn()[i] = true;

            // Decelerating or stopped by FWD limit switch OR soft limit FL
            if (this->SendCommandDouble("MG _" + SC[i]) == 2)
            {
                state.SoftFwdLimitHit()[i] = true;
                m_SoftFwdLimitHit = true;
            }
            if (this->SendCommandDouble("MG _" + SC[i]) == 3)
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

            if (this->SendCommandDouble("MG _" + LF[i]) == 0)
            {
                state.HardFwdLimitHit()[i] = true;
            }

            if (this->SendCommandDouble("MG _" + LR[i]) == 0)
            {
                state.HardFwdLimitHit()[i] = true;
            }

            // user variable used to store state of home
            if (this->SendCommandDouble("MG _" + ZA[i]) == 1)
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

        state.Timestamp() = this->SendCommandDouble("TIME");

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
    catch (std::string e)
    {
        CMN_LOG_CLASS_RUN_ERROR << "GetActuatorState: Error  \"" << e << "\"" << std::endl;
        cmnThrow(ExcpSystemError("GetActuatorState: Error "));
    }
}

void GalilControllerInterface::GetAnalogInputs(mtsDoubleVec &ain) const
{

    if (ain.size() == m_AnalogInput.size())
    {
        ain = m_AnalogInput;
    }
    else
    {
        CMN_LOG_CLASS_RUN_ERROR << "GetAnalogInputs: Size of vectors mismatched" << std::endl;
        cmnThrow(ExcpSystemError("GetAnalogInputs: Size of vectors mismatched "));
    }
}

// Disha-encoder
void GalilControllerInterface::GetToolZEncoder(mtsInt &toolZencoder) const
{

    toolZencoder = m_DecPosition;
}

// The expected  values are in counts.
// Set the desired motion goals.
// this sets the desired actuator goal, also sets the velocity
// in PT - POSITION TRACKING MODE (PT1,1..) SP, AC, DC commands are allowed while the robot is moving.
// If in a differnet mode, ST (stop) command has to be called first.
void GalilControllerInterface::SetPositionMove(const prmMaskedDoubleVec &goalPosition) throw(ExcpSystemError, RobotException)
{
    CMN_LOG_CLASS_RUN_VERBOSE << "SetPositionMove \"" << goalPosition << "\"" << std::endl;
    if (goalPosition.Mask().Equal(false))
    { // otherwise all axes on???
        CMN_LOG_CLASS_RUN_WARNING << "SetPositionMove: Error  \" Mask empty " << std::endl;
        cmnThrow(ExcpSystemError("SetPositionMove: Error - Mask Empty "));
    }
    // this sets the mode on the controller in order to allow changes in goal absolute
    // position while moving. It also allows changing the velocity and accelerations on the fly.
    // if in jog mode then PT will shut it off and turn position tracking without BGA
    prmMaskedDoubleVec goalPositionEnc = ConvertAxisUnitToEncoderCounts(goalPosition);
    char buffer[G_SMALL_BUFFER];
    // createa  command that will print PT1,,1,,,, etc.
    prmMaskedDoubleVec cmdPT(goalPosition.Mask().size());
    cmdPT.Data().SetAll(1);
    CreateCommandLong(buffer, "PT", goalPosition.Mask(), cmdPT.Data());
    SendCommand(buffer);
    // if in motion then BG is not required

    CreateCommand(buffer, "PA", goalPosition.Mask(), goalPosition.Data());
    SendCommand(buffer);

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
void GalilControllerInterface::SetVelocityMove(const prmMaskedDoubleVec &goalVelocity) throw(ExcpSystemError, RobotException)
{
    CMN_LOG_CLASS_RUN_VERBOSE << "SetVelocityMove \"" << goalVelocity << "\"" << std::endl;
    // set only the ones that are desired
    if (goalVelocity.Mask().Equal(false))
    {
        CMN_LOG_CLASS_RUN_ERROR << "SetVelocityMove: Error  \" Mask empty " << std::endl;
        cmnThrow(ExcpSystemError("SetVelocityMove: Error - Mask Empty "));
    }

    // correct for TM2500
    prmMaskedDoubleVec goalVelocityEnc = ConvertAxisUnitToEncoderCounts(goalVelocity);
    // std::cerr << "Joint velocities being set by JG command:" << v <<std::endl;
    char buffer[G_SMALL_BUFFER];
    CreateCommand(buffer, "JG", goalVelocityEnc.Mask(), goalVelocityEnc.Data());
    SendCommand(buffer);

    // Note when a soft limit switch is hit, it takes two ms to notice, so if we constantly call BG then the profile is started from scratch
    //  and the soft limit is only check on the following loop cycle in the galil controller.
    // so try to avoid callign BG after JG

    // just in case they are not moving.
    vctBoolVec startJogMask(goalVelocityEnc.Mask());

    for (unsigned int i = 0; i < NumberActuators; i++)
    {
        if (m_SoftRevLimitHit[i] && (goalVelocityEnc.Data()[i] < 0))
            startJogMask[i] = false;

        if (m_SoftFwdLimitHit[i] && (goalVelocityEnc.Data()[i] > 0))
            startJogMask[i] = false;
    }

    // Check if we are in a profile mode?Stop code?
    CreateCommandForAxis(buffer, "BG", startJogMask); // this required if the robot is not moving in JG mode yet.
    // CreateCommandForAxis(buffer,"BG", goalVelocity.Mask());   //this required if the robot is not moving in JG mode yet.
    SendCommand(buffer);
};

// Note : while the robot is moving we can not change the acceleration or decelaration.
// this sets the velocity rather then position
void GalilControllerInterface::SetSpeed(const prmMaskedDoubleVec &speed) throw(ExcpSystemError, RobotException)
{
    CMN_LOG_CLASS_RUN_VERBOSE << "Setting velocity \"" << speed << "\"" << std::endl;
    char buffer[G_SMALL_BUFFER];

    prmMaskedDoubleVec speedEnc = ConvertAxisUnitToEncoderCounts(speed);
    CreateCommand(buffer, "SP", speedEnc.Mask(), speedEnc.Data());
    SendCommand(buffer);
}
void GalilControllerInterface::SetAcceleration(const prmMaskedDoubleVec &acceleration) throw(ExcpSystemError, RobotException)
{
    CMN_LOG_CLASS_RUN_VERBOSE << "Setting acceleration \"" << acceleration << "\"" << std::endl;
    char buffer[G_SMALL_BUFFER];

    prmMaskedDoubleVec accelerationEnc = ConvertAxisUnitToEncoderCounts(acceleration);
    CreateCommand(buffer, "AC", accelerationEnc.Mask(), accelerationEnc.Data());
    SendCommand(buffer);
}
void GalilControllerInterface::SetDeceleration(const prmMaskedDoubleVec &deceleration) throw(ExcpSystemError, RobotException)
{
    CMN_LOG_CLASS_RUN_VERBOSE << "Setting deceleration \"" << deceleration << "\"" << std::endl;
    char buffer[G_SMALL_BUFFER];

    prmMaskedDoubleVec decelerationEnc = ConvertAxisUnitToEncoderCounts(deceleration);
    CreateCommand(buffer, "DC", decelerationEnc.Mask(), decelerationEnc.Data());
    SendCommand(buffer);
}

void GalilControllerInterface::SetAbsolutePosition(const prmMaskedDoubleVec &position) throw(ExcpSystemError, RobotException)
{
    CMN_LOG_CLASS_RUN_VERBOSE << "Setting absolute Position \"" << position << "\"" << std::endl;
    // unset
    //  UnHome(position.Mask());
    char buffer[G_SMALL_BUFFER];
    // Define Position
    prmMaskedDoubleVec positionEnc = ConvertAxisUnitToEncoderCounts(position);
    CreateCommand(buffer, "DP", positionEnc.Mask(), positionEnc.Data());
    SendCommand(buffer);
}

void GalilControllerInterface::StopMovement(const mtsBoolVec &mask, double timeout) throw(ExcpWaitMotion, ExcpSystemError)
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
            this->SendCommand(buffer);
        }
    }
    try
    {
        this->WaitMotion(mask, timeout);
    }
    catch (ExcpSystemError e)
    {
        CMN_LOG_CLASS_RUN_ERROR << "Stop movement failed: " << e.what() << std::endl;
        cmnThrow(ExcpSystemError(std::string("Stop movement failed with error") + e.what()));
    }
}

// WaitMotion:  Wait for the robot to stop the current motion
// waits for the specified axes
// specified timeout in seconds
void GalilControllerInterface::WaitMotion(const mtsBoolVec &mask, double timeout) throw(ExcpWaitMotion, ExcpSystemError)
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
            cmnThrow(ExcpWaitMotion(std::string("Timeout reached:")));
    }
    catch (std::string e)
    {
        CMN_LOG_CLASS_RUN_ERROR << "Wait for Motion failed: " << e << std::endl;
        cmnThrow(ExcpSystemError(std::string("Wait for motion error") + e));
    }
}

// SendCommand:  send a command to the Galil controller.
// Returns ref to the static buffer for the response message.
std::string GalilControllerInterface::SendCommand(const std::string &cmd)
{
    GCStringOut response;
    if (!m_galil)
    {

        cmnThrow(ExcpSystemError("SendCommand: ( No Controller Handle = Not Connected )"));
        return " ";
    }
    CMN_LOG_CLASS_RUN_DEBUG << "Sending to Galil [" << cmd << "]" << std::endl;

    GCStringOut buffer;
    try
    {
        CheckErrorGCommand(
            GCmdT(m_galil, cmd.c_str(), buffer, G_SMALL_BUFFER, NULL));

        response = BufferToGCStringOut(buffer, G_SMALL_BUFFER);
    }
    catch (GReturn rc)
    {
        CMN_LOG_CLASS_RUN_ERROR << "Error for command \\" << cmd << "\\" << std::endl;
        CMN_LOG_CLASS_RUN_ERROR << "Galil error code: " << rc << std::endl;
        cmnThrow(ExcpSystemError(std::string("SendCommand:  Error sending command--") + std::to_string(rc)), CMN_LOG_LOD_RUN_ERROR);

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

int GalilControllerInterface::SendCommandInt(const std::string &cmd)
{
    int response;
    if (!m_galil)
    {

        cmnThrow(ExcpSystemError("SendCommandInt: ( No Controller Handle = Not Connected )"));
    }
    CMN_LOG_CLASS_RUN_DEBUG << "Sending to Galil [" << cmd << "]" << std::endl;

    try
    {
        CheckErrorGCommand(
            GCmdI(m_galil, cmd.c_str(), &response));
    }
    catch (GReturn rc)
    {
        CMN_LOG_CLASS_RUN_ERROR << "Error for command \\" << cmd << "\\" << std::endl;
        CMN_LOG_CLASS_RUN_ERROR << "Galil error code: " << rc << std::endl;
        cmnThrow(ExcpSystemError(std::string("SendCommand:  Error sending command--") + std::to_string(rc)), CMN_LOG_LOD_RUN_ERROR);

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

double GalilControllerInterface::SendCommandDouble(const std::string &cmd)
{
    double response;
    if (!m_galil)
    {

        cmnThrow(ExcpSystemError("SendCommandInt: ( No Controller Handle = Not Connected )"));
    }
    CMN_LOG_CLASS_RUN_DEBUG << "Sending to Galil [" << cmd << "]" << std::endl;

    try
    {
        CheckErrorGCommand(
            GCmdD(m_galil, cmd.c_str(), &response));
    }
    catch (GReturn rc)
    {
        CMN_LOG_CLASS_RUN_ERROR << "Error for command \\" << cmd << "\\" << std::endl;
        CMN_LOG_CLASS_RUN_ERROR << "Galil error code: " << rc << std::endl;
        cmnThrow(ExcpSystemError(std::string("SendCommand:  Error sending command--") + std::to_string(rc)), CMN_LOG_LOD_RUN_ERROR);

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
void GalilControllerInterface::CreateCommand(char *buffer,
                                             const char *galilCmd,
                                             const vctBoolVec &mask,
                                             const vctDoubleVec &cmdParam)
{

    long bufpos;
    bufpos = sprintf(buffer, "%s", galilCmd);

    unsigned int ii;
    for (ii = 0; ii < mask.size() - 1; ii++)
    {
        if (mask[ii])
            bufpos += sprintf(buffer + bufpos, "%f,", cmdParam[ii]);
        else
            bufpos += sprintf(buffer + bufpos, ",");
    }
    if (mask[ii])
    {
        sprintf(buffer + bufpos, "%f", cmdParam[cmdParam.size() - 1]);
        // else don't add anything so it is not set.
    }
}

// Helper functions for formatting the string commands sent to the controller.
void GalilControllerInterface::CreateCommandLong(char *buffer,
                                                 const char *galilCmd,
                                                 const vctBoolVec &mask,
                                                 const vctDoubleVec &cmdParam)
{

    long bufpos;
    bufpos = sprintf(buffer, "%s", galilCmd);
    unsigned int ii;
    for (ii = 0; ii < mask.size() - 1; ii++)
    {
        if (mask[ii])
            bufpos += sprintf(buffer + bufpos, "%ld,", (long)cmdParam[ii]);
        else
            bufpos += sprintf(buffer + bufpos, ",");
    }
    if (mask[ii])
    {
        sprintf(buffer + bufpos, "%ld", (long)cmdParam[cmdParam.size() - 1]);
        // else don't add anything so it is not set.
    }
}

void GalilControllerInterface::CreateCommandForAxis(char *buffer,
                                                    const char *galilCmd,
                                                    const vctBoolVec &mask)
{
    //"CMD ABCDEFGH"
    // ascii A is 65
    // Note a blank mask calls the command for all axis. (has some undesirable side effects for ST command)
    long bufpos;
    bufpos = sprintf(buffer, "%s ", galilCmd);
    unsigned int ii;
    for (ii = 0; ii < mask.size(); ii++)
    {
        if (mask[ii])
        {
            bufpos += sprintf(buffer + bufpos, "%c", 'A' + ii); // A+next char
        }
    }
}

GDataRecord GalilControllerInterface::RecordData(const DataRecordMethod &method)
{
    GDataRecord record;
    // DR -> Non-blocking asynchronous | QR -> Query-based
    GRecord(
        m_galil,
        &record,
        method == DataRecordMethod::QR ? G_QR : G_DR);

    return record;
}

// This should be never used.
void GalilControllerInterface::Reset()
{
    this->SendCommand("RS");
}

void GalilControllerInterface::ProgramUploadFile(const std::string &filepath)
{
    GProgramUploadFile(m_galil, filepath.c_str());
}

void GalilControllerInterface::SetEncoderCountConversionFactors(const prmMaskedDoubleVec& conversionFactors)
{
    for (size_t i = 0; i < m_EncoderCountsPerUnit.size(); i++)
    {
        // set only conversion factors that have a mask
        if (!conversionFactors.Mask()[i])
            continue;

        m_EncoderCountsPerUnit[i] = conversionFactors.Data()[i];
    }
}

prmMaskedDoubleVec GalilControllerInterface::ConvertAxisUnitToEncoderCounts(const prmMaskedDoubleVec &axisUnits) const
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

prmMaskedIntVec GalilControllerInterface::ConvertAxisUnitToEncoderCountsRounded(const prmMaskedDoubleVec &axisUnits) const
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

prmMaskedDoubleVec GalilControllerInterface::ConvertEncoderCountsToAxisUnit(const prmMaskedDoubleVec &encoderCounts) const
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

prmMaskedDoubleVec GalilControllerInterface::ConvertEncoderCountsToAxisUnit(const prmMaskedIntVec &encoderCounts) const
{
    prmMaskedDoubleVec encoderCountsDbl(encoderCounts.Data().size());

    for (size_t i = 0; i < encoderCountsDbl.Data().size(); i++)
    {
        encoderCountsDbl.Data()[i] = (double)encoderCounts.Data()[i];
        encoderCountsDbl.Mask()[i] = encoderCounts.Mask()[i];
    }

    return ConvertEncoderCountsToAxisUnit(encoderCountsDbl);
}

/* ================= DEMO INTERFACE ===============================*/

DemoGalilControllerInterface::DemoGalilControllerInterface() : GalilControllerInterface()
{
}

bool DemoGalilControllerInterface::Init(const std::string &deviceName)
{
    std::cout << "Hello Demo Galil from: " << deviceName << std::endl;
    CMN_LOG_CLASS_RUN_VERBOSE << "Hello Demo Galil from: " << deviceName << std::endl;

    GalilControllerInterfaceFactory::s_instances.insert({deviceName, std::shared_ptr<DemoGalilControllerInterface>(this)});

    return true;
}

void DemoGalilControllerInterface::Close()
{
    std::cout << "Closing Demo Galil Controller" << std::endl;
    CMN_LOG_CLASS_RUN_VERBOSE << "Closing Demo Galil Controller" << std::endl;
}

std::string DemoGalilControllerInterface::SendCommand(const std::string &command)
{
    CMN_LOG_CLASS_RUN_DEBUG << "Galil command sent: [" << command << "]" << std::endl;

    return "DEMO RESPONSE";
}