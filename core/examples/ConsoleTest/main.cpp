/*-*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-   */
/*ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab:*/

/*
(C) Copyright 2024 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/

#include <cisstCommon/cmnLogger.h>
#include <cisstCommon/cmnKbHit.h>
#include <cisstCommon/cmnGetChar.h>
#include <cisstCommon/cmnConstants.h>
#include <cisstOSAbstraction/osaSleep.h>
#include <cisstMultiTask/mtsManagerLocal.h>
#include <cisstMultiTask/mtsTaskContinuous.h>
#include <cisstMultiTask/mtsInterfaceRequired.h>

#define USE_DR

#ifdef USE_DR
#include <sawGalilController/mtsGalilControllerDR.h>
#else
#include <sawGalilController/mtsGalilController.h>
#endif

class GalilClient : public mtsTaskMain {

private:
    size_t NumAxes;
    vctDoubleVec jtgoal, jtvel;
    vctDoubleVec jtpos;

#ifdef USE_DR
    prmStateJoint m_measured_js;
    prmPositionJointSet jtposSet;
    prmVelocityJointSet jtvelSet;
    prmOperatingState m_op_state;
    uint16_t mSampleNum;
    uint8_t  mErrorCode;
#else
    prmActuatorState m_ActuatorState;
#endif

    // Both
    mtsFunctionRead GetConnected;
    mtsFunctionWrite SendCommand;
    mtsFunctionWriteReturn SendCommandRet;
    mtsFunctionVoid crtk_enable;
    mtsFunctionVoid crtk_disable;
#ifdef USE_DR
    // mtsGalilRobotDR
    mtsFunctionRead measured_js;
    mtsFunctionRead setpoint_js;
    mtsFunctionRead operating_state;
    mtsFunctionWrite servo_jp;
    mtsFunctionWrite servo_jr;
    mtsFunctionWrite servo_jv;
    mtsFunctionVoid hold;
    mtsFunctionRead get_header;
    mtsFunctionRead get_sample_num;
    mtsFunctionRead get_error_code;
    mtsFunctionRead get_status;
    mtsFunctionRead get_stop_code;
    mtsFunctionRead get_switches;
    mtsFunctionRead get_analog;
#else
    // mtsGalilRobot
    mtsFunctionRead GetActuatorState;
#endif

    void OnErrorEvent(const mtsMessage &msg) {
        std::cout << std::endl << "Error: " << msg.Message << std::endl;
    }

public:

    GalilClient() : mtsTaskMain("GalilClient"), NumAxes(0)
    {
        mtsInterfaceRequired *req = AddInterfaceRequired("Input", MTS_OPTIONAL);
        if (req) {
            // Both
            req->AddFunction("GetConnected", GetConnected);
            req->AddFunction("SendCommand", SendCommand);
            req->AddFunction("SendCommandRet", SendCommandRet);
#ifdef USE_DR
            // mtsGalilComponentDR only
            req->AddFunction("measured_js", measured_js);
            req->AddFunction("setpoint_js", setpoint_js);
            req->AddFunction("operating_state", operating_state);
            req->AddFunction("servo_jp", servo_jp);
            req->AddFunction("servo_jr", servo_jr);
            req->AddFunction("servo_jv", servo_jv);
            req->AddFunction("hold", hold);
            req->AddFunction("EnableMotorPower", crtk_enable);
            req->AddFunction("DisableMotorPower", crtk_disable);
            req->AddFunction("GetHeader", get_header);
            req->AddFunction("GetSampleNum", get_sample_num);
            req->AddFunction("GetErrorCode", get_error_code);
            req->AddFunction("GetAxisStatus", get_status);
            req->AddFunction("GetStopCode", get_stop_code);
            req->AddFunction("GetSwitches", get_switches);
            req->AddFunction("GetAnalogInput", get_analog);
#else
            // mtsGalilComponent only
            req->AddFunction("GetActuatorState", GetActuatorState);
            req->AddFunction("EnableAllMotorPower", crtk_enable);
            req->AddFunction("DisableAllMotorPower", crtk_disable);
#endif
            req->AddEventHandlerWrite(&GalilClient::OnErrorEvent, this, "error");
        }
    }

    void Configure(const std::string&) {}

    void PrintHelp()
    {
        std::cout << "Available commands:" << std::endl
#ifdef USE_DR
                  << "  m: position move joints (servo_jp)" << std::endl
                  << "  r: relative move joints (servo_jr)" << std::endl
                  << "  v: velocity move joints (servo_jv)" << std::endl
                  << "  s: stop move (hold)" << std::endl
#endif
                  << "  c: send command" << std::endl
                  << "  h: display help information" << std::endl
                  << "  e: enable motor power" << std::endl
                  << "  n: disable motor power" << std::endl
#ifdef USE_DR
                  << "  a: get analog input" << std::endl
                  << "  o: get operating state" << std::endl
                  << "  i: display header info" << std::endl
#endif
                  << "  q: quit" << std::endl;
    }

    void Startup()
    {
        NumAxes = 0;
#ifdef USE_DR
        const mtsGenericObject *p = measured_js.GetArgumentPrototype();
        const prmStateJoint *psj = dynamic_cast<const prmStateJoint *>(p);
        if (psj) NumAxes = psj->Position().size();
#else
        const mtsGenericObject *p = GetActuatorState.GetArgumentPrototype();
        const prmActuatorState *pas = dynamic_cast<const prmActuatorState *>(p);
        if (pas) NumAxes = pas->Position().size();
#endif
        std::cout << "GalilClient: Detected " << NumAxes << " axes" << std::endl;
        jtpos.SetSize(NumAxes);
        jtgoal.SetSize(NumAxes);
        jtvel.SetSize(NumAxes);
#ifdef USE_DR
        jtposSet.Goal().SetSize(NumAxes);
        jtvelSet.SetSize(NumAxes);
#endif

        PrintHelp();
    }

    void Run() {

        ProcessQueuedEvents();

        bool galilOK;
        GetConnected(galilOK);

        if (galilOK) {
#ifdef USE_DR
            measured_js(m_measured_js);
            m_measured_js.GetPosition(jtpos);
            operating_state(m_op_state);
#else
            GetActuatorState(m_ActuatorState);
            mtsDoubleVec mpos;
            m_ActuatorState.GetPosition(mpos);
            jtpos = mpos;
#endif
        }

        char c = 0;
        size_t i;
        if (cmnKbHit()) {
            c = cmnGetChar();
            switch (c) {

#ifdef USE_DR
            case 'm':   // position move joint
                std::cout << std::endl << "Enter joint positions: ";
                for (i = 0; i < NumAxes; i++)
                    std::cin >> jtgoal[i];
                jtposSet.SetGoal(jtgoal);
                std::cout << "Moving to " << jtgoal << std::endl;
                servo_jp(jtposSet);
                break;

            case 'r':   // relative move joint
                std::cout << std::endl << "Enter relative joint positions: ";
                for (i = 0; i < NumAxes; i++)
                    std::cin >> jtgoal[i];
                jtposSet.SetGoal(jtgoal);
                std::cout << "Relative move by " << jtgoal << std::endl;
                servo_jr(jtposSet);
                break;

            case 'v':   // velocity move joint
                std::cout << std::endl << "Enter joint velocities: ";
                for (i = 0; i < NumAxes; i++)
                    std::cin >> jtvel[i];
                jtvelSet.SetGoal(jtvel);
                servo_jv(jtvelSet);
                break;

            case 's':   // stop move (hold)
                hold();
                break;
#endif

            case 'c':
                if (galilOK) {
                    std::string cmdString;
                    std::string retString;
                    std::cout << std::endl << "Enter command: ";
                    std::cin >> cmdString;
                    SendCommandRet(cmdString, retString);
                    std::cout << "Return: " << retString << std::endl;
                }
                else {
                    std::cout << std::endl << "Command not available - Galil not connected" << std::endl;
                }
                break;
                
            case 'e':   // enable motor power
                crtk_enable();
                break;

            case 'n':   // disable motor power
                crtk_disable();
                break;

            case 'h':
                std::cout << std::endl;
                PrintHelp();
                break;

#ifdef USE_DR
            case 'a':
                {
                    vctUShortVec analog_in;
                    get_analog(analog_in);
                    std::cout << std::endl << "Analog input: " << analog_in << std::endl;
                }
                break;

            case 'o':
                std::cout << std::endl << "Operating state: " << m_op_state << std::endl;
                break;

            case 'i':
                if (get_header.IsValid()) {
                    uint32_t header;
                    get_header(header);
                    std::cout << std::endl << "Header: " << std::hex << header << std::endl;
                }
                break;
#endif

            case 'q':   // quit program
                std::cout << "Exiting.. " << std::endl;
                this->Kill();
                break;

            }
        }

        if (galilOK) {
            size_t i;
#ifdef USE_DR
            get_sample_num(mSampleNum);
            get_error_code(mErrorCode);
            vctUShortVec axStatus;
            vctUCharVec  axStopCode;
            vctUCharVec  axSwitches;
            get_status(axStatus);
            get_stop_code(axStopCode);
            get_switches(axSwitches);
            // Do not know if there is a Galil command to return axis status
            // Bit meanings:
            //   15:  axis in motion
            //    0:  motor off
            // See Galil User Manual for other bits
            printf("st: ");
            for (i = 0; i < axStatus.size(); i++)
                printf("%x ", (int)axStatus[i]);
            // Print Stop Code (matches Galil SC command)
            printf("SC: ");
            for (i = 0; i < axStopCode.size(); i++)
                printf("%d ", (int)axStopCode[i]);
            // Some overlap with Galil TS command, but not the same
            printf("ts: ");
            for (i = 0; i < axSwitches.size(); i++)
                printf("%x ", (int)axSwitches[i]);
            printf("| ");
#endif
            printf("%d (%d) ", (int)mSampleNum, (int)mErrorCode);
            printf("POS: [");
            for (i = 0; i < jtpos.size(); i++)
                printf(" %7.2lf ", jtpos[i]);
#ifdef USE_DR
            printf("] TORQUE: [");
            vctDoubleVec jtt;
            m_measured_js.GetEffort(jtt);
            for (i = 0; i < jtt.size(); i++)
                printf(" %7.2lf ", jtt[i]);
#endif
            printf("]\r");
        }
        else {
            printf("Galil not connected\r");
        }

        osaSleep(0.01);  // to avoid taking too much CPU time
    }

    void Cleanup() {}

};

int main(int argc, char **argv)
{
    cmnLogger::SetMask(CMN_LOG_ALLOW_ALL);
    cmnLogger::SetMaskDefaultLog(CMN_LOG_ALLOW_ALL);
    cmnLogger::SetMaskFunction(CMN_LOG_ALLOW_ALL);
    cmnLogger::AddChannel(std::cout, CMN_LOG_ALLOW_ERRORS_AND_WARNINGS);

    double periodSec = 0.005;
    if (argc < 2) {
#ifdef USE_DR
        std::cout << "Syntax: sawGalilConsole <config>" << std::endl;
        std::cout << "        <config>      Configuration file (JSON format)" << std::endl;
#else
        std::cout << "Syntax: sawGalilConsole <config> [<period>]" << std::endl;
        std::cout << "        <config>      Configuration file (JSON format)" << std::endl;
        std::cout << "        [<period>]    Sampling period, sec (default " << periodSec << ")" << std::endl;
#endif
        return 0;
    }

#ifndef USE_DR
    if (argc > 2) {
        if (sscanf(argv[2], "%lf", &periodSec) != 1) {
            std::cout << "Failed to parse period " << argv[2] << std::endl;
            return -1;
        }
    }
#endif

#ifdef USE_DR
    std::cout << "Starting mtsGalilController with DR interface" << std::endl;
    mtsGalilControllerDR *galilServer;
    galilServer = new mtsGalilControllerDR("galilServer");
#else
    std::cout << "Starting mtsGalilController with period " << periodSec << " sec" << std::endl;
    mtsGalilController *galilServer;
    galilServer = new mtsGalilController("galilServer", periodSec);
#endif
    galilServer->Configure(argv[1]);

    mtsComponentManager * componentManager = mtsComponentManager::GetInstance();
    componentManager->AddComponent(galilServer);

    GalilClient client;
    componentManager->AddComponent(&client);

    if (!componentManager->Connect(client.GetName(), "Input", galilServer->GetName(), "control")) {
        std::cout << "Failed to connect: "
            << client.GetName() << "::Input to "
            << galilServer->GetName() << "::control" << std::endl;
        delete galilServer;
        return -1;
    }

    componentManager->CreateAll();
    componentManager->StartAll();

    // Main thread passed to client task

    galilServer->Kill();
    componentManager->WaitForStateAll(mtsComponentState::FINISHED, 2.0 * cmn_s);

    componentManager->Cleanup();

    // stop all logs
    cmnLogger::SetMask(CMN_LOG_ALLOW_NONE);
    delete galilServer;

    return 0;
}
