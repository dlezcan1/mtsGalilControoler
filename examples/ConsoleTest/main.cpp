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
    mtsFunctionWrite servo_jp;
    mtsFunctionWrite servo_jv;
    mtsFunctionRead get_header;
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
            req->AddFunction("servo_jp", servo_jp);
            req->AddFunction("servo_jv", servo_jv);
            req->AddFunction("EnableMotorPower", crtk_enable);
            req->AddFunction("DisableMotorPower", crtk_disable);
            req->AddFunction("GetHeader", get_header, MTS_OPTIONAL);
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
                  << "  v: velocity move joints (servo_jv)" << std::endl
#endif
                  << "  c: send command" << std::endl
                  << "  h: display help information" << std::endl
                  << "  e: enable motor power" << std::endl
                  << "  n: disable motor power" << std::endl
#ifdef USE_DR
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
        // Following does not yet work because GetActuatorState is added to
        // provided interface before sizes are set.
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
                std::cout << std::endl << "Moving to " << jtgoal << std::endl;
                servo_jp(jtposSet);
                break;

            case 'v':   // velocity move joint
                std::cout << std::endl << "Enter joint velocities: ";
                for (i = 0; i < NumAxes; i++)
                    std::cin >> jtvel[i];
                jtvelSet.SetGoal(jtvel);
                servo_jv(jtvelSet);
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
            printf("JOINT POS:   [");
            for (size_t i = 0; i < jtpos.size(); i++)
                printf(" %7.2lf ", jtpos[i]);
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
