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
#include <sawGalilController/mtsGalilController.h>

class GalilClient : public mtsTaskMain {

private:
    prmActuatorState m_ActuatorState;

    mtsFunctionRead GetActuatorState;
    mtsFunctionWrite SendCommand;

public:

    GalilClient() : mtsTaskMain("GalilClient")
    {
        mtsInterfaceRequired *req = AddInterfaceRequired("Input", MTS_OPTIONAL);
        if (req) {
            req->AddFunction("GetActuatorState", GetActuatorState);
            req->AddFunction("SendCommand", SendCommand);
        }
    }

    void Configure(const std::string&) {}

    void PrintHelp()
    {
        std::cout << "Available commands:" << std::endl
                  << "  c: send command" << std::endl
                  << "  h: display help information" << std::endl
                  << "  q: quit" << std::endl;
    }

    void Startup()
    {
        PrintHelp();
    }

    void Run() {

        ProcessQueuedEvents();

        GetActuatorState(m_ActuatorState);

        char c = 0;
        if (cmnKbHit()) {
            c = cmnGetChar();
            switch (c) {

            case 'c':
                {
                std::string cmdString;
                std::cout << std::endl << "Enter command: ";
                std::cin >> cmdString;
                SendCommand(cmdString);
                }
                break;
                
            case 'h':
                std::cout << std::endl;
                PrintHelp();
                break;

            case 'q':   // quit program
                std::cout << "Exiting.. " << std::endl;
                this->Kill();
                break;
            }
        }

        mtsDoubleVec pos;
        m_ActuatorState.GetPosition(pos);
        printf("JOINT POS:   [");
        for (size_t i = 0; i < pos.size(); i++)
            printf(" %7.2lf ", pos[i]);
        printf("]\r");

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
        std::cout << "Syntax: sawGalilConsole <config> [<period>]" << std::endl;
        std::cout << "        <config>      Configuration file (JSON format)" << std::endl;
        std::cout << "        [<period>]    Sampling period, sec (default " << periodSec << ")" << std::endl;
        return 0;
    }
    if (argc > 2) {
        if (sscanf(argv[2], "%lf", &periodSec) != 1) {
            std::cout << "Failed to parse period " << argv[2] << std::endl;
            return -1;
        }
    }
        
    std::cout << "Starting mtsGalilController with period " << periodSec << " sec" << std::endl;
    mtsGalilController *galilServer;
    galilServer = new mtsGalilController("galilServer", periodSec);
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
