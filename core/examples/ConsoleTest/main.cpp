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

#include <string>

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
    size_t NumAxes;
    vctDoubleVec jtgoal, jtvel;
    vctDoubleVec jtpos;

    prmStateJoint m_measured_js;
    prmStateJoint m_setpoint_js;
    prmPositionJointSet jtposSet;
    prmVelocityJointSet jtvelSet;
    prmOperatingState m_op_state;
    prmActuatorState m_ActuatorState;
    uint16_t mSampleNum;
    uint8_t  mErrorCode;

    mtsFunctionRead GetConnected;
    mtsFunctionWrite SendCommand;
    mtsFunctionWriteReturn SendCommandRet;
    mtsFunctionVoid crtk_enable;
    mtsFunctionVoid crtk_disable;
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
    mtsFunctionRead GetActuatorState;

    void OnErrorEvent(const mtsMessage &msg) {
        std::cout << std::endl << "Error: " << msg.Message << std::endl;
    }

public:

    GalilClient() : mtsTaskMain("GalilClient"), NumAxes(0)
    {
        mtsInterfaceRequired *req = AddInterfaceRequired("Input", MTS_OPTIONAL);
        if (req) {
            req->AddFunction("GetConnected", GetConnected);
            req->AddFunction("SendCommand", SendCommand);
            req->AddFunction("SendCommandRet", SendCommandRet);
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
            req->AddFunction("GetActuatorState", GetActuatorState);
            req->AddEventHandlerWrite(&GalilClient::OnErrorEvent, this, "error");
        }
    }

    void Configure(const std::string&) {}

    void PrintHelp()
    {
        std::cout << "Available commands:" << std::endl
                  << "  m: position move joints (servo_jp)" << std::endl
                  << "  r: relative move joints (servo_jr)" << std::endl
                  << "  v: velocity move joints (servo_jv)" << std::endl
                  << "  s: stop move (hold)" << std::endl
                  << "  c: send command" << std::endl
                  << "  h: display help information" << std::endl
                  << "  e: enable motor power" << std::endl
                  << "  n: disable motor power" << std::endl
                  << "  a: get analog input" << std::endl
                  << "  o: get operating state" << std::endl
                  << "  i: display header info" << std::endl
                  << "  q: quit" << std::endl;
    }

    void Startup()
    {
        NumAxes = 0;
        const mtsGenericObject *p = measured_js.GetArgumentPrototype();
        const prmStateJoint *psj = dynamic_cast<const prmStateJoint *>(p);
        if (psj) NumAxes = psj->Position().size();
        std::cout << "GalilClient: Detected " << NumAxes << " axes" << std::endl;
        jtpos.SetSize(NumAxes);
        jtgoal.SetSize(NumAxes);
        jtvel.SetSize(NumAxes);
        jtposSet.Goal().SetSize(NumAxes);
        jtvelSet.SetSize(NumAxes);

        PrintHelp();
    }

    void Run() {

        ProcessQueuedEvents();

        bool galilOK;
        GetConnected(galilOK);

        if (galilOK) {
            measured_js(m_measured_js);
            setpoint_js(m_setpoint_js);
            m_measured_js.GetPosition(jtpos);
            operating_state(m_op_state);
            // GetActuatorState(m_ActuatorState);
        }

        char c = 0;
        size_t i;
        if (cmnKbHit()) {
            c = cmnGetChar();
            switch (c) {

            case 'm':   // position move joint
                std::cout << std::endl << "Enter joint positions (mm): ";
                for (i = 0; i < NumAxes; i++)
                    std::cin >> jtgoal[i];
                std::cout << "Moving to " << jtgoal << std::endl;
                jtgoal.Divide(1000.0);   // Convert to SI
                jtposSet.SetGoal(jtgoal);
                servo_jp(jtposSet);
                break;

            case 'r':   // relative move joint
                std::cout << std::endl << "Enter relative joint positions (mm): ";
                for (i = 0; i < NumAxes; i++)
                    std::cin >> jtgoal[i];
                std::cout << "Relative move by " << jtgoal << std::endl;
                jtgoal.Divide(1000.0);   // Convert to SI
                jtposSet.SetGoal(jtgoal);
                servo_jr(jtposSet);
                break;

            case 'v':   // velocity move joint
                std::cout << std::endl << "Enter joint velocities (mm/s): ";
                for (i = 0; i < NumAxes; i++)
                    std::cin >> jtvel[i];
                jtvel.Divide(1000.0);   // Convert to SI
                jtvelSet.SetGoal(jtvel);
                servo_jv(jtvelSet);
                break;

            case 's':   // stop move (hold)
                hold();
                break;

            case 'c':
                if (galilOK) {
                    std::string cmdString;
                    std::string retString;
                    std::cout << std::endl << "Enter command: ";
                    for (;;) {
                        std::getline(std::cin, cmdString);
                        if (!cmdString.empty()) {
                            SendCommandRet(cmdString, retString);
                            std::cout << "Return: " << retString << std::endl;
                            break;
                        }
                    }
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

            case 'q':   // quit program
                std::cout << "Exiting.. " << std::endl;
                this->Kill();
                break;

            }
        }

        if (galilOK) {
            size_t i;
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
            printf("%d (%d) ", (int)mSampleNum, (int)mErrorCode);
            jtpos.Multiply(1000.0);  // Convert to mm for display
            printf("POS: [");
            for (i = 0; i < jtpos.size(); i++)
                printf(" %7.2lf ", jtpos[i]);
            printf("] TORQUE: [");
            vctDoubleVec jtt;
            m_setpoint_js.GetEffort(jtt);
            for (i = 0; i < jtt.size(); i++)
                printf(" %7.2lf ", jtt[i]);
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

    if (argc < 2) {
        std::cout << "Syntax: sawGalilConsole <config>" << std::endl;
        std::cout << "        <config>      Configuration file (JSON format)" << std::endl;
        return 0;
    }

    std::cout << "Starting mtsGalilController" << std::endl;
    mtsGalilController *galilServer;
    galilServer = new mtsGalilController("galilServer");
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
