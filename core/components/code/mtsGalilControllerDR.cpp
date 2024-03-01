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

#include <gclib.h>
#include <gclibo.h>

#include <cisstCommon/cmnPath.h>
#include <cisstCommon/cmnAssert.h>

#include <sawGalilController/mtsGalilControllerDR.h>

//****** Axis Data structures in DR packet ******

#pragma pack(push, 1)     // Eliminate structure padding

// AxisDataMin supported by all Galil DMC controllers
//   - GDataRecord4000 (DMC 4000, 4200, 4103, and 500x0)
//   - GDataRecord52000 (DMC 52000)
//   - GDataRecord1806 (DMC 1806)
//   - GDataRecord2103 (DMC 2103)
//   - GDataRecord1802 (DMC 1802)
//   - GDataRecord30000 (DMC 30010)
struct AxisDataMin {
    uint16_t status;
    uint8_t  switches;
    uint8_t  stop_code;
    int32_t  ref_pos;
    int32_t  pos;
    int32_t  pos_error;
    int32_t  aux_pos;
    int32_t  vel;
    int32_t  torque;
    uint16_t analog_in;   // reserved for 1802
};

// AxisDataMax supported by:
//   - GDataRecord4000 (DMC 4000, 4200, 4103, and 500x0)
//   - GDataRecord52000 (DMC 52000)
//   - GDataRecord30000 (DMC 30010)
struct AxisDataMax : public AxisDataMin {
    uint8_t  hall;        // reserved for 1806
    uint8_t  reserved;
    int32_t  var;         // User-defined (ZA)
};

#pragma pack(pop)

// Bit masks for AxisData fields
// For a full list, see Galil User Manual

const uint16_t StatusMotorMoving     = 0x8000;
const uint16_t StatusFindEdgeActive  = 0x1000;
const uint16_t StatusFindIndexActive = 0x0200;
const uint16_t StatusMotorOff        = 0x0001;
const uint8_t  SwitchFwdLimit        = 0x08;
const uint8_t  SwitchRevLimit        = 0x04;
const uint8_t  SwitchHome            = 0x02;

// Following is information specific to the different Galil DMC controller models.
// There currently are 6 different DMC model types. We do not support any RIO controllers.
// Note also the Galil QZ command, which returns information about the DR structure.
const size_t NUM_MODELS = 6;
const size_t ADmin = sizeof(AxisDataMin);
const size_t ADmax = sizeof(AxisDataMax);
// The Galil model types (corresponding to the different GDataRecord structs)
const unsigned int ModelTypes[NUM_MODELS]     = {  4000, 52000,  1806,  2103,  1802, 30000 };
// Byte offset to the start of the axis data
const unsigned int AxisDataOffset[NUM_MODELS] = {    82,    82,    78 ,   44,    40,    38 };
// Size of the axis data
const size_t AxisDataSize[NUM_MODELS]         = { ADmax, ADmax, ADmin, ADmin, ADmin, ADmax };
// Whether the first 4 bytes contain header information
// Galil does not document the contents of the header. For at least one DMC-4143, the header
// bytes are: 135 (0x87), 15 (0xf), 226 (0xe2), 0
const bool HasHeader[NUM_MODELS]              = {  true,  true, false,  true, false,  true };
// Byte offset to the sample number
const unsigned int SampleOffset[NUM_MODELS]   = {     4,     4,     0,     4,     0,     4 };
// Byte offset to the error code
const unsigned int ErrorCodeOffset[NUM_MODELS] = {   50,    50,    46,    26,    22,    10 };

CMN_IMPLEMENT_SERVICES_DERIVED_ONEARG(mtsGalilControllerDR, mtsTaskContinuous, mtsTaskContinuousConstructorArg)

mtsGalilControllerDR::mtsGalilControllerDR(const std::string &name, unsigned int sizeStateTable, bool newThread) :
    mtsTaskContinuous(name, sizeStateTable, newThread), mGalil(0), mHeader(0), mMotorPowerOn(false), mMotionActive(false)
{
    Init();
}

mtsGalilControllerDR::mtsGalilControllerDR(const mtsTaskContinuousConstructorArg & arg) :
    mtsTaskContinuous(arg), mGalil(0), mHeader(0), mMotorPowerOn(false), mMotionActive(false)
{
    Init();
}

mtsGalilControllerDR::~mtsGalilControllerDR()
{
    Close();
}

void mtsGalilControllerDR::Init(void)
{
    // Call SetupInterfaces after Configure, for reasons documented below
    // (see comment at end of Configure method).
}

void mtsGalilControllerDR::SetupInterfaces(void)
{
    StateTable.AddData(mHeader, "dr_header");
    StateTable.AddData(mSampleNum, "sample_num");
    StateTable.AddData(mErrorCode, "error_code");
    StateTable.AddData(m_measured_js, "measured_js");
    StateTable.AddData(m_setpoint_js, "setpoint_js");
    StateTable.AddData(m_op_state, "op_state");
    StateTable.AddData(mAxisStatus, "axis_status");
    StateTable.AddData(mStopCode, "stop_code");
    StateTable.AddData(mSwitches, "switches");
    StateTable.AddData(mAnalogIn, "analog_in");

    mInterface = AddInterfaceProvided("control");
    if (mInterface) {
        // for Status, Warning and Error with mtsMessage
        mInterface->AddMessageEvents();

        // Standard CRTK interfaces
        mInterface->AddCommandReadState(this->StateTable, m_measured_js, "measured_js");
        mInterface->AddCommandReadState(this->StateTable, m_setpoint_js, "setpoint_js");
        mInterface->AddCommandReadState(this->StateTable, m_op_state, "operating_state");  // TODO
        mInterface->AddCommandWrite(&mtsGalilControllerDR::servo_jp, this, "servo_jp");
        mInterface->AddCommandWrite(&mtsGalilControllerDR::servo_jr, this, "servo_jr");
        mInterface->AddCommandWrite(&mtsGalilControllerDR::servo_jv, this, "servo_jv");
        mInterface->AddCommandVoid(&mtsGalilControllerDR::hold, this, "hold");
        mInterface->AddCommandRead(&mtsGalilControllerDR::GetConfig_js, this, "configuration_js");

        mInterface->AddCommandVoid(&mtsGalilControllerDR::EnableMotorPower, this, "EnableMotorPower");
        mInterface->AddCommandVoid(&mtsGalilControllerDR::DisableMotorPower, this, "DisableMotorPower");

        // TEMP: following is to be able to use prmStateRobotQtWidgetComponent
        mInterface->AddCommandRead(&mtsGalilControllerDR::measured_cp, this, "measured_cp");

        // Stats
        mInterface->AddCommandReadState(StateTable, StateTable.PeriodStats, "period_statistics");

        // Extra stuff
        mInterface->AddCommandRead(&mtsGalilControllerDR::GetNumAxes, this, "GetNumAxes");
        mInterface->AddCommandRead(&mtsGalilControllerDR::GetHeader, this, "GetHeader");
        mInterface->AddCommandReadState(this->StateTable, mSampleNum, "GetSampleNum");
        mInterface->AddCommandReadState(this->StateTable, mErrorCode, "GetErrorCode");
        mInterface->AddCommandRead(&mtsGalilControllerDR::GetConnected, this, "GetConnected");
        mInterface->AddCommandWrite(&mtsGalilControllerDR::SendCommand, this, "SendCommand");
        mInterface->AddCommandWriteReturn(&mtsGalilControllerDR::SendCommandRet, this, "SendCommandRet");
        mInterface->AddCommandReadState(this->StateTable, mAnalogIn, "GetAnalogInput");
        mInterface->AddCommandVoid(&mtsGalilControllerDR::AbortProgram, this, "AbortProgram");
        mInterface->AddCommandVoid(&mtsGalilControllerDR::AbortMotion, this, "AbortMotion");
        mInterface->AddCommandWrite(&mtsGalilControllerDR::SetSpeed, this, "SetSpeed");
        mInterface->AddCommandWrite(&mtsGalilControllerDR::SetAccel, this, "SetAccel");
        mInterface->AddCommandWrite(&mtsGalilControllerDR::SetDecel, this, "SetDecel");
        mInterface->AddCommandWrite(&mtsGalilControllerDR::SetAbsolutePosition, this, "SetAbsolutePosition");
        // Low-level axis data for testing
        mInterface->AddCommandReadState(this->StateTable, mAxisStatus, "GetAxisStatus");
        mInterface->AddCommandReadState(this->StateTable, mStopCode, "GetStopCode");
        mInterface->AddCommandReadState(this->StateTable, mSwitches, "GetSwitches");
    }
}

void mtsGalilControllerDR::Close()
{

    if (mGalil) {
        GClose(mGalil);
        mGalil = 0;
    }
}

void mtsGalilControllerDR::Configure(const std::string& fileName)
{
    std::string dmcStartupFile;

    std::ifstream jsonStream;
    jsonStream.open(fileName.c_str());
    Json::Value jsonConfig;
    Json::Reader jsonReader;
    if (!jsonReader.parse(jsonStream, jsonConfig)) {
        CMN_LOG_CLASS_INIT_ERROR << "Configure: failed to parse " << fileName << " for Galil config" << std::endl
                                 << jsonReader.getFormattedErrorMessages();
        return;
    }

    const Json::Value jsonVersion = jsonConfig["FileVersion"];
    if (jsonVersion.empty()) {
        CMN_LOG_CLASS_INIT_ERROR << "Error: no \"FileVersion\" field in JSON file: " << fileName << std::endl;
        return;
    }
    unsigned int version = jsonVersion.asUInt();
    if (version != 1) {
        CMN_LOG_CLASS_INIT_ERROR << "Error: unsupported JSON FileVersion: " << version << std::endl;
        return;
    }

    const Json::Value ipAddr = jsonConfig["IP_Address"];
    if (ipAddr.empty()) {
        CMN_LOG_CLASS_INIT_ERROR << "Configure: no \"IP_Address\" field in JSON file: " << fileName << std::endl;
        return;
    }
    mDeviceName = ipAddr.asString();
    CMN_LOG_CLASS_INIT_VERBOSE << "Configure: setting IP address " << mDeviceName << std::endl;

    mDirectMode = jsonConfig.get("Galil_Direct", false).asBool();
    if (mDirectMode) {
        CMN_LOG_CLASS_INIT_VERBOSE << "Configure: setting Galil direct mode" << std::endl;
    }

    unsigned int modelType = jsonConfig.get("Galil_Model", 4000).asUInt();
    unsigned int i;
    for (i = 0; i < NUM_MODELS; i++) {
        if (modelType == ModelTypes[i]) {
            mModel = i;
            break;
        }
    }
    if (i == NUM_MODELS) {
        CMN_LOG_CLASS_INIT_ERROR << "Configure: invalid Galil model type " << modelType << std::endl;
        return;
    }
    CMN_LOG_CLASS_INIT_VERBOSE << "Configure: setting Galil model to " << modelType
                               << " (index = " << mModel << ")" << std::endl;

    mDR_Period_ms = jsonConfig.get("DR_Period_ms", 2).asUInt();
    CMN_LOG_CLASS_INIT_VERBOSE << "Configure: setting DR period to " << mDR_Period_ms << " ms" << std::endl;

    // Get axes array
    const Json::Value axesArray = jsonConfig["Axes"];
    if (axesArray.empty()) {
        CMN_LOG_CLASS_INIT_ERROR << "Configure: cannot find \"Axes\" in " << fileName << std::endl;
        return;
    }

    // Size of array determines number of axes
    mNumAxes = axesArray.size();
    CMN_LOG_CLASS_INIT_VERBOSE << "Configure: found " << mNumAxes << " axes" << std::endl;

    // Now, set the data sizes
    m_config_j.Name().SetSize(mNumAxes);
    m_config_j.Type().SetSize(mNumAxes);
    // We have position, velocity and effort for measured_js
    m_measured_js.SetSize(mNumAxes);
    m_measured_js.Position().SetAll(0.0);
    m_measured_js.Velocity().SetAll(0.0);
    m_measured_js.Effort().SetAll(0.0);
    // We have only position for setpoint_js
    m_setpoint_js.Name().SetSize(mNumAxes);
    m_setpoint_js.Position().SetSize(mNumAxes);
    m_setpoint_js.Position().SetAll(0.);

    mAxisToGalilChannelMap.SetSize(mNumAxes);
    mGalilChannelToAxisMap.SetSize(GALIL_MAX_AXES);
    mGalilChannelToAxisMap.SetAll(mNumAxes);   // Initialize to invalid value
    mEncoderCountsPerUnit.SetSize(mNumAxes);
    mAxisStatus.SetSize(mNumAxes);
    mStopCode.SetSize(mNumAxes);
    mSwitches.SetSize(mNumAxes);
    mAnalogIn.SetSize(mNumAxes);

    mGalilIndexMax = 0;
    for (i = 0; i < GALIL_MAX_AXES; i++)
        mGalilIndexValid[i] = false;

    for (Json::ArrayIndex axis = 0; axis < axesArray.size(); axis++) {
        const Json::Value curAxis = axesArray[axis];
        const Json::Value galilIndexJson = curAxis["Galil_Channel"];
        if (galilIndexJson.empty()) {
            CMN_LOG_CLASS_INIT_ERROR << "Configure: missing \"Galil_Channel\" for axis " << axis << std::endl;
            return;
        }
        if (!galilIndexJson.isNumeric()) {
            CMN_LOG_CLASS_INIT_ERROR << "Configure: \"Galil_Channel\" for axis " << axis
                                     << " is not a numeric value" << std::endl;
            return;
        }
        unsigned int galilIndex = galilIndexJson.asUInt();
        if (galilIndex >= GALIL_MAX_AXES) {
            CMN_LOG_CLASS_INIT_ERROR << "Configure: \"Galil_Channel\" for axis " << axis
                                     << " must be 0-" << (GALIL_MAX_AXES-1) << std::endl;
            return;
        }
        mGalilIndexValid[axis] = true;
        mAxisToGalilChannelMap[axis] = galilIndex;
        mGalilChannelToAxisMap[galilIndex] = axis;
        char galilChannel = 'A'+galilIndex;
        if (galilIndex > mGalilIndexMax)
            mGalilIndexMax = galilIndex;   // Save largest GalilIndex for future efficiency
        m_measured_js.Name()[axis].assign(1, galilChannel);
        m_setpoint_js.Name()[axis].assign(1, galilChannel);
        m_config_j.Name()[axis].assign(1, galilChannel);
        m_config_j.Type()[axis] = PRM_JOINT_PRISMATIC;
        mEncoderCountsPerUnit[axis] = curAxis.get("Encoder_Conversion", 1.0).asDouble();
    }
    mGalilIndexMax++;   // Increment so that we can test for less than

    unsigned int k = 0;
    for (i = 0; i < mGalilIndexMax; i++) {
        // If valid axis, add to mGalilAxes
        if (mGalilIndexValid[i])
            mGalilAxes[k++] = 'A'+i;
    }
    mGalilAxes[k] = 0;           // NULL termination

    // Galil Startup Program
    if (jsonConfig.isMember("DMC_Startup_Program")) {
        mDmcFile = jsonConfig["DMC_Startup_Program"].asString();
    }

    // Call SetupInterfaces after Configure because we need to know the correct sizes of
    // the dynamic vectors, which are based on the number of configured axes.
    // These sizes should be set before calling StateTable.AddData and AddCommandReadState;
    // in the latter case, this ensures that the argument prototype has the correct size.
    SetupInterfaces();
}

void mtsGalilControllerDR::Startup()
{
    std::string GalilString = mDeviceName;
    if (mDirectMode) {
        GalilString.append(" -d");
    }
    GalilString.append(" -s DR");  // Subscribe to DR records
    GReturn ret = GOpen(GalilString.c_str(), &mGalil);
    if (ret != G_NO_ERROR) {
        mInterface->SendError(this->GetName() + ": error opening " + mDeviceName);
        CMN_LOG_CLASS_INIT_ERROR << "Galil GOpen: error opening " << mDeviceName
                                 << ": " << ret << std::endl;
        return;
    }

    // Upload a DMC program file if available
    if (!mDmcFile.empty()) {
        if (cmnPath::Exists(mDmcFile)) {
            GProgramUploadFile(mGalil, mDmcFile.c_str());
        }
        else {
            CMN_LOG_CLASS_INIT_ERROR << "Configure: DMC program file \""
                                     << mDmcFile << "\" not found" << std::endl;
        }
    }
    ret = GRecordRate(mGalil, mDR_Period_ms);
    if (ret != G_NO_ERROR) {
        CMN_LOG_CLASS_INIT_ERROR << "Galil GRecordRate: error " << ret << " setting rate to "
                                 << mDR_Period_ms << " ms" << std::endl;
        // Close connection so we do not hang waiting for data
        Close();
    }
}

void mtsGalilControllerDR::Run()
{
    GDataRecord gRec;
    GReturn ret;

    // Get the Galil data record (DR) and parse it
    if (mGalil) {
        ret = GRecord(mGalil, &gRec, G_DR);
        if (ret == G_NO_ERROR) {
            // First 4 bytes are header (for most controllers)
            if (HasHeader[mModel])
                mHeader = *reinterpret_cast<uint32_t *>(gRec.byte_array);
            // Controller sample number
            mSampleNum = *reinterpret_cast<uint16_t *>(gRec.byte_array + SampleOffset[mModel]);
            mErrorCode = gRec.byte_array[ErrorCodeOffset[mModel]];
            // Get the axis data
            // Since we currently do not care about the last 3 entries (in AxisDataMax), we
            // just cast to AxisDataMin and handle the different offsets.
            bool isAnyMoving = false;
            bool isAllMotorOn = true;
            bool isAllMotorOff = true;
            for (size_t i = 0; i < mNumAxes; i++) {
                unsigned int galilAxis = mAxisToGalilChannelMap[i];
                AxisDataMin *axisPtr = reinterpret_cast<AxisDataMin *>(gRec.byte_array +
                                                                       AxisDataOffset[mModel] +
                                                                       galilAxis*AxisDataSize[mModel]);
                m_measured_js.Position()[i] = mEncoderCountsPerUnit[i] * axisPtr->pos;
                m_measured_js.Velocity()[i] = mEncoderCountsPerUnit[i] * axisPtr->vel;
                m_measured_js.Effort()[i] = (axisPtr->torque*9.9982)/32767.0;  // See Galil TT command
                m_setpoint_js.Position()[i] = mEncoderCountsPerUnit[i] * axisPtr->ref_pos;
                mAxisStatus[i] = axisPtr->status;     // See Galil User Manual
                mStopCode[i] = axisPtr->stop_code;    // See Galil SC command
                mSwitches[i] = axisPtr->switches;     // See Galil User Manual
                mAnalogIn[i] = axisPtr->analog_in;
                if (mAxisStatus[i] & StatusMotorMoving)
                    isAnyMoving = true;
                if (mAxisStatus[i] & StatusMotorOff)
                    isAllMotorOn = false;
                else
                    isAllMotorOff = false;
            }
            if (!isAllMotorOn && !isAllMotorOff) {
                // If a mix of on/off motors, turn them all off
                mInterface->SendWarning(this->GetName() + ": turning off all motors");
                DisableMotorPower();
                isAllMotorOn = false;
                isAllMotorOff = true;
            }
            mMotionActive = isAnyMoving;
            mMotorPowerOn = isAllMotorOn;
            m_op_state.SetState(mMotorPowerOn ? prmOperatingState::ENABLED : prmOperatingState::DISABLED);
            m_op_state.SetIsBusy(mMotionActive);
        }
        else {
            mMotionActive = false;
            mMotorPowerOn = false;
            m_op_state.SetState(prmOperatingState::FAULT);
            m_op_state.SetIsBusy(false);
            char buf[128];
            sprintf(buf, ": GRecord error %d", ret);
            mInterface->SendError(this->GetName() + buf);
        }
    }

    // Advance the state table now, so that any connected components can get
    // the latest data.
    StateTable.Advance();

    // Call any connected components
    RunEvent();

    ProcessQueuedCommands();
}

void mtsGalilControllerDR::Cleanup(){
    Close();
}

// Returns command, followed by list of axes (e.g., "BG ABC")
char *mtsGalilControllerDR::GetCmdAxesBuffer(const char *cmd, const char *axes)
{
    static char buf[3+GALIL_MAX_AXES+1];
    CMN_ASSERT((strlen(cmd) <= 3) && (strlen(axes) <= GALIL_MAX_AXES));
    strcpy(buf, cmd);
    strcat(buf, axes);
    return buf;
}

// Returns command, followed by list of values (e.g., "SP 1000,,500")
char *mtsGalilControllerDR::GetCmdValuesBuffer(const char *cmd, int32_t *data, bool *valid, unsigned int num)
{
    static char Cmd_buffer[G_SMALL_BUFFER];

    strcpy(Cmd_buffer, cmd);
    size_t Cmd_len = strlen(Cmd_buffer);
    for (unsigned int i = 0; i < num; i++) {
        if (valid[i]) {
            sprintf(Cmd_buffer+Cmd_len, "%ld,", data[i]);
            Cmd_len = strlen(Cmd_buffer);
        }
        else {
            Cmd_buffer[Cmd_len++] = ',';
            Cmd_buffer[Cmd_len] = 0;
        }
    }
    // Remove last comma
    Cmd_buffer[Cmd_len-1] = 0;

    return Cmd_buffer;
}

void mtsGalilControllerDR::SendCommand(const std::string &cmdString)
{
    if (mGalil) {
        GReturn ret = GCmd(mGalil, cmdString.c_str());
        if (ret != G_NO_ERROR) {
            CMN_LOG_CLASS_RUN_ERROR << "SendCommand: error " << ret << " sending " << cmdString << std::endl;
        }
    }
}

void mtsGalilControllerDR::SendCommandRet(const std::string &cmdString, std::string &retString)
{
    if (mGalil) {
        char buffer[G_SMALL_BUFFER];
        char *firstChar;
        GReturn ret = GCmdT(mGalil, cmdString.c_str(), buffer, G_SMALL_BUFFER, &firstChar);
        if (ret == G_NO_ERROR) {
            retString.assign(firstChar);
        }
        else {
            retString.clear();
            CMN_LOG_CLASS_RUN_ERROR << "SendCommandRet: error " << ret << " sending " << cmdString << std::endl;
        }
    }
}

// Enable motor power
void mtsGalilControllerDR::EnableMotorPower(void)
{
    SendCommand(GetCmdAxesBuffer("SH ", mGalilAxes));
}

// Disable motor power
void mtsGalilControllerDR::DisableMotorPower(void)
{
    SendCommand(GetCmdAxesBuffer("MO ", mGalilAxes));
}

void mtsGalilControllerDR::AbortProgram()
{
    SendCommand("AB");
}

void mtsGalilControllerDR::AbortMotion()
{
    SendCommand("AB 1");
}

bool mtsGalilControllerDR::galil_cmd_common(const char *cmdName, const char *cmdGalil,
                                            const vctDoubleVec &data, const vctDoubleVec &conv)
{
    if (!mGalil)
        return false;

    if (data.size() != mNumAxes) {
        mInterface->SendError(this->GetName() + ": size mismatch in " + std::string(cmdName));
        CMN_LOG_CLASS_RUN_ERROR << cmdName << ": size mismatch (data size = " << data.size()
                                << ", num_axes = " << mNumAxes << ")" << std::endl;
        return false;
    }

    int32_t galilData[GALIL_MAX_AXES];
    size_t i;
    for (i = 0; i < mNumAxes; i++) {
        unsigned int galilIndex = mAxisToGalilChannelMap[i];
        galilData[galilIndex] = static_cast<int32_t>(data[i]/conv[i]);
    }

    SendCommand(GetCmdValuesBuffer(cmdGalil, galilData, mGalilIndexValid, mGalilIndexMax));
    return true;
}

void mtsGalilControllerDR::servo_jp(const prmPositionJointSet &jtpos)
{
    // Stop motion if active
    if (mMotionActive)
        SendCommand(GetCmdAxesBuffer("ST ", mGalilAxes));
    if (galil_cmd_common("servo_jp", "PA ", jtpos.Goal(), mEncoderCountsPerUnit))
        SendCommand(GetCmdAxesBuffer("BG ", mGalilAxes));
}

void mtsGalilControllerDR::servo_jr(const prmPositionJointSet &jtpos)
{
    // Stop motion if active
    if (mMotionActive)
        SendCommand(GetCmdAxesBuffer("ST ", mGalilAxes));
    if (galil_cmd_common("servo_jr", "PR ", jtpos.Goal(), mEncoderCountsPerUnit))
        SendCommand(GetCmdAxesBuffer("BG ", mGalilAxes));
}

void mtsGalilControllerDR::servo_jv(const prmVelocityJointSet &jtvel)
{
    galil_cmd_common("servo_jv", "JG ", jtvel.Goal(), mEncoderCountsPerUnit);
}

void mtsGalilControllerDR::hold(void)
{
    SendCommand(GetCmdAxesBuffer("ST ", mGalilAxes));
}

void mtsGalilControllerDR::SetSpeed(const vctDoubleVec &spd)
{
    galil_cmd_common("SetSpeed", "SP ", spd, mEncoderCountsPerUnit);
}

void mtsGalilControllerDR::SetAccel(const vctDoubleVec &accel)
{
    galil_cmd_common("SetAccel", "AC ", accel, mEncoderCountsPerUnit);
}

void mtsGalilControllerDR::SetDecel(const vctDoubleVec &decel)
{
    galil_cmd_common("SetDecel", "DC ", decel, mEncoderCountsPerUnit);
}

void mtsGalilControllerDR::SetAbsolutePosition(const vctDoubleVec &pos)
{
    galil_cmd_common("SetAbsolutePosition", "DP ", pos, mEncoderCountsPerUnit);
}
