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

#include <sawGalilController/mtsGalilControllerDR.h>

const size_t GALIL_MAX_AXES = 8;

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

// Following is information specific to the different Galil DMC controller models.
// There currently are 6 different DMC model types. We do not support any RIO controllers.
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
const bool HasHeader[NUM_MODELS]              = {  true,  true, false,  true, false,  true };
// Byte offset to the sample number
const unsigned int SampleOffset[NUM_MODELS]   = {     4,     4,     0,     4,     0,     4 };

CMN_IMPLEMENT_SERVICES_DERIVED_ONEARG(mtsGalilControllerDR, mtsTaskContinuous, mtsTaskContinuousConstructorArg)

mtsGalilControllerDR::mtsGalilControllerDR(const std::string &name, unsigned int sizeStateTable, bool newThread) :
    mtsTaskContinuous(name, sizeStateTable, newThread), mGalil(0)
{
    Init();
}

mtsGalilControllerDR::mtsGalilControllerDR(const mtsTaskContinuousConstructorArg & arg) :
    mtsTaskContinuous(arg), mGalil(0)
{
    Init();
}

mtsGalilControllerDR::~mtsGalilControllerDR()
{
    Close();
}

void mtsGalilControllerDR::Init(void)
{
    mHeader = 0;
    StateTable.AddData(mHeader, "dr_header");
    StateTable.AddData(mSampleNum, "sample_num");
    StateTable.AddData(m_measured_js, "measured_js");
    StateTable.AddData(m_setpoint_js, "setpoint_js");

    mInterface = AddInterfaceProvided("control");
    if (mInterface) {
        // for Status, Warning and Error with mtsMessage
        mInterface->AddMessageEvents();

        // Standard CRTK interfaces
        mInterface->AddCommandReadState(this->StateTable, m_measured_js, "measured_js");
        mInterface->AddCommandReadState(this->StateTable, m_setpoint_js, "setpoint_js");
        mInterface->AddCommandWrite(&mtsGalilControllerDR::servo_jp, this, "servo_jp");
        mInterface->AddCommandWrite(&mtsGalilControllerDR::servo_jv, this, "servo_jv");

        mInterface->AddCommandVoid(&mtsGalilControllerDR::EnableMotorPower, this, "EnableMotorPower");
        mInterface->AddCommandVoid(&mtsGalilControllerDR::DisableMotorPower, this, "DisableMotorPower");

        // Stats
        mInterface->AddCommandReadState(StateTable, StateTable.PeriodStats, "GetPeriodStatistics");

        // Extra stuff
        mInterface->AddCommandRead(&mtsGalilControllerDR::GetNumAxes, this, "GetNumAxes");
        mInterface->AddCommandRead(&mtsGalilControllerDR::GetHeader, this, "GetHeader");
        mInterface->AddCommandReadState(this->StateTable, mSampleNum, "GetSampleNum");
        mInterface->AddCommandRead(&mtsGalilControllerDR::GetConnected, this, "GetConnected");
		mInterface->AddCommandWrite(&mtsGalilControllerDR::SendCommand, this, "SendCommand");
		mInterface->AddCommandWriteReturn(&mtsGalilControllerDR::SendCommandRet, this, "SendCommandRet");
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
    if (mDirectMode)
        CMN_LOG_CLASS_INIT_VERBOSE << "Configure: setting Galil direct mode" << std::endl;

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

    unsigned int mDR_Period_ms = jsonConfig.get("DR_Period_ms", 2).asUInt();
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
    // We have position, velocity and effort for measured_js
    m_measured_js.SetSize(mNumAxes);
    // We have only position for setpoint_js
    m_setpoint_js.Name().SetSize(mNumAxes);
    m_setpoint_js.Position().SetSize(mNumAxes);

    mAxisToGalilChannelMap.SetSize(mNumAxes);
    mEncoderCountsPerUnit.SetSize(mNumAxes);

    for (Json::ArrayIndex i = 0; i < axesArray.size(); i++) {
        const Json::Value curAxis = axesArray[i];
        const Json::Value galilIndexJson = curAxis["Galil_Channel"];
        if (galilIndexJson.empty()) {
            CMN_LOG_CLASS_INIT_ERROR << "Configure: missing \"Galil_Channel\" for axis " << i << std::endl;
            return;
        }
        if (!galilIndexJson.isNumeric()) {
            CMN_LOG_CLASS_INIT_ERROR << "Configure: \"Galil_Channel\" for axis " << i
                                     << " is not a numeric value" << std::endl;
            return;
        }
        unsigned int galilIndex = galilIndexJson.asUInt();
        if (galilIndex >= GALIL_MAX_AXES) {
            CMN_LOG_CLASS_INIT_ERROR << "Configure: \"Galil_Channel\" for axis " << i
                                     << " must be 0-" << (GALIL_MAX_AXES-1) << std::endl;
            return;
        }
        mAxisToGalilChannelMap[i] = galilIndex;
        char galilChannel = 'A'+galilIndex;
        m_measured_js.Name()[i].assign(1, galilChannel);
        m_setpoint_js.Name()[i].assign(1, galilChannel);
        mEncoderCountsPerUnit[i] = curAxis.get("Encoder_Conversion", 1.0).asDouble();
    }

    // Galil Startup Program
    if (jsonConfig.isMember("DMC_Startup_Program")) {
        dmcStartupFile = jsonConfig["DMC_Startup_Program"].asString();
    }

    // upload a dmc program file if available
    if (dmcStartupFile.length() > 0) {
        if (cmnPath::Exists(dmcStartupFile)) {
            GProgramUploadFile(mGalil, dmcStartupFile.c_str());
        }
        else {
            CMN_LOG_CLASS_INIT_ERROR << "Configure: Invalid DMC program file \""
                                     << dmcStartupFile << "\"" << std::endl;
        }
    }
}

void mtsGalilControllerDR::Startup()
{
    std::string GalilString = mDeviceName;
    if (mDirectMode)
        GalilString.append(" -d");
    GalilString.append(" -s DR");  // Subscribe to DR records
    GReturn ret = GOpen(GalilString.c_str(), &mGalil);
    if (ret != G_NO_ERROR) {
        CMN_LOG_CLASS_INIT_ERROR << "Galil GOpen: error opening " << mDeviceName
                                 << ": " << ret << std::endl;
        return;
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

    // Get the Galil data record (DR) and parse it
    if (mGalil && (GRecord(mGalil, &gRec, G_DR) == G_NO_ERROR)) {
        // First 4 bytes are header (for most controllers)
        if (HasHeader[mModel])
            mHeader = *reinterpret_cast<uint32_t *>(gRec.byte_array);
        // Controller sample number
        mSampleNum = *reinterpret_cast<uint16_t *>(gRec.byte_array + SampleOffset[mModel]);
        // Get the axis data
        // Since we currently do not care about the last 3 entries (in AxisDataMax), we
        // just cast to AxisDataMin and handle the different offsets.
        for (size_t i = 0; i < mNumAxes; i++) {
            unsigned int galilAxis = mAxisToGalilChannelMap[i];
            AxisDataMin *axisPtr = reinterpret_cast<AxisDataMin *>(gRec.byte_array +
                                                                   AxisDataOffset[mModel] +
                                                                   galilAxis*AxisDataSize[mModel]);
            m_measured_js.Position()[i] = mEncoderCountsPerUnit[i] * axisPtr->pos;
            m_measured_js.Velocity()[i] = mEncoderCountsPerUnit[i] * axisPtr->vel;
            m_measured_js.Effort()[i] = axisPtr->torque;
            m_setpoint_js.Position()[i] = mEncoderCountsPerUnit[i] * axisPtr->ref_pos;
        }
    }

    // Call any connected components
    RunEvent();

    ProcessQueuedCommands();
}

void mtsGalilControllerDR::Cleanup(){
    Close();
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
    SendCommand("SH");
}

// Disable motor power
void mtsGalilControllerDR::DisableMotorPower(void)
{
    SendCommand("MO");
}

void mtsGalilControllerDR::servo_common(const char *cmdName, const char *cmdGalil, const vctDoubleVec &goal)
{
    if (!mGalil)
        return;

    size_t i;
    size_t goalAxes = goal.size();
    if (goalAxes != mNumAxes) {
        mInterface->SendWarning(this->GetName() + ": size mismatch in " + std::string(cmdName));
        CMN_LOG_CLASS_RUN_WARNING << cmdName << ": size mismatch (num_goals = " << goalAxes
                                  << ", num_axes = " << mNumAxes << ")" << std::endl;
    }
    // Ignore any extra axes
    if (goalAxes > mNumAxes)
        goalAxes = mNumAxes;

    char Cmd_buffer[G_SMALL_BUFFER];
    char BG_buffer[G_SMALL_BUFFER];

    int32_t galilData[GALIL_MAX_AXES];
    bool galilDataValid[GALIL_MAX_AXES];
    for (i = 0; i < GALIL_MAX_AXES; i++)
        galilDataValid[i] = false;

    unsigned int galilIndexMax = 0;
    for (i = 0; i < goalAxes; i++) {
        unsigned int galilIndex = mAxisToGalilChannelMap[i];
        if (galilIndex > galilIndexMax)
            galilIndexMax = galilIndex;
        galilData[galilIndex] = static_cast<int32_t>(goal[i]/mEncoderCountsPerUnit[i]);
        galilDataValid[galilIndex] = true;
    }
    strcpy(Cmd_buffer, cmdGalil);
    strcpy(BG_buffer, "BG ");
    size_t Cmd_len = strlen(Cmd_buffer);
    size_t BG_len = 3;
    for (i = 0; i < galilIndexMax; i++) {
        if (galilDataValid[i]) {
            sprintf(Cmd_buffer+Cmd_len, "%ld", galilData[i]);
            Cmd_len = strlen(Cmd_buffer);
        }
        if (--goalAxes > 0) {
            Cmd_buffer[Cmd_len++] = ',';
            Cmd_buffer[Cmd_len] = 0;
        }
        BG_buffer[BG_len++] = static_cast<char>('A'+i);
        BG_buffer[BG_len] = 0;
    }
    CMN_LOG_CLASS_RUN_VERBOSE << cmdName << ": sending \"" << Cmd_buffer << "\", followed by \""
                              << BG_buffer << "\" to Galil" << std::endl;
    SendCommand(Cmd_buffer);
    SendCommand(BG_buffer);
}

void mtsGalilControllerDR::servo_jp(const prmPositionJointSet &jtpos)
{
    servo_common("servo_jp", "PA ", jtpos.Goal());
}

void mtsGalilControllerDR::servo_jv(const prmVelocityJointSet &jtvel)
{
    servo_common("servo_jv", "JG ", jtvel.Goal());
}
