/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  (C) Copyright 2024 Johns Hopkins University (JHU), All Rights Reserved.

  This component provides an interface to a Galil DMC controller, using the DR
  (DataRecord) approach, where the Galil controller periodically sends a data
  record. The format of the data record varies based on Galil DMC model type,
  which can be specified (as "Galil_Model") in the JSON configuration file.
  Valid values of "Galil_Model" (which is an unsigned integer) are:

      4000   for DMC 4000, 4200, 4103, and 500x0 (default)
     52000   for DMC 52000
      1806   for DMC 1806
      2103   for DMC 2103
      1802   for DMC 1802
     30000   for DMC 30010

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/

#ifndef _mtsGalilContollerDR_h
#define _mtsGalilContollerDR_h

#include <string>

#include <cisstVector/vctDynamicVectorTypes.h>
#include <cisstMultiTask/mtsTaskContinuous.h>
#include <cisstMultiTask/mtsInterfaceProvided.h>
#include <cisstParameterTypes/prmConfigurationJoint.h>
#include <cisstParameterTypes/prmStateJoint.h>
#include <cisstParameterTypes/prmPositionJointSet.h>
#include <cisstParameterTypes/prmVelocityJointSet.h>
#include <cisstParameterTypes/prmPositionCartesianGet.h>
#include <cisstParameterTypes/prmOperatingState.h>

// Always include last
#include <sawGalilController/sawGalilControllerExport.h>

class CISST_EXPORT mtsGalilControllerDR : public mtsTaskContinuous
{
    CMN_DECLARE_SERVICES(CMN_DYNAMIC_CREATION_ONEARG, CMN_LOG_LOD_RUN_ERROR)

    void         *mGalil;                   // Gcon
    std::string   mDeviceName;              // IP address
    bool          mDirectMode;              // Direct connection (not using gcaps)
    unsigned int  mDR_Period_ms;            // DR period, in milliseconds
    unsigned int  mModel;                   // Galil model (see list of supported models above)
    std::string   mDmcFile;                 // DMC program to download to Galil on startup
    unsigned int  mNumAxes;                 // Number of axes
    unsigned int  mGalilIndexMax;           // Maximum galil channel index
    uint32_t      mHeader;                  // Header bytes in DR packet
    uint16_t      mSampleNum;               // Sample number from controller
    uint8_t       mErrorCode;               // Error code from controller
    prmConfigurationJoint m_config_j;       // Joint configuration
    prmStateJoint m_measured_js;            // Measured joint state (CRTK)
    prmStateJoint m_setpoint_js;            // Setpoint joint state (CRTK)
    prmOperatingState m_op_state;           // Operating state (CRTK)
    vctUIntVec    mAxisToGalilChannelMap;   // Map from axis index to Galil channel
    vctUIntVec    mGalilChannelToAxisMap;   // Map from Galil channel to axis index
    vctDoubleVec  mEncoderCountsPerUnit;    // Encoder conversion factors
    vctUShortVec  mAxisStatus;              // Axis status
    vctUCharVec   mStopCode;                // Axis stop code (see Galil SC command)
    vctUCharVec   mSwitches;                // Axis switches (see Galil TS command)
    vctUShortVec  mAnalogIn;                // Axis analog input
    bool          mMotorPowerOn;            // Whether motor power is on (for all configured motors)
    bool          mMotionActive;            // Whether a motion is active
    mtsInterfaceProvided *mInterface;       // Provided interface

 public:

    mtsGalilControllerDR(const std::string &name, unsigned int sizeStateTable = 1024, bool newThread = true);
    mtsGalilControllerDR(const mtsTaskContinuousConstructorArg &arg);

    ~mtsGalilControllerDR();

    enum { GALIL_MAX_AXES = 8 };

    // cisstMultiTask functions
    void Configure(const std::string &fileName) override;
    void Startup(void) override;
    void Run(void) override;
    void Cleanup(void) override;

protected:

    // String of configured axes (e.g., "ABC")
    char mGalilAxes[GALIL_MAX_AXES+1];
    // Boolean array indicating which Galil indexes are valid
    bool mGalilIndexValid[GALIL_MAX_AXES];

    // Local static method to concatenate cmd (no more than 3 chars, including any spaces)
    // and axes (no more than GALIL_MAX_AXES chars).
    // Parameters:
    //    cmd    Galil command string, including space if desired (e.g, "BG ")
    //    axes   Galil axes string (e.g., "ABC")
    // Example output: "BG ABC"
    static char *GetCmdAxesBuffer(const char *cmd, const char *axes);

    // Local static method to create cmd followed by comma-separated values
    // Parameters:
    //    cmd    Galil command string, including space if desired (e.g, "SP ")
    //    data   Data values (indexed by Galil channel, so valid values may not be contiguous)
    //    valid  Boolean array indicating which data values are valid
    //    num    Size of data and valid arrays
    // Example output: "SP 1000,,500"
    static char *GetCmdValuesBuffer(const char *cmd, int32_t *data, bool *valid, unsigned int num);

    void Init();
    void Close();

    void SetupInterfaces();

    void GetNumAxes(unsigned int &numAxes) const { numAxes = mNumAxes; }
    void GetHeader(uint32_t &header) const { header = mHeader; }
    void GetConnected(bool &val) const { val = (mGalil != 0); }

    void SendCommand(const std::string& cmdString);
    void SendCommandRet(const std::string& cmdString, std::string &retString);

    // Enable motor power
    void EnableMotorPower(void);
    // Disable motor power
    void DisableMotorPower(void);

    // Abort robot command
    void AbortProgram();
    void AbortMotion();

    // Common method for sending command to Galil
    bool galil_cmd_common(const char *cmdName, const char *cmdGalil, const vctDoubleVec &goal,
                          const vctDoubleVec &conv);

    // Move joint to specified position
    void servo_jp(const prmPositionJointSet &jtpos);
    // Move joint to specified relative position
    void servo_jr(const prmPositionJointSet &jtpos);
    // Move joint at specified velocity
    void servo_jv(const prmVelocityJointSet &jtvel);
    // Hold joint at current position (Stop)
    void hold(void);

    // Get joint configuration
    void GetConfig_js(prmConfigurationJoint &cfg_j) const
    { cfg_j = m_config_j; }

    // TEMP: following is to be able to use prmStateRobotQtWidgetComponent
    void measured_cp(prmPositionCartesianGet &pos) const
    { pos = prmPositionCartesianGet(); }

    // Set speed, acceleration and deceleration
    void SetSpeed(const vctDoubleVec &spd);
    void SetAccel(const vctDoubleVec &accel);
    void SetDecel(const vctDoubleVec &decel);

    // Set absolute position (e.g., for homing)
    void SetAbsolutePosition(const vctDoubleVec &pos);
};

CMN_DECLARE_SERVICES_INSTANTIATION(mtsGalilControllerDR)

#endif
