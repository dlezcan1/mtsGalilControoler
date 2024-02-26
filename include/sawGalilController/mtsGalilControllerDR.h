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
#include <cisstParameterTypes/prmStateJoint.h>
#include <cisstParameterTypes/prmPositionJointSet.h>
#include <cisstParameterTypes/prmVelocityJointSet.h>

// Always include last
#include <sawGalilController/sawGalilControllerExport.h>

class CISST_EXPORT mtsGalilControllerDR : public mtsTaskContinuous
{
    CMN_DECLARE_SERVICES(CMN_DYNAMIC_CREATION_ONEARG, CMN_LOG_LOD_RUN_ERROR);

    void         *mGalil;          // Gcon
    std::string   mDeviceName;     // IP address
    bool          mDirectMode;     // Direct connection (not using gcaps)
    unsigned int  mDR_Period_ms;   // DR period, in milliseconds
    unsigned int  mModel;          // Galil model (see list of supported models above)
    std::string   mDmcFile;        // DMC program to download to Galil on startup
    unsigned int  mNumAxes;        // Number of axes
    uint32_t      mHeader;         // Header bytes in DR packet
    uint16_t      mSampleNum;      // Sample number from controller
    prmStateJoint m_measured_js;   // Measured joint state (CRTK)
    prmStateJoint m_setpoint_js;   // Setpoint joint state (CRTK)
    vctUIntVec    mAxisToGalilChannelMap;   // Map from axis index to Galil channel
    vctDoubleVec  mEncoderCountsPerUnit;    // Encoder conversion factors
    mtsInterfaceProvided *mInterface;       // Provided interface

 public:

    mtsGalilControllerDR(const std::string &name, unsigned int sizeStateTable, bool newThread);
    mtsGalilControllerDR(const mtsTaskContinuousConstructorArg &arg);

    ~mtsGalilControllerDR();

    // cisstMultiTask functions
    void Configure(const std::string &fileName) override;
    void Startup(void) override;
    void Run(void) override;
    void Cleanup(void) override;

protected:

    void Init();
    void Close();

    void GetNumAxes(unsigned int &numAxes) const { numAxes = mNumAxes; }
    void GetHeader(uint32_t &header) const { header = mHeader; }
    void GetSampleNum(unsigned int &sampleNum) const { sampleNum = mSampleNum; }
    void GetConnected(bool &val) const { val = (mGalil != 0); }

    void SendCommand(const std::string& cmdString);
    void SendCommandRet(const std::string& cmdString, std::string &retString);

    // Enable motor power
    void EnableMotorPower(void);
    // Disable motor power
    void DisableMotorPower(void);

    // Called by servo_jp and servo_jv
    void servo_common(const char *cmdName, const char *cmdGalil, const vctDoubleVec &goal);

    // Move joint to specified position
    void servo_jp(const prmPositionJointSet &jtpos);
    // Move joint at specified velocity
    void servo_jv(const prmVelocityJointSet &jtvel);
};

CMN_DECLARE_SERVICES_INSTANTIATION(mtsGalilControllerDR)

#endif
