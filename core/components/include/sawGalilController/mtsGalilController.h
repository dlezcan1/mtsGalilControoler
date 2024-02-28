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

#ifndef _mtsGalilContoller_h
#define _mtsGalilContoller_h

#include <string>
#include <vector>

#include <gclib.h>
#include <gclibo.h>

#include <cisstVector/vctDynamicVectorTypes.h>
#include <cisstParameterTypes/prmActuatorState.h>
#include <cisstParameterTypes/prmMaskedVector.h>
#include <cisstMultiTask/mtsTaskPeriodic.h>

// Always include last
#include <sawGalilController/sawGalilControllerExport.h>

class CISST_EXPORT mtsGalilController : public mtsTaskPeriodic
{
    CMN_DECLARE_SERVICES(CMN_DYNAMIC_CREATION_ONEARG, CMN_LOG_LOD_RUN_ERROR)

public:
    mtsGalilController(const std::string &componentName, double period_secs);
    mtsGalilController(const mtsTaskPeriodicConstructorArg &arg);

    ~mtsGalilController();

    // CISST MultiTask functions
    void Configure(const std::string &fileName) override;
    void Startup(void) override;
    void Run(void) override;
    void Cleanup(void) override;

    // Galil Controller Functions
    void GetConnected(bool &val) const { val = (m_Galil != 0); }

    void SetTimeout(const double &timeout, bool &success);

    inline void WaitMotion(const vctBoolVec &mask) { WaitMotion(mask, m_timeout); }
    inline void StopMovement(const vctBoolVec &mask) { StopMovement(mask, m_timeout); }

    // Initialize (e.g., establish communications with the controller)
    // and configure the robot.  Note that a configuration file name could
    // be added as a parameter.
    virtual void Close();

    // Enable/disable motor power for all axes
    void EnableAllMotorPower();
    void DisableAllMotorPower();

    // specify the axes.
    void EnableMotorPower(const vctBoolVec &mask);
    // turn power off to the motors, run stopmotion command first just in case
    // otherwise it is not possible to turn off motors when running?
    void DisableMotorPower(const vctBoolVec &mask);

    // Stop robot motion (do not disable motor power)
    void StopMotionAll();
    void StopMotion(const vctBoolVec &mask);

    // the mask is used as command mask, where bool=true homes that axes in the vector
    void Home(const vctBoolVec &mask);
    void UnHome(const vctBoolVec &mask);

    // Abort robot command
    void AbortProgram();
    void AbortMotion();

    // Reset robot, don't use this too often
    void Reset();

    //////---- Getters for most of the actuator state   -----//////
    // This returns the state of the actuators and the controller
    // the way this is implemented only works on 1800 level controllers
    //*** Position feedback (all positions are in counts and are relative to
    //     the robot world coordinate system, unless specified otherwise)
    //*** Motion parameters, speeds are in counts/sec, accelerations/decelerations are
    //     in counts/sec**2
    void GetActuatorState(prmActuatorState &state);

    void GetAnalogInputs(vctDoubleVec &ain) const;
    // Disha-encoder
    void GetToolZEncoder(int &toolZencoder) const;

    // The expected  values are in counts.
    // Set the desired motion goals.
    // this sets the desired actuator goal, also sets the velocity
    void SetPositionMove(const prmMaskedDoubleVec &goalPosition);

    // this sets the velocity rather then position
    void SetVelocityMove(const prmMaskedDoubleVec &goalVelocity);

    // Set the parameters used in motion commands.
    // The velocity is used in position moves

    // both of the following are used in position and move requests.
    void SetSpeed(const prmMaskedDoubleVec &speed);
    void SetAcceleration(const prmMaskedDoubleVec &acceleration);
    void SetDeceleration(const prmMaskedDoubleVec &deceleration);

    // this is used to set the reference position, or reset the absolute position
    // it unsets the home variable at the same time.
    void SetAbsolutePosition(const prmMaskedDoubleVec &position);

    //*** Other functions:
    // Wait for all motion to be complete, with a timeout in seconds.
    void StopMovement(const vctBoolVec &mask, double timeout = 60);
    // Wait for all motion to be complete, with a timeout in seconds.
    void WaitMotion(const vctBoolVec &mask, double timeout = 60);

    //*** Low-level functions that have been made public for use with the IRE:
    // Send command to Galil controller and return pointer to response buffer.
    inline void SendCommand(const std::string& cmd) { SendCommandString(cmd); }
    inline void SendCommandRet(const std::string& cmd, std::string &ret) { ret = SendCommandString(cmd); }
    std::string SendCommandString(const std::string& cmd);
    /* Sends a command knowing that the return value will be a int */
    int         SendCommandInt(const std::string& cmd);
    /* Sends a command knowing that the return value will be a double */
    double      SendCommandDouble(const std::string& cmd);

    void ProgramUploadFile(const std::string &filepath);

    enum MotionMode
    {
        MICROMOTIONMODE,
        MACROMOTIONMODE,
        SMOOTHMOTIONMODE
    };

    // change the pid parameters by loading different
    void GetMotionMode(unsigned int &mode) const { mode = m_MotionMode; }
    void SetMotionMode(const unsigned int &mode) { m_MotionMode = mode; }

    // Method to record values via QR/DR packets
    enum DataRecordMethod
    {
        QR,
        DR
    };
    GDataRecord RecordData(const DataRecordMethod &method = DataRecordMethod::QR);

    vctDoubleVec GetEncoderCountConversionFactors() const { return m_EncoderCountsPerUnit; }
    void SetEncoderCountConversionFactors(const vctDoubleVec &conversionFactors)
    {
        assert(conversionFactors.size() == m_EncoderCountsPerUnit.size());
        m_EncoderCountsPerUnit = conversionFactors;
    }
    void SetEncoderCountConversionFactors(const prmMaskedDoubleVec &conversionFactors);

    prmMaskedIntVec    ConvertAxisUnitToEncoderCountsRounded(const prmMaskedDoubleVec &axisUnits) const;
    vctIntVec          ConvertAxisUnitToEncoderCountsRounded(const vctDoubleVec &axisUnits) const
    {
        return ConvertAxisUnitToEncoderCountsRounded(prmMaskedDoubleVec(axisUnits, vctBoolVec(axisUnits.size(), true))).Data();
    }
    prmMaskedDoubleVec ConvertAxisUnitToEncoderCounts(const prmMaskedDoubleVec &axisUnits) const; 
    vctDoubleVec       ConvertAxisUnitToEncoderCounts(const vctDoubleVec &axisUnits) const
    {
        return ConvertAxisUnitToEncoderCounts(prmMaskedDoubleVec(axisUnits, vctBoolVec(axisUnits.size(), true))).Data();
    }
    
    prmMaskedDoubleVec ConvertEncoderCountsToAxisUnit(const prmMaskedDoubleVec &encoderCounts) const;
    prmMaskedDoubleVec ConvertEncoderCountsToAxisUnit(const prmMaskedIntVec &encoderCounts) const;
    vctDoubleVec       ConvertEncoderCountsToAxisUnit(const vctDoubleVec &encoderCounts) const 
    {
        return ConvertEncoderCountsToAxisUnit(prmMaskedDoubleVec(encoderCounts, vctBoolVec(encoderCounts.size(), true))).Data();
    }
    vctDoubleVec       ConvertEncoderCountsToAxisUnit(const vctIntVec &encoderCounts) const 
    {
        return ConvertEncoderCountsToAxisUnit(prmMaskedIntVec(encoderCounts, vctBoolVec(encoderCounts.size(), true))).Data();
    }

protected:
    void Init(void);
    void SetupInterfaces(void);

    // Double format. TK1.3,2.3,233.2323,,233.2,
    void CreateCommand(char *buffer, const char *galilCmd, const vctBoolVec &mask, const vctDoubleVec &cmdParam);
    // prints in Long format. PT1,2,233,,23,
    void CreateCommandLong(char *buffer, const char *galilCmd, const vctBoolVec &mask, const vctDoubleVec &cmdParam);
    // If the mask is empty (all masks=false) then command will apply to all axes!
    // This is the default behavior for Galil controller commands.
    // use this to set any variable (doubles are converted to long if needed automatically)
    void CreateCommandForAxis(char *buffer, const char *galilCmd, const vctBoolVec &mask);

    // converts commanded values to the actual galil controller mappings
    inline size_t GetNumberActuators() { return m_AxisToGalilChannelMappings.size(); }
    unsigned int RemapAxisIndex(const unsigned int index);
    unsigned int RemapGalilIndex(const unsigned int index, bool& valid);
    vctBoolVec RemapAxisMask(const vctBoolVec& axisMask);
    template <typename T> prmMaskedVector<T> RemapAxisValues(const vctDynamicVector<T>& axisValues);
    template <typename T> prmMaskedVector<T> RemapAxisValues(const prmMaskedVector<T>& axisValues);

    // Galil Controller containers
    //helper command variables in accessing data record elements
    std::vector <std::string> TP; // pos
    std::vector <std::string> TV; // vel (filtered) counts/sec
    std::vector <std::string> BG; // axis moving
    std::vector <std::string> HM; // home
    std::vector <std::string> LF; // for limit
    std::vector <std::string> LR; // rev limit
    std::vector <std::string> MO; // Motor off
    std::vector <std::string> SC; // stop code
    std::vector <std::string> ZA; // user variable   can be used to store state of home
    std::vector <std::string> AN; // Analog input 1 per axis

    // Disha-encoder
    std::vector<std::string> DI;

    // Component fields
    mtsStateTable m_StateTable;
    prmActuatorState m_ActuatorState;

    float m_timeout = 60.0 * cmn_s;

private:
    static inline void CheckErrorGCommand(GReturn rc) { if (rc != G_NO_ERROR) throw rc; }

    void ConnectToGalilController(const std::string& deviceName);

    //////----- Motion commands   -----//////
    //  (all positions are in COUNTS and are relative to the ACTUATOR HOME position).
    // All the mtsVectors are going to be the size of the MAX_
    vctBoolVec   m_IsHomed;  // TRUE if actuator IsHomed
    vctDoubleVec m_AnalogInput;

    // Disha-encoder
    vctIntVec        m_DigitalInput;
    cmnInt           m_DecPosition;
    cmnUInt          m_MotionMode; //indicates the type of config file loaded;
    vctDoubleVec     m_EncoderCountsPerUnit; // The encoder counts per unit (m, mm, rads, revolutions, etc.)
    vctUIntVec       m_AxisToGalilChannelMappings;
    prmMaskedUIntVec m_GalilToAxisChannelMappings;

    // galil controller class handle
    GCon         m_Galil;
    std::string  m_DeviceName;

    // DMC program to download to Galil on startup
    std::string   mDmcFile;

    // internal variable used to calculate velocity times.
    double            m_ServoLoopTime;
    double            m_TMVelocityMultiplier;
    GDataRecord       m_dataRecord;

    // internal is moving state
    vctBoolVec m_SoftRevLimitHit;
    vctBoolVec m_SoftFwdLimitHit;
    size_t     m_NumberEncoderPins;

};

CMN_DECLARE_SERVICES_INSTANTIATION(mtsGalilController)

#endif
