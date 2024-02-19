#ifndef _GalilControllerInterface_h
#define _GalilControllerInterface_h

#include <gclib.h>
#include <gclibo.h>

#include <cisstCommon.h>
#include <cisstParameterTypes/prmActuatorState.h>
#include <cisstParameterTypes/prmMaskedVector.h>
#include <cisstParameterTypes/prmMotionBase.h>
#include <cisstMultiTask/mtsVector.h>
#include <cisstMultiTask.h>

#include <vector>
#include <string>
#include <sstream>
#include <iostream>
#include <stdexcept>
#include <unordered_map>
#include <memory>



// Conditional compilation used because SWIG cannot handle nested class definitions
class CISST_EXPORT GalilControllerInterface : public cmnGenericObject
{
    #ifndef SWIG
        CMN_DECLARE_SERVICES(CMN_NO_DYNAMIC_CREATION, CMN_LOG_LOD_RUN_ERROR);
    #else 
        CMN_DECLARE_SERVICES(CMN_NO_DYNAMIC_CREATION, 5);
    #endif

public:
    class RobotException : public std::runtime_error {
    public:
        explicit RobotException(const std::string& msg = "") : std::runtime_error(msg) {
            //CMN_LOG(1) << "RobotException: " << msg << std::endl;
        }
    };

    class ExcpPowerOff : public RobotException {
    public:
        explicit ExcpPowerOff(const std::string& msg = "Motor Power is Off") : RobotException(msg) { }
    };

    class ExcpAmpError : public RobotException {
    public:
        explicit ExcpAmpError(const std::string& msg = "Amplifier Error") : RobotException(msg) { }
    };

    class ExcpMotorOff : public RobotException {
    public:
        explicit ExcpMotorOff(const std::string& msg = "Motor is Disabled") : RobotException(msg) { }
    };

    class ExcpNotHomed : public RobotException {
    public:
        explicit ExcpNotHomed(const std::string& msg = "Robot is not Homed") : RobotException(msg) { }
    };

    class ExcpLimitError : public RobotException {
    public:
        explicit ExcpLimitError(const std::string& msg = "Out of Limits") : RobotException(msg) { }
    };

    class ExcpSystemError : public RobotException {
    public:
        explicit ExcpSystemError(const std::string& msg = "System Error") : RobotException(msg) { }
    };

    class ExcpWaitMotion : public RobotException {
    public:
        explicit ExcpWaitMotion(const std::string& msg = "Error waiting for robot to stop") : RobotException(msg) { }
    };

    class ExcpRangeError : public RobotException {
    public:
        explicit ExcpRangeError(const std::string& msg = "Motion parameter (speed, accel, or decel) out of range") : RobotException(msg) { }
    };

    // Last axis is used for the break, so its acutally 7 total.    
    const static int NumberActuators = 6;

    // Default Constructor
    GalilControllerInterface();
    // Copy constructor
    GalilControllerInterface( const GalilControllerInterface& controller );
    // Destructor
    ~GalilControllerInterface();

    // Get Interface
    static GalilControllerInterface* GetControllerInterface(const std::string& deviceName);

    // Initialize (e.g., establish communications with the controller)
    // and configure the robot.  Note that a configuration file name could
    // be added as a parameter.
    virtual bool Init(const std::string & deviceName);
    virtual void Close() CISST_THROW( ExcpSystemError );
    // Enable/disable motor power for all axes
    void EnableAllMotorPower() throw( ExcpPowerOff, ExcpSystemError );
    void DisableAllMotorPower() CISST_THROW( ExcpSystemError );

    //specify the axes.
    void EnableMotorPower( const mtsBoolVec &mask ) CISST_THROW(ExcpSystemError);
    //turn power off to the motors, run stopmotion command first just in case 
    //otherwise it is not possible to turn off motors when running?
    void DisableMotorPower( const mtsBoolVec &mask ) CISST_THROW( ExcpSystemError );

    // Stop robot motion (do not disable motor power)
    void StopMotionAll() CISST_THROW( ExcpSystemError );
    void StopMotion( const mtsBoolVec &mask ) CISST_THROW( ExcpSystemError );


    //the mask is used as command mask, where bool=true homes that axes in the vector
    void Home( const mtsBoolVec &mask ) CISST_THROW( RobotException );
    void UnHome( const mtsBoolVec &mask ) CISST_THROW( ExcpSystemError );

    // Abort robot command
    void AbortProgram();
    void AbortMotion();

    // Reset robot, don't use this too often
    void Reset(); 

    //////---- Getters for most of the actuator state   -----//////
    //This returns the state of the actuators and the controller
    //the way this is implemented only works on 1800 level controllers
    //*** Position feedback (all positions are in counts and are relative to
    //    the robot world coordinate system, unless specified otherwise)
    //*** Motion parameters, speeds are in counts/sec, accelerations/decelerations are
    //    in counts/sec**2
    void GetActuatorState( prmActuatorState &state ) throw(ExcpSystemError, RobotException);

    void GetAnalogInputs(mtsDoubleVec &ain) const;
    //Disha-encoder
    void GetToolZEncoder(mtsInt &toolZencoder) const;

    //The expected  values are in counts.
    //Set the desired motion goals.
    //this sets the desired actuator goal, also sets the velocity
    void SetPositionMove( const prmMaskedDoubleVec &goalPosition) throw( ExcpSystemError, RobotException);
    
    //this sets the velocity rather then position       
    void SetVelocityMove( const prmMaskedDoubleVec &goalVelocity) throw( ExcpSystemError, RobotException);
    
    //Set the parameters used in motion commands.
    //The velocity is used in position moves  
    
    //both of the following are used in position and move requests.
    void SetSpeed( const prmMaskedDoubleVec &speed) throw( ExcpSystemError, RobotException);
    void SetAcceleration( const  prmMaskedDoubleVec &acceleration) throw( ExcpSystemError, RobotException);
    void SetDeceleration( const prmMaskedDoubleVec &deceleration) throw( ExcpSystemError, RobotException);
    
    //this is used to set the reference position, or reset the absolute position
    //it unsets the home variable at the same time.
    void SetAbsolutePosition( const prmMaskedDoubleVec &position) throw( ExcpSystemError, RobotException);

    //*** Other functions:
    // Wait for all motion to be complete, with a timeout in seconds.
    void StopMovement( const mtsBoolVec &mask , double timeout=60 ) throw( ExcpWaitMotion, ExcpSystemError);
    // Wait for all motion to be complete, with a timeout in seconds.
    void WaitMotion( const mtsBoolVec &mask , double timeout=60 ) throw( ExcpWaitMotion, ExcpSystemError);

    //*** Low-level functions that have been made public for use with the IRE:
    // Send command to Galil controller and return pointer to response buffer.
    virtual std::string SendCommand( const std::string& cmd )       CISST_THROW (ExcpSystemError);
    
    /* Sends a command knowing that the return value will be a int */
    virtual int         SendCommandInt( const std::string& cmd )    CISST_THROW (ExcpSystemError);
    /* Sends a command knowing that the return value will be a double */
    virtual double      SendCommandDouble( const std::string& cmd ) CISST_THROW (ExcpSystemError);

    void ProgramUploadFile( const std::string& filepath ) CISST_THROW (ExcpSystemError);

    enum MotionMode{MICROMOTIONMODE, MACROMOTIONMODE, SMOOTHMOTIONMODE};
    
    // change the pid parameters by loading different 
    void GetMotionMode(mtsUInt &mode) const {mode=m_MotionMode;}
    void SetMotionMode(const mtsUInt &mode) {m_MotionMode=mode;}

    // Method to record values via QR/DR packets
    enum DataRecordMethod { QR, DR };
    GDataRecord RecordData(const DataRecordMethod& method = DataRecordMethod::QR);

    mtsDoubleVec GetEncoderCountConversionFactors() const { return m_EncoderCountsPerUnit; }
    void         SetEncoderCountConversionFactors(const mtsDoubleVec& conversionFactors) { assert(conversionFactors.size() == m_EncoderCountsPerUnit.size()); m_EncoderCountsPerUnit = conversionFactors;}
    void         SetEncoderCountConversionFactors(const prmMaskedDoubleVec& conversionFactors);
    
    prmMaskedIntVec    ConvertAxisUnitToEncoderCountsRounded(const prmMaskedDoubleVec& axisUnits) const;
    prmMaskedDoubleVec ConvertAxisUnitToEncoderCounts(const prmMaskedDoubleVec& axisUnits) const;
    prmMaskedDoubleVec ConvertEncoderCountsToAxisUnit(const prmMaskedDoubleVec& encoderCounts) const;
    prmMaskedDoubleVec ConvertEncoderCountsToAxisUnit(const prmMaskedIntVec& encoderCounts) const;


    /*	
    inline void GetActuatorParameters(mtsVector<prmGalilActuatorParameters> &actuatorParameters) const CISST_THROW() 
    {
        actuatorParameters=this->ActuatorParameters;
    }
    */

protected:
    //Double format. TK1.3,2.3,233.2323,,233.2,
    void CreateCommand(char *buffer, const char *galilCmd,  const vctBoolVec &mask, const vctDoubleVec &cmdParam);
    
    // prints in Long format. PT1,2,233,,23,
    void CreateCommandLong(char *buffer, const char *galilCmd,  const vctBoolVec &mask, const vctDoubleVec &cmdParam);
    
    // If the mask is empty (all masks=false) then command will apply to all axes! 
    // This is the default behavior for Galil controller commands.
    // use this to set any variable (doubles are converted to long if needed automatically)
    void CreateCommandForAxis(char *buffer, const char *galilCmd, const vctBoolVec &mask);


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

private: 
    static inline void CheckErrorGCommand(GReturn rc) throw(GReturn) { if (rc != G_NO_ERROR) throw rc; } 
    static GCStringOut BufferToGCStringOut(char* buffer, unsigned int buffer_size);
    
    // Internal functions
    // Read configuration file (configuration file name could be added as
    // a parameter)
    void ReadConfigFile(const std::string &fileName ) CISST_THROW(ExcpSystemError);

    // Check status and raise exception if not valid.
    // callerName -- name of calling function (for debug purposes)
    // statusMask -- mask to indicate which status fields to check
    // Returns:  the robot status
    long VerifyStatus(const char *callerName, long statusMask);

    mtsDoubleVec m_EncoderCountsPerUnit; // The encoder counts per unit (m, mm, rads, revolutions, etc.)

    //////-----	Motion commands   -----//////
    //  (all positions are in COUNTS and are relative to the ACTUATOR HOME position).
    // All the mtsVectors are going to be the size of the MAX_
    mtsBoolVec	 m_IsHomed;  // TRUE if actuator IsHomed
    mtsDoubleVec m_AnalogInput;

    // Disha-encoder
    mtsIntVec m_DigitalInput;
    mtsInt    m_DecPosition;
    mtsUInt   m_MotionMode; //indicates the type of config file loaded;

    // galil controller class handle
    GCon m_galil;
    
    // internal variable used to calculate velocity times.
    double            m_ServoLoopTime;
    double            m_TMVelocityMultiplier;
    GDataRecord       m_dataRecord;
    
    // internal is moving state
    vctBoolVec m_SoftRevLimitHit;
    vctBoolVec m_SoftFwdLimitHit;
    size_t     m_NumberEncoderPins;


};

CMN_DECLARE_SERVICES_INSTANTIATION(GalilControllerInterface)

class CISST_EXPORT DemoGalilControllerInterface : public GalilControllerInterface
{
public:
    DemoGalilControllerInterface();
    DemoGalilControllerInterface( const DemoGalilControllerInterface& controller ) = default;
    DemoGalilControllerInterface( const GalilControllerInterface& controller ) : GalilControllerInterface(controller) {}
    ~DemoGalilControllerInterface() {}

    bool Init( const std::string& deviceName ) override;
    void Close() override;

    std::string SendCommand( const std::string& command) override;
    double      SendCommandDouble( const std::string& command) override { SendCommand(command); return 0.0; }
    int         SendCommandInt( const std::string& command) override { SendCommand(command); return 0; }

}; // class: DemoGalilControllerInterface

CMN_DECLARE_SERVICES_INSTANTIATION(DemoGalilControllerInterface);


class GalilControllerInterfaceFactory {
public:

    enum Interface
    {
        DEMO = 0,
        CONTROLLER
    };

    static std::shared_ptr<GalilControllerInterface> GetControllerInterface(const std::string& deviceName, const Interface& interface)
    {
        if (s_instances.count(deviceName) > 0)
            return s_instances[deviceName];

        std::shared_ptr<GalilControllerInterface> gc;
        switch (interface)
        {
            case Interface::DEMO:
                gc = std::make_shared<DemoGalilControllerInterface>();
                break;

            case Interface::CONTROLLER:
                gc = std::make_shared<GalilControllerInterface>();
                break;

            default:
                throw std::invalid_argument("Galil controller Interface: is not implemented!");
        }    
        

        gc->Init(deviceName);

        return gc;
    }


protected:
    friend class GalilControllerInterface;
    friend class DemoGalilControllerInterface;
    
    static std::unordered_map<std::string, std::shared_ptr<GalilControllerInterface>> s_instances;

}; // class: GalilControllerInterfaceFactory

#endif