#ifndef _mtsGalilContoller_h
#define _mtsGalilContoller_h

#include <memory>

#include <cisstCommon/cmnUnits.h>
#include <cisstMultiTask.h>

#include "mtsGalilController/GalilControllerInterface.h"

class CISST_EXPORT mtsGalilController : public mtsTaskPeriodic
{
    CMN_DECLARE_SERVICES(CMN_NO_DYNAMIC_CREATION, CMN_LOG_LOD_RUN_ERROR);

    public:
        mtsGalilController(const std::string & componentName, double period_secs);
        mtsGalilController(const mtsTaskPeriodicConstructorArg & arg);

        ~mtsGalilController() {}

        void Configure(const std::string & fileName) override;
        void Startup(void) override;
        void Run(void) override;
        void Cleanup(void) override;

        inline void SendCommand(const mtsStdString& command) { m_galilController->SendCommand(command.Data); }
        void SetTimeout(const mtsDouble& timeout, mtsBool& success); 

        inline void WaitMotion(const mtsBoolVec& mask) { m_galilController->WaitMotion(mask, m_timeout); }
        inline void StopMovement(const mtsBoolVec& mask) { m_galilController->StopMovement(mask, m_timeout); }

    protected:
        void Init(void);
        void SetupInterfaces(void);
        
        mtsStateTable        m_StateTable;

        prmActuatorState m_ActuatorState;

        float m_timeout = 60.0 * cmn_s; 

    private:
        GalilControllerInterface* m_galilController;

}; // class: mtsGalilControllerSensorFeedback_Templated

CMN_DECLARE_SERVICES_INSTANTIATION(mtsGalilController);

#endif