#pragma once

#include "indifocuser.h"

#include <chrono>
 
class WilliamOpticsOwlto : public INDI::Focuser
{
  public:
      WilliamOpticsOwlto();
      virtual ~WilliamOpticsOwlto() override = default;
  
      const char * getDefaultName() override;
      virtual bool initProperties() override;
      virtual bool updateProperties() override;
      // virtual bool ISNewSwitch(const char * dev, const char * name, ISState * states, char * names[], int n) override;
      // virtual bool ISNewNumber(const char * dev, const char * name, double values[], char * names[], int n) override;

      static void timedMoveHelper(void * context);

  protected:
      virtual bool Handshake() override;

      virtual IPState MoveFocuser(FocusDirection dir, int speed, uint16_t duration) override;

      virtual IPState MoveAbsFocuser(uint32_t targetTicks) override;

      virtual IPState MoveRelFocuser(FocusDirection dir, uint32_t ticks) override;

      virtual bool SyncFocuser(uint32_t ticks) override;
      // virtual bool ReverseFocuser(bool enabled) override;
      virtual bool AbortFocuser() override;
      virtual void TimerHit() override;

  private:
      bool Ack();

      /**
         * @brief sendCommand Send a string command to Owlto.
         * @param cmd Command to be sent, must already have the necessary delimeter ('#')
         * @param res If not nullptr, the function will read until it detects the default delimeter ('#') up to ML_RES length.
         *        if nullptr, no read back is done and the function returns true.
         * @return True if successful, false otherwise.
         */
      bool sendCommand(const char * cmd, char * res = nullptr);

      // Get initial focuser parameter when we first connect
      void GetFocusParams();
      // Read and update Step Mode
      bool readStepMode();
      // Read and update Temperature
      bool readTemperature();
      // Read and update Position
      bool readPosition();
      bool readMaxPosition();
      // Read and update speed
      bool readSpeed();
      // Are we moving?
      bool isMoving();

      void timedMoveCallback();

      bool MoveFocuser(uint32_t position);
      double targetPos { 0 }, lastPos { 0 }, lastTemperature { 0 };

      static const uint8_t OWLTO_RES { 32 };
      static const char OWLTO_DEL { '\n' };
      static const uint8_t OWLTO_TIMEOUT { 3 };
};