#include "williamoptics_owlto.h"

#include "indicom.h"

#include <cmath>
#include <cstring>
#include <memory>

#include <termios.h>
#include <unistd.h>
 
static std::unique_ptr<WilliamOpticsOwlto> woOwlto(new WilliamOpticsOwlto());

void ISGetProperties(const char * dev)
{
    woOwlto->ISGetProperties(dev);
}

void ISNewSwitch(const char * dev, const char * name, ISState * states, char * names[], int n)
{
    woOwlto->ISNewSwitch(dev, name, states, names, n);
}

void ISNewText(const char * dev, const char * name, char * texts[], char * names[], int n)
{
    woOwlto->ISNewText(dev, name, texts, names, n);
}

void ISNewNumber(const char * dev, const char * name, double values[], char * names[], int n)
{
    woOwlto->ISNewNumber(dev, name, values, names, n);
}

void ISNewBLOB(const char * dev, const char * name, int sizes[], int blobsizes[], char * blobs[], char * formats[],
               char * names[], int n)
{
    INDI_UNUSED(dev);
    INDI_UNUSED(name);
    INDI_UNUSED(sizes);
    INDI_UNUSED(blobsizes);
    INDI_UNUSED(blobs);
    INDI_UNUSED(formats);
    INDI_UNUSED(names);
    INDI_UNUSED(n);
}

void ISSnoopDevice(XMLEle * root)
{
    woOwlto->ISSnoopDevice(root);
}

const char * WilliamOpticsOwlto::getDefaultName()
{
    return "William Optics Owlto";
}

WilliamOpticsOwlto::WilliamOpticsOwlto()
{
  FI::SetCapability(FOCUSER_CAN_ABS_MOVE | FOCUSER_CAN_REL_MOVE | FOCUSER_CAN_SYNC | FOCUSER_CAN_ABORT);
}

bool WilliamOpticsOwlto::initProperties()
{
  INDI::Focuser::initProperties();

  /* Relative and absolute movement */
    FocusRelPosN[0].min   = 0.;
    FocusRelPosN[0].max   = 50000.;
    FocusRelPosN[0].value = 0;
    FocusRelPosN[0].step  = 1000;

    FocusAbsPosN[0].min   = 0.;
    FocusAbsPosN[0].max   = 300000.;
    FocusAbsPosN[0].value = 0;
    FocusAbsPosN[0].step  = 1000;

    setDefaultPollingPeriod(500);
    addDebugControl();

    return true;
}

bool WilliamOpticsOwlto::updateProperties(){
  if (isConnected()){

    GetFocusParams();
    LOG_INFO("Owlto paramaters updated, focuser ready for use.");
  }else{

  }
  return true;
}

bool WilliamOpticsOwlto::Handshake()
{
    if (Ack())
    {
        LOG_INFO("Owlto is online. Getting focus parameters...");
        return true;
    }

    LOG_INFO(
        "Error retreiving data from Owlto.");
    return false;
}

bool WilliamOpticsOwlto::Ack(){
  char res[OWLTO_RES] = {0};

  if (!sendCommand("P:", res))
  {
    LOG_ERROR("ACK - getPosition failed");
    return false;
  }

  int32_t pos;
  int rc = sscanf(res, "(%d)", &pos);

  if (rc <= 0)
    {
        LOG_ERROR("ACK - getPosition failed");
        return false;
    }
  
  return true;
}

bool WilliamOpticsOwlto::readPosition()
{
    char res[OWLTO_RES] = {0};

    if (!sendCommand("P:", res))
        return false;

    int32_t pos;
    int rc = sscanf(res, "(%d)", &pos);

    if (rc > 0)
        FocusAbsPosN[0].value = pos;
    else
    {
        LOGF_ERROR("Unknown error: focuser position value (%s)", res);
        return false;
    }

    return true;
}

bool WilliamOpticsOwlto::readMaxPosition()
{
    char res[OWLTO_RES] = {0};

    if (sendCommand("MP:", res) == false)
        return false;

    uint32_t steps = 0;
    int rc = sscanf(res, "(%d)", &steps);
    if (rc > 0)
    {
        FocusMaxPosN[0].value = steps;
        FocusMaxPosNP.s = IPS_OK;
    }
    else
    {
        LOGF_ERROR("Unknown error: maximum position value (%s)", res);
        return false;
    }

    return true;
}

bool WilliamOpticsOwlto::isMoving()
{
    char res[OWLTO_RES] = {0};

    if (sendCommand("IM:", res) == false)
        return false;

    if (strcmp(res, "(1)") == 0)
        return true;
    else if (strcmp(res, "(0)") == 0)
        return false;

    LOGF_ERROR("Unknown error: isMoving value (%s)", res);
    return false;
}

bool WilliamOpticsOwlto::SyncFocuser(uint32_t ticks)
{
    char cmd[OWLTO_RES] = {0};
    snprintf(cmd, OWLTO_RES, "SP:%06d", ticks);
    return sendCommand(cmd);
}

// bool WilliamOpticsOwlto::ReverseFocuser(bool enabled)
// {
//     char cmd[OWLTO_RES] = {0};
//     snprintf(cmd, OWLTO_RES, "[SREV%01d]", enabled ? 1 : 0);
//     return sendCommand(cmd);
// }

void WilliamOpticsOwlto::GetFocusParams()
{
    if (readPosition())
        IDSetNumber(&FocusAbsPosNP, nullptr);

    if (readMaxPosition())
        IDSetNumber(&FocusMaxPosNP, nullptr);
}

bool WilliamOpticsOwlto::MoveFocuser(uint32_t position)
{
    char cmd[OWLTO_RES] = {0};
    // char res[OWLTO_RES] = {0};
    snprintf(cmd, OWLTO_RES, "M:%06d", position);
    // Set Position First
    if (sendCommand(cmd) == false)
        return false;

    return true;
}

IPState WilliamOpticsOwlto::MoveFocuser(FocusDirection dir, int speed, uint16_t duration)
{
    INDI_UNUSED(speed);
    // either go all the way in or all the way out
    // then use timer to stop
    if (dir == FOCUS_INWARD)
        MoveFocuser(0);
    else
        MoveFocuser(FocusMaxPosN[0].value);

    IEAddTimer(duration, &WilliamOpticsOwlto::timedMoveHelper, this);
    return IPS_BUSY;
}

void WilliamOpticsOwlto::timedMoveHelper(void * context)
{
    static_cast<WilliamOpticsOwlto*>(context)->timedMoveCallback();
}

void WilliamOpticsOwlto::timedMoveCallback()
{
    AbortFocuser();
    FocusAbsPosNP.s = IPS_IDLE;
    FocusRelPosNP.s = IPS_IDLE;
    FocusTimerNP.s = IPS_IDLE;
    FocusTimerN[0].value = 0;
    // IDSetNumber(&FocusAbsPosNP, nullptr);
    // IDSetNumber(&FocusRelPosNP, nullptr);
    // IDSetNumber(&FocusTimerNP, nullptr);
}


IPState WilliamOpticsOwlto::MoveAbsFocuser(uint32_t targetTicks)
{
    targetPos = targetTicks;

    if (!MoveFocuser(targetPos))
        return IPS_ALERT;

    return IPS_BUSY;
}


IPState WilliamOpticsOwlto::MoveRelFocuser(FocusDirection dir, uint32_t ticks)
{
    int32_t newPosition = 0;

    if (dir == FOCUS_INWARD)
        newPosition = FocusAbsPosN[0].value - ticks;
    else
        newPosition = FocusAbsPosN[0].value + ticks;

    // Clamp
    newPosition = std::max(0, std::min(static_cast<int32_t>(FocusAbsPosN[0].max), newPosition));
    if (!MoveFocuser(newPosition))
        return IPS_ALERT;

    return IPS_BUSY;
}

void WilliamOpticsOwlto::TimerHit()
{
    if (!isConnected())
    {
        SetTimer(POLLMS);
        return;
    }

    bool rc = readPosition();
    if (rc)
    {
        if (fabs(lastPos - FocusAbsPosN[0].value) > 5)
        {
            IDSetNumber(&FocusAbsPosNP, nullptr);
            lastPos = FocusAbsPosN[0].value;
        }
    }

    if (FocusAbsPosNP.s == IPS_BUSY || FocusRelPosNP.s == IPS_BUSY)
    {
        if (!isMoving())
        {
            FocusAbsPosNP.s = IPS_OK;
            FocusRelPosNP.s = IPS_OK;
            IDSetNumber(&FocusAbsPosNP, nullptr);
            IDSetNumber(&FocusRelPosNP, nullptr);
            lastPos = FocusAbsPosN[0].value;
            LOG_INFO("Focuser reached requested position.");
        }
    }
	
    // rc = readTemperature();
    // if (rc)
    // {
    //     if (fabs(lastTemperature - TemperatureN[0].value) >= 0.5)
    //     {
    //         IDSetNumber(&TemperatureNP, nullptr);
    //         lastTemperature = TemperatureN[0].value;
    //     }
    // }

    SetTimer(POLLMS);
}

bool WilliamOpticsOwlto::AbortFocuser()
{
    return sendCommand("A:");
}

bool WilliamOpticsOwlto::sendCommand(const char * cmd, char * res)
{
    int nbytes_written = 0, nbytes_read = 0, rc = -1;

    tcflush(PortFD, TCIOFLUSH);

    LOGF_DEBUG("CMD <%s>", cmd);

    if ((rc = tty_write_string(PortFD, cmd, &nbytes_written)) != TTY_OK)
    {
        char errstr[MAXRBUF] = {0};
        tty_error_msg(rc, errstr, MAXRBUF);
        LOGF_ERROR("Serial write error: %s.", errstr);
        return false;
    }

    if (res == nullptr)
        return true;

    if ((rc = tty_nread_section(PortFD, res, OWLTO_RES, OWLTO_DEL, OWLTO_TIMEOUT, &nbytes_read)) != TTY_OK)
    {
        char errstr[MAXRBUF] = {0};
        tty_error_msg(rc, errstr, MAXRBUF);
        LOGF_ERROR("Serial read error: %s.", errstr);
        return false;
    }

    LOGF_DEBUG("RES <%s>", res);

    tcflush(PortFD, TCIOFLUSH);

    return true;
}