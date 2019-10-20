/*
    William Optics Owlto Focuser

    Copyright (C) 2019 James Yang

    Based on Moonline driver.
    Copyright (C) 2013-2019 Jasem Mutlaq (mutlaqja@ikarustech.com)

    This library is free software; you can redistribute it and/or
    modify it under the terms of the GNU Lesser General Public
    License as published by the Free Software Foundation; either
    version 2.1 of the License, or (at your option) any later version.

    This library is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
    Lesser General Public License for more details.

    You should have received a copy of the GNU Lesser General Public
    License along with this library; if not, write to the Free Software
    Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA

*/

#include "owltofocus.h"

#include "indicom.h"

#include <cmath>
#include <cstring>
#include <memory>

#include <termios.h>
#include <unistd.h>

static std::unique_ptr<Owlto> owlto(new Owlto());

void ISGetProperties(const char * dev)
{
    owlto->ISGetProperties(dev);
}

void ISNewSwitch(const char * dev, const char * name, ISState * states, char * names[], int n)
{
    owlto->ISNewSwitch(dev, name, states, names, n);
}

void ISNewText(const char * dev, const char * name, char * texts[], char * names[], int n)
{
    owlto->ISNewText(dev, name, texts, names, n);
}

void ISNewNumber(const char * dev, const char * name, double values[], char * names[], int n)
{
    owlto->ISNewNumber(dev, name, values, names, n);
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
    owlto->ISSnoopDevice(root);
}

Owlto::Owlto()
{
    FI::SetCapability(FOCUSER_CAN_ABS_MOVE | FOCUSER_CAN_REL_MOVE | FOCUSER_CAN_SYNC | FOCUSER_CAN_REVERSE | FOCUSER_CAN_ABORT);
}

bool Owlto::initProperties()
{
    INDI::Focuser::initProperties();
    
    IUFillSwitch(&CalibrationS[CALIBRATION_START],"CALIBRATION_START","Start Calibration", ISS_OFF);
    IUFillSwitch(&CalibrationS[CALIBRATION_RESET],"CALIBRATION_RESET","Reset Calibration", ISS_OFF);
    IUFillSwitchVector(&CalibrationSP, CalibrationS, 2, getDeviceName(), "FOCUS_CALIBRATION", "Calibration", MAIN_CONTROL_TAB, IP_RW, ISR_1OFMANY, 0 , IPS_IDLE);

    /* Relative and absolute movement */
    FocusRelPosN[0].min = 0.;
    FocusRelPosN[0].max = 5000.;
    FocusRelPosN[0].value = 0.;
    FocusRelPosN[0].step = 10.;

    FocusAbsPosN[0].min = 0.;
    FocusAbsPosN[0].max = 100000.;
    FocusAbsPosN[0].value = 50000.;
    FocusAbsPosN[0].step = 500.;

    IUFillNumber(&StallguardThresN[0], "STALLGUARD_THRES", "UNIT", "%d", -64, 64, 1, 0);
    IUFillNumberVector(&StallguardThresNP, StallguardThresN, 1, getDeviceName(), "FOCUS_STALLGUARD_THRES","Stallguard Thres", MAIN_CONTROL_TAB, IP_RW, 0, IPS_IDLE);

    IUFillNumber(&MoveCurrentN[0], "MOVE_CURRENT", "32 MAX", "%d", 0, 32, 1, 0);
    IUFillNumberVector(&MoveCurrentNP, MoveCurrentN, 1, getDeviceName(), "FOCUS_MOVE_CURRENT","Move Current", MAIN_CONTROL_TAB, IP_RW, 0, IPS_IDLE);

    setDefaultPollingPeriod(500);
    addDebugControl();

    return true;
}

const char * Owlto::getDefaultName()
{
    return "WO Owlto Focus";
}

bool Owlto::updateProperties()
{
    INDI::Focuser::updateProperties();

    if (isConnected())
    {
        defineSwitch(&CalibrationSP);
        defineNumber(&StallguardThresNP);
        defineNumber(&MoveCurrentNP);

        GetFocusParams();

        LOG_INFO("Owlto Focus paramaters updated, focuser ready for use.");
    }
    else
    {
        deleteProperty(CalibrationSP.name);
        deleteProperty(MoveCurrentNP.name);
    }

    return true;
}

bool Owlto::Handshake()
{
    if (Ack())
    {
        LOG_INFO("Owlto Focus is online. Getting focus parameters...");
        return true;
    }

    LOG_INFO("Error retreiving data from Owlto Focus, please ensure owlto controller is powered and the port is correct.");
    return false;
}

bool Owlto::Ack()
{
    sleep(2);

    char res[OWLTO_RES] = {0};
    if (!sendCommand("<GPOS>", res))
    {
        LOG_ERROR("ACK - getPosition failed");
        return false;
    }

    int32_t pos;
    int rc = sscanf(res, "[%d]", &pos);

    LOGF_INFO("POS:%d", pos);

    if (rc <= 0)
    {
        LOG_ERROR("ACK - getPosition failed");
        return false;
    }

    return true;
}

bool Owlto::readPosition()
{
    char res[OWLTO_RES] = {0};

    if (sendCommand("<GPOS>", res) == false)
        return false;

    int32_t pos;
    int rc = sscanf(res, "[%d]", &pos);

    if (rc > 0)
        FocusAbsPosN[0].value = pos;
    else
    {
        LOGF_ERROR("Unknown error: focuser position value (%s)", res);
        return false;
    }

    return true;
}

bool Owlto::readMaxPosition()
{
    char res[OWLTO_RES] = {0};

    if (sendCommand("<GMXP>", res) == false)
        return false;

    int32_t steps = 0;
    int rc = sscanf(res, "[%d]", &steps);

    if (rc > 0)
    {
        FocusMaxPosN[0].value = steps;
        FocusMaxPosNP.s = IPS_OK;
    }
    else
    {
        LOGF_ERROR("Unknown error: focuser max position value (%s)", res);
        return false;
    }

    return true;
}

bool Owlto::readStallguardThres()
{
    char res[OWLTO_RES] = {0};

    if (sendCommand("<GSGV>", res) == false)
        return false;

    int32_t thres = 0;
    int rc = sscanf(res, "[%d]", &thres);
    if (rc > 0)
    {
        StallguardThresN[0].value = thres;
        StallguardThresNP.s = IPS_OK;
    }
    else
    {
        LOGF_ERROR("Unknown error: focuser SG value (%s)", res);
        return false;
    }

    return true;
}

bool Owlto::isMoving()
{
    char res[OWLTO_RES] = {0};

    if (sendCommand("<GMOV>", res) == false)
        return false;

    if (strcmp(res, "[1]") == 0)
        return true;
    else if (strcmp(res, "[0]") == 0)
        return false;

    LOGF_ERROR("Unknown error: isMoving value (%s)", res);
    return false;
}

bool Owlto::SyncFocuser(uint32_t ticks)
{
    char cmd[OWLTO_RES] = {0};
    snprintf(cmd, OWLTO_RES, "<SPOS%06d>", ticks);
    return sendCommand(cmd);
}

bool Owlto::ReverseFocuser(bool enabled)
{
    char cmd[OWLTO_RES] = {0};
    snprintf(cmd, OWLTO_RES, "<SINV%01d>", enabled ? 1 : 0);
    return sendCommand(cmd);
}

bool Owlto::MoveFocuser(uint32_t position)
{
    char cmd[OWLTO_RES] = {0};
    char res[OWLTO_RES] = {0};
    snprintf(cmd, OWLTO_RES, "<STAR%06d>", position);
    // Set Position First
    if (sendCommand(cmd, res) == false)
        return false;

    // if(strcmp(res, "!101)") == 0)
    // {
    //     LOG_ERROR("MoveFocuserFailed - requested movement too big. You can increase the limit by changing the value of Max. movement.");
    //     return false;
    // }

    // Now start motion toward position
    if (sendCommand("<SMOV>") == false)
        return false;

    return true;
}

bool Owlto::ISNewSwitch(const char *dev, const char *name, ISState *states, char *names[], int n)
{
    if (dev != nullptr && !strcmp(dev, getDeviceName()))
    {
        //Calibrate
        if (!strcmp(name, CalibrationSP.name)){
            IUUpdateSwitch(&CalibrationSP, states, names, n);
            int current_switch = IUFindOnSwitchIndex(&CalibrationSP);

            switch (current_switch){
                case CALIBRATION_START:
                    if (sendCommand("<CALI>") == false)
                        return false;
                    break;
                case CALIBRATION_RESET:
                    if (sendCommand("<RCAL>") == false)
                        return false;
                    break;
                default:
                    CalibrationSP.s = IPS_ALERT;
                    IDSetSwitch(&CalibrationSP, "Unknown mode index %d", current_switch);
                    return true;
            }

            CalibrationSP.s = IPS_BUSY;
            IDSetSwitch(&CalibrationSP, nullptr);
            return true;
        }
    }

    return INDI::Focuser::ISNewSwitch(dev, name, states, names, n);
}

bool Owlto::ISNewNumber(const char * dev, const char * name, double values[], char * names[], int n){
    if (dev != nullptr && !strcmp(dev, getDeviceName()))
    {
        if (!strcmp(name, FocusMaxPosNP.name)){
            IUUpdateNumber(&FocusAbsPosNP, values, names, n);
            // char cmd[OWLTO_RES] = {0};
            // snprintf(cmd , OWLTO_RES, "")

            FocusMaxPosNP.s = IPS_OK;
            IDSetNumber(&FocusMaxPosNP, nullptr);
            return true;
        }

        if (!strcmp(name, StallguardThresNP.name)){
            IUUpdateNumber(&StallguardThresNP, values, names, n);
            char cmd[OWLTO_RES] = {0};
            snprintf(cmd , OWLTO_RES, "<SSGV%06d>", static_cast<int>(StallguardThresN[0].value));
            bool rc = sendCommand(cmd);
            if (!rc)
            {
                StallguardThresNP.s = IPS_ALERT;
                return false;
            }

            StallguardThresNP.s = IPS_OK;
            IDSetNumber(&StallguardThresNP, nullptr);
            return true;
        }
        if (!strcmp(name, MoveCurrentNP.name)){
            IUUpdateNumber(&MoveCurrentNP, values, names, n);
            char cmd[OWLTO_RES] = {0};
            snprintf(cmd , OWLTO_RES, "<SMVC%06d>", static_cast<int>(MoveCurrentN[0].value));
            bool rc = sendCommand(cmd);
            if (!rc)
            {
                MoveCurrentNP.s = IPS_ALERT;
                return false;
            }

            MoveCurrentNP.s = IPS_OK;
            IDSetNumber(&MoveCurrentNP, nullptr);
            return true;
        }
    }
    return INDI::Focuser::ISNewNumber(dev, name, values, names, n);
}

void Owlto::GetFocusParams()
{
    IUResetSwitch(&CalibrationSP);

    if (readPosition())
        IDSetNumber(&FocusAbsPosNP, nullptr);
    
    if (readMaxPosition())
        IDSetNumber(&FocusMaxPosNP, nullptr);

    if (readStallguardThres())
        IDSetNumber(&StallguardThresNP, nullptr);
}

IPState Owlto::MoveFocuser(FocusDirection dir, int speed, uint16_t duration)
{
    INDI_UNUSED(speed);
    // either go all the way in or all the way out
    // then use timer to stop
    if (dir == FOCUS_INWARD)
        MoveFocuser(0);
    else
        MoveFocuser(FocusMaxPosN[0].value);

    IEAddTimer(duration, &Owlto::timedMoveHelper, this);
    return IPS_BUSY;
}

void Owlto::timedMoveHelper(void * context)
{
    static_cast<Owlto*>(context)->timedMoveCallback();
}

void Owlto::timedMoveCallback()
{
    AbortFocuser();
    FocusAbsPosNP.s = IPS_IDLE;
    FocusRelPosNP.s = IPS_IDLE;
    FocusTimerNP.s = IPS_IDLE;
    FocusTimerN[0].value = 0;
    IDSetNumber(&FocusAbsPosNP, nullptr);
    IDSetNumber(&FocusRelPosNP, nullptr);
    IDSetNumber(&FocusTimerNP, nullptr);
}


IPState Owlto::MoveAbsFocuser(uint32_t targetTicks)
{
    targetPos = targetTicks;

    if (!MoveFocuser(targetPos))
        return IPS_ALERT;

    return IPS_BUSY;
}

IPState Owlto::MoveRelFocuser(FocusDirection dir, uint32_t ticks)
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

    // JM 2019-02-10: This is already set by the framework
    //FocusRelPosN[0].value = ticks;
    //FocusRelPosNP.s       = IPS_BUSY;

    return IPS_BUSY;
}

void Owlto::TimerHit()
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

bool Owlto::AbortFocuser()
{
    return sendCommand("<ABRT>");
}

bool Owlto::saveConfigItems(FILE * fp)
{
    Focuser::saveConfigItems(fp);

    return true;
}

bool Owlto::sendCommand(const char * cmd, char * res)
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
