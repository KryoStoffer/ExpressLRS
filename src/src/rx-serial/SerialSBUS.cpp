#include "SerialSBUS.h"
#include "CRSF.h"
#include "device.h"
#include "config.h"

#if defined(TARGET_RX)

#define SBUS_FLAG_SIGNAL_LOSS       (1 << 2)
#define SBUS_FLAG_FAILSAFE_ACTIVE   (1 << 3)

const auto UNCONNECTED_CALLBACK_INTERVAL_MS = 10;
const auto SBUS_CALLBACK_INTERVAL_MS = 9;

uint32_t SerialSBUS::sendRCFrame(bool frameAvailable, bool frameMissed, uint32_t *channelData)
{
    static auto sendPackets = false;
    bool effectivelyFailsafed = failsafe || (!connectionHasModelMatch) || (!teamraceHasModelMatch);
    if ((effectivelyFailsafed && config.GetFailsafeMode() == FAILSAFE_NO_PULSES) || (!sendPackets && connectionState != connected))
    {
        return UNCONNECTED_CALLBACK_INTERVAL_MS;
    }
    sendPackets = true;

    if ((!frameAvailable && !frameMissed && !effectivelyFailsafed) || _outputPort->availableForWrite() < 25)
    {
        return DURATION_IMMEDIATELY;
    }

    // TODO: if failsafeMode == FAILSAFE_SET_POSITION then we use the set positions rather than the last values
    crsf_channels_s PackedRCdataOut;

    if ((effectivelyFailsafed && config.GetFailsafeMode() == FAILSAFE_SET_POSITION) || (!sendPackets && connectionState != connected))
    {
        PackedRCdataOut.ch0 = UINT10_to_CRSF(config.GetPwmChannel(0)->val.failsafe);
        PackedRCdataOut.ch1 = UINT10_to_CRSF(config.GetPwmChannel(1)->val.failsafe);
        PackedRCdataOut.ch2 = UINT10_to_CRSF(config.GetPwmChannel(2)->val.failsafe);
        PackedRCdataOut.ch3 = UINT10_to_CRSF(config.GetPwmChannel(3)->val.failsafe);
        PackedRCdataOut.ch4 = UINT10_to_CRSF(config.GetPwmChannel(4)->val.failsafe);
        PackedRCdataOut.ch5 = UINT10_to_CRSF(config.GetPwmChannel(5)->val.failsafe);
        PackedRCdataOut.ch6 = UINT10_to_CRSF(config.GetPwmChannel(6)->val.failsafe);
        PackedRCdataOut.ch7 = UINT10_to_CRSF(config.GetPwmChannel(7)->val.failsafe);
        PackedRCdataOut.ch8 = UINT10_to_CRSF(config.GetPwmChannel(8)->val.failsafe);
        PackedRCdataOut.ch9 = UINT10_to_CRSF(config.GetPwmChannel(9)->val.failsafe);
        PackedRCdataOut.ch10 = UINT10_to_CRSF(config.GetPwmChannel(10)->val.failsafe);
        PackedRCdataOut.ch11 = UINT10_to_CRSF(config.GetPwmChannel(11)->val.failsafe);
        PackedRCdataOut.ch12 = UINT10_to_CRSF(config.GetPwmChannel(12)->val.failsafe);
        PackedRCdataOut.ch13 = UINT10_to_CRSF(config.GetPwmChannel(13)->val.failsafe);
        PackedRCdataOut.ch14 = UINT10_to_CRSF(config.GetPwmChannel(14)->val.failsafe);
        PackedRCdataOut.ch15 = UINT10_to_CRSF(config.GetPwmChannel(15)->val.failsafe);
    }
    else if (config.GetSerialProtocol() == PROTOCOL_DJI_RS_PRO)
    {
        PackedRCdataOut.ch0 = fmap(channelData[0], CRSF_CHANNEL_VALUE_MIN, CRSF_CHANNEL_VALUE_MAX, 352, 1696);
        PackedRCdataOut.ch1 = fmap(channelData[1], CRSF_CHANNEL_VALUE_MIN, CRSF_CHANNEL_VALUE_MAX, 352, 1696);
        PackedRCdataOut.ch2 = fmap(channelData[2], CRSF_CHANNEL_VALUE_MIN, CRSF_CHANNEL_VALUE_MAX, 352, 1696);
        PackedRCdataOut.ch3 = fmap(channelData[3], CRSF_CHANNEL_VALUE_MIN, CRSF_CHANNEL_VALUE_MAX, 352, 1696);
        PackedRCdataOut.ch4 = fmap(channelData[5], CRSF_CHANNEL_VALUE_MIN, CRSF_CHANNEL_VALUE_MAX, 352, 1696); // Record start/stop and photo
        PackedRCdataOut.ch5 = fmap(channelData[6], CRSF_CHANNEL_VALUE_MIN, CRSF_CHANNEL_VALUE_MAX, 352, 1696); // Mode
        PackedRCdataOut.ch6 = fmap(channelData[7], CRSF_CHANNEL_VALUE_MIN, CRSF_CHANNEL_VALUE_MAX, 176,  848); // Recenter and Selfie
        PackedRCdataOut.ch7 = fmap(channelData[8], CRSF_CHANNEL_VALUE_MIN, CRSF_CHANNEL_VALUE_MAX, 352, 1696);
        PackedRCdataOut.ch8 = fmap(channelData[9], CRSF_CHANNEL_VALUE_MIN, CRSF_CHANNEL_VALUE_MAX, 352, 1696);
        PackedRCdataOut.ch9 = fmap(channelData[10], CRSF_CHANNEL_VALUE_MIN, CRSF_CHANNEL_VALUE_MAX, 352, 1696);
        PackedRCdataOut.ch10 = fmap(channelData[11], CRSF_CHANNEL_VALUE_MIN, CRSF_CHANNEL_VALUE_MAX, 352, 1696);
        PackedRCdataOut.ch11 = fmap(channelData[12], CRSF_CHANNEL_VALUE_MIN, CRSF_CHANNEL_VALUE_MAX, 352, 1696);
        PackedRCdataOut.ch12 = fmap(channelData[13], CRSF_CHANNEL_VALUE_MIN, CRSF_CHANNEL_VALUE_MAX, 352, 1696);
        PackedRCdataOut.ch13 = fmap(channelData[14], CRSF_CHANNEL_VALUE_MIN, CRSF_CHANNEL_VALUE_MAX, 352, 1696);
        PackedRCdataOut.ch14 = fmap(channelData[15], CRSF_CHANNEL_VALUE_MIN, CRSF_CHANNEL_VALUE_MAX, 352, 1696);
        PackedRCdataOut.ch15 = channelData[4] < CRSF_CHANNEL_VALUE_MID ? 352 : 1696;
    }
    else
    {
        PackedRCdataOut.ch0 = channelData[config.GetPwmChannel(0)->val.inputChannel];
        PackedRCdataOut.ch1 = channelData[config.GetPwmChannel(1)->val.inputChannel];
        PackedRCdataOut.ch2 = channelData[config.GetPwmChannel(2)->val.inputChannel];
        PackedRCdataOut.ch3 = channelData[config.GetPwmChannel(3)->val.inputChannel];
        PackedRCdataOut.ch4 = channelData[config.GetPwmChannel(4)->val.inputChannel];
        PackedRCdataOut.ch5 = channelData[config.GetPwmChannel(5)->val.inputChannel];
        PackedRCdataOut.ch6 = channelData[config.GetPwmChannel(6)->val.inputChannel];
        PackedRCdataOut.ch7 = channelData[config.GetPwmChannel(7)->val.inputChannel];
        PackedRCdataOut.ch8 = channelData[config.GetPwmChannel(8)->val.inputChannel];
        PackedRCdataOut.ch9 = channelData[config.GetPwmChannel(9)->val.inputChannel];
        PackedRCdataOut.ch10 = channelData[config.GetPwmChannel(10)->val.inputChannel];
        PackedRCdataOut.ch11 = channelData[config.GetPwmChannel(11)->val.inputChannel];
        PackedRCdataOut.ch12 = channelData[config.GetPwmChannel(12)->val.inputChannel];
        PackedRCdataOut.ch13 = channelData[config.GetPwmChannel(13)->val.inputChannel];
        PackedRCdataOut.ch14 = channelData[config.GetPwmChannel(14)->val.inputChannel];
        PackedRCdataOut.ch15 = channelData[config.GetPwmChannel(15)->val.inputChannel];
    }

    uint8_t extraData = 0;
    extraData |= effectivelyFailsafed ? SBUS_FLAG_FAILSAFE_ACTIVE : 0;
    extraData |= frameMissed ? SBUS_FLAG_SIGNAL_LOSS : 0;

    _outputPort->write(0x0F);    // HEADER
    _outputPort->write((byte *)&PackedRCdataOut, RCframeLength);
    _outputPort->write((uint8_t)extraData);    // ch 17, 18, lost packet, failsafe
    _outputPort->write((uint8_t)0x00);    // FOOTER
    return SBUS_CALLBACK_INTERVAL_MS;
}

#endif
