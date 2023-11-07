#if defined(PLATFORM_ESP32)

#include "targets.h"
#include "common.h"
#include "device.h"
#include "config.h"
#include "telemetry.h"
#include "logging.h"

#include "devHobbyWing.h"

extern Telemetry telemetry;

static void initialize()
{

    for (int i = 0 ; i < GPIO_PIN_PWM_OUTPUTS_COUNT ; i++)
    {
        eServoOutputMode pinMode = (eServoOutputMode)config.GetPwmChannel(i)->val.mode;
        DBGLN("HobbyWing_device PWM_config %d mode %d pin %d",i,pinMode,GPIO_PIN_PWM_OUTPUTS[i]);
        if (pinMode == somRX2 ) {
            Serial2.begin(19200, SERIAL_8N1, GPIO_PIN_PWM_OUTPUTS[i], -1);
            DBGLN("HobbyWing_device Seria open pin: %d", GPIO_PIN_PWM_OUTPUTS[i]);
        }
    }


}

static int start()
{
    return DURATION_IMMEDIATELY;
}

static int timeout()
{

    // indicate external sensor is present
    //telemetry.SetCrsfBatterySensorDetected();


    uint8_t buffer[64];
    auto size = min(Serial2.available(), 64);
    if (size) {
        DBG("Serial2 in size=%d ",size);
        Serial2.readBytes(buffer, size);
        for (uint16_t i=0; i < size; i++)
        {
            DBG("%u ",buffer[i]);
        }
        DBGCR;
        switch (size)
        {
            case 19:
            case 20:
                uint32_t count=buffer[1] << 16 | buffer[2] << 8 | buffer[3];
                uint16_t thr= buffer[4] << 8 | buffer[5];
                uint16_t pwm= buffer[6] << 8 | buffer[7];
                uint32_t rpm=buffer[8] << 16 | buffer[9] << 8 | buffer[10];
                uint16_t voltage=8*(buffer[11] << 8 | buffer[12])/91;
                uint16_t curr= buffer[13] << 8 | buffer[14];
                DBGLN("Count: %d, Thr: %d, pwm:%d, RPM:%d, Vol: %d, Cur: %d",count,thr,pwm,rpm,voltage,curr);
                // prepare CRSF telemetry packet
                CRSF_MK_FRAME_T(crsf_sensor_battery_t) crsfBatt = {0};
                crsfBatt.p.voltage = htobe16(voltage);
                crsfBatt.p.current = htobe16(curr);
                crsfBatt.p.capacity = htobe32(rpm << 8);
                crsfBatt.p.remaining = 40;
                CRSF::SetHeaderAndCrc((uint8_t *)&crsfBatt, CRSF_FRAMETYPE_BATTERY_SENSOR, CRSF_FRAME_SIZE(sizeof(crsf_sensor_battery_t)), CRSF_ADDRESS_CRSF_TRANSMITTER);

                telemetry.AppendTelemetryPackage((uint8_t *)&crsfBatt);
                break;
        }
    }


    return 20;
}

static int event()
{
    return 10;
}

device_t HobbyWing_device = {
    .initialize = initialize,
    .start = start,
    .event = event,
    .timeout = timeout
};
#endif
