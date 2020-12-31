/* Includes ----------------------------------------------------------*/
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#include "bmx280.h"
#include "bmx280_params.h"
#include "fmt.h"
#include "msg.h"
#include "net/loramac.h"
#include "periph/adc.h"
#include "periph/gpio.h"
#include "periph/pm.h"
#include "periph/rtc.h"
#include "scaling.h"
#include "semtech_loramac.h"
#include "thread.h"
#include "xtimer.h"

/* Private typedef ---------------------------------------------------*/

/* Private define ----------------------------------------------------*/
#define SENDER_PRIO (THREAD_PRIORITY_MAIN - 1)
#define RECV_MSG_QUEUE (4U)
#define ON_TIME_S 6
#define PM_LOCK_LEVEL (1)

#define ADC_VREF_INT 7
#define VREF_INT_CAL ((uint16_t *) ((uint32_t) 0x1FF80078))

#ifndef DEFAULT_PERIOD_SENDING
#define DEFAULT_PERIOD_SENDING 600
#endif /* DEFAULT_PERIOD_SENDING */

#ifndef DEFAULT_RESOLUTION
#define DEFAULT_RESOLUTION 0
#endif /* DEFAULT_RESOLUTION */

/* Private macro -----------------------------------------------------*/
/* Private variables -------------------------------------------------*/
static kernel_pid_t sender_pid;
static char sender_stack[THREAD_STACKSIZE_MAIN / 2];
static char recv_stack[THREAD_STACKSIZE_DEFAULT];
static uint8_t deveui[LORAMAC_DEVEUI_LEN];
static uint8_t appeui[LORAMAC_APPEUI_LEN];
static uint8_t appkey[LORAMAC_APPKEY_LEN];
static msg_t recv_queue[RECV_MSG_QUEUE];
semtech_loramac_t loramac;
uint8_t message[50] = {0};
uint32_t sending_interval_s = DEFAULT_PERIOD_SENDING;
uint8_t resolution = DEFAULT_RESOLUTION;
bmx280_t dev;
bool bmx280_initialized = false;

/* Private function prototypes ---------------------------------------*/
static void rtc_cb(void *arg);
static int prepare_next_alarm(void);
static void send_message(uint8_t length);
static void *sender(void *arg);
static void *recv(void *arg);
float get_vcc(void);
void init_unused_pins(void);
static void set_resolution(uint8_t received_resolution);
static void set_sending_interval(uint16_t received_interval);

/* Private functions -------------------------------------------------*/
int main(void)
{
    uint8_t number_join_tries = 0;
    uint8_t loramac_datarate = 0;

    /* Init peripherie */
    init_unused_pins();
    adc_init(ADC_LINE(ADC_VREF_INT));

    switch (bmx280_init(&dev, &bmx280_params[0])) {
        case BMX280_ERR_BUS:
            puts("[Error] Something went wrong when using the I2C bus");
            break;
        case BMX280_ERR_NODEV:
            puts("[Error] Unable to communicate with any BMX280 device");
            break;
        default:
            bmx280_initialized = true;
            break;
    }

    /* Convert identifiers and application key */
    fmt_hex_bytes(deveui, DEVEUI);
    fmt_hex_bytes(appeui, APPEUI);
    fmt_hex_bytes(appkey, APPKEY);

    /* Initialize the loramac stack */
    semtech_loramac_init(&loramac);
    semtech_loramac_set_deveui(&loramac, deveui);
    semtech_loramac_set_appeui(&loramac, appeui);
    semtech_loramac_set_appkey(&loramac, appkey);

    /* Use a fast datarate, e.g. BW125/SF7 in EU868 */
    semtech_loramac_set_dr(&loramac, LORAMAC_DR_5);

    /* Use ADR */
    semtech_loramac_set_adr(&loramac, true);

    /* Use unconfirmed data mode */
    semtech_loramac_set_tx_mode(&loramac, LORAMAC_TX_UNCNF);

    /* Use port 1 for uplink masseges */
    semtech_loramac_set_tx_port(&loramac, 1);

    /* Start the Over-The-Air Activation (OTAA) procedure to retrieve the
     * generated device address and to get the network and application session
     * keys.
     */
    puts("Starting join procedure");
    while (semtech_loramac_join(&loramac, LORAMAC_JOIN_OTAA) != SEMTECH_LORAMAC_JOIN_SUCCEEDED) {
        puts("Join procedure failed, try in 30s again");
        xtimer_sleep(22);

        /* increase datarate after 3 join tries */
        number_join_tries++;
        loramac_datarate = semtech_loramac_get_dr(&loramac);
        if (number_join_tries > 2 && loramac_datarate > LORAMAC_DR_0) {
            number_join_tries = 0;
            loramac_datarate--;
            semtech_loramac_set_dr(&loramac, loramac_datarate);
        }
    }
    puts("Join procedure succeeded");

    /* start the sender thread */
    sender_pid = thread_create(sender_stack, sizeof(sender_stack), SENDER_PRIO, 0, sender, NULL, "sender");

    /* start the receive thread */
    thread_create(recv_stack, sizeof(recv_stack), THREAD_PRIORITY_MAIN - 1, 0, recv, NULL, "recv thread");

    /* trigger the first send */
    msg_t msg;
    msg_send(&msg, sender_pid);
    return 0;
}

static void rtc_cb(void *arg)
{
    (void)arg;
    
    /* block sleep level mode until the next sending cycle has completed */
    pm_block(PM_LOCK_LEVEL);

    msg_t msg;
    //disable_gpio_irq();
    msg_send(&msg, sender_pid);
}

static int prepare_next_alarm(void)
{
    struct tm time;
    struct tm get_alarm;
    struct tm set_alarm;
    int rc;
    uint8_t tries = 3;

    rtc_get_time(&time);
    memcpy(&set_alarm, &time, sizeof(struct tm));
    set_alarm.tm_sec += (sending_interval_s - ON_TIME_S);
    mktime(&set_alarm);

    do {
        rtc_set_alarm(&set_alarm, rtc_cb, NULL);
        rtc_get_alarm(&get_alarm);
        rc = rtc_tm_compare(&set_alarm, &get_alarm);
        tries--;
    } while ((rc != 0) && (tries != 0));

    if (rc == 0) {
        puts("RTC alarm set");
    } else {
        puts("RTC alarm not set");
    }

    return rc;
}

static void send_message(uint8_t length)
{
    //TODO print data and port

    printf("Sending\n");
    /* Try to send the message */
    uint8_t ret = semtech_loramac_send(&loramac, message, length);
    if (ret != SEMTECH_LORAMAC_TX_DONE) {
        printf("Cannot send message, ret code: %d\n", ret);
        return;
    }
}

static void *sender(void *arg) {
    (void)arg;

    msg_t msg;
    msg_t msg_queue[8];
    msg_init_queue(msg_queue, 8);

    while (1) {
        msg_receive(&msg);

        float temperature_f = 0.0;
        int16_t temperature_i = 0;
        float humidity_f = 0.0;
        float pressure_f = 0.0;
        int8_t sensor_status = 0;

        if (bmx280_initialized) {
            temperature_i = bmx280_read_temperature(&dev);
            if (temperature_i != INT16_MIN) {
                temperature_f = (float)temperature_i;
                humidity_f = (float)bme280_read_humidity(&dev);
                pressure_f = (float)bmx280_read_pressure(&dev);
            } else {
                puts("[Error] Unable to communicate with BMX280 device");
                sensor_status = -2;
            }
        } else {
            puts("[Error] BMX280 not initialized");
            sensor_status = -2;
        }

        if (sensor_status != 0) {
            semtech_loramac_set_tx_port(&loramac, 10);  //TODO Port enum
            message[0] = sensor_status;
            send_message(1);
        } else {
            if (resolution == 0) {
                semtech_loramac_set_tx_port(&loramac, 1);  //TODO Port enum

                uint8_t vbat = (uint8_t)scaling_float(get_vcc(), 2.0, 4.0, 0.0, 255.0, LIMIT_OUTPUT);
                uint8_t temperature = (uint8_t)scaling_float(temperature_f, -4000.0, 8500.0, 0.0, 250.0, LIMIT_OUTPUT);
                uint8_t humidity = (uint8_t)scaling_float(humidity_f, 0.0, 10000.0, 0.0, 200.0, LIMIT_OUTPUT);
                uint8_t pressure = (uint8_t)scaling_float(pressure_f, 95000.0, 105000.0, 0.0, 200.0, LIMIT_OUTPUT);

                message[0] = (uint8_t)vbat;
                message[1] = (uint8_t)temperature;
                message[2] = (uint8_t)humidity;
                message[3] = (uint8_t)pressure;

                send_message(4);
            } else if (resolution == 1) {
                semtech_loramac_set_tx_port(&loramac, 2);  //TODO Port enum

                uint16_t vbat = (uint16_t)scaling_float(get_vcc(), 2.0, 4.0, 0.0, 65535.0, LIMIT_OUTPUT);
                uint16_t temperature = (uint16_t)scaling_float(temperature_f, -4000.0, 8500.0, 0.0, 65535.0, LIMIT_OUTPUT);
                uint16_t humidity = (uint16_t)scaling_float(humidity_f, 0.0, 10000.0, 0.0, 65535.0, LIMIT_OUTPUT);
                uint16_t pressure = (uint16_t)scaling_float(pressure_f, 95000.0, 105000.0, 0.0, 65535.0, LIMIT_OUTPUT);

                message[0] = (uint8_t)(vbat >> 8);
                message[1] = (uint8_t)(vbat);
                message[2] = (uint8_t)(temperature >> 8);
                message[3] = (uint8_t)(temperature);
                message[4] = (uint8_t)(humidity >> 8);
                message[5] = (uint8_t)(humidity);
                message[6] = (uint8_t)(pressure >> 8);
                message[7] = (uint8_t)(pressure);

                send_message(8);
            }
        }

        /* Schedule the next wake-up alarm */
        int rc_rtc = 0;

        rc_rtc = prepare_next_alarm();

        if (rc_rtc == 0) {
            /* going to deep sleep */
            puts("Going to sleep");
            pm_unblock(PM_LOCK_LEVEL);
        } else {
            /* rtc can not set, try reboot */
            pm_reboot();
        }
    }

    /* this should never be reached */
    return NULL;
}

static void *recv(void *arg)
{
    msg_init_queue(recv_queue, RECV_MSG_QUEUE);

    (void)arg;

    while (1) {
        /* blocks until some data is received */
        switch (semtech_loramac_recv(&loramac)) {
            case SEMTECH_LORAMAC_RX_DATA:
                loramac.rx_data.payload[loramac.rx_data.payload_len] = 0;
                printf("Data received: %s, port: %d\n", (char *)loramac.rx_data.payload, loramac.rx_data.port);

                /* process received data */
                switch (loramac.rx_data.port) {
                    case 1:
                        set_resolution(loramac.rx_data.payload[0]);
                        break;
                    
                    case 2:
                        set_sending_interval((loramac.rx_data.payload[0] << 8) + loramac.rx_data.payload[1]);
                        break;
                    
                    case 3: /* System reboot */
                        if (loramac.rx_data.payload[0] == 1) {
                            pm_reboot();
                        }
                        break;
                    
                    default:
                        break;
                }
                break;

            case SEMTECH_LORAMAC_RX_CONFIRMED:
                puts("Received ACK from network");
                break;

            default:
                break;
        }
    }
    return NULL;
}

float get_vcc(void)
{
    uint16_t *vref_int_cal = VREF_INT_CAL;
    float vbat = 3.0 * *vref_int_cal / adc_sample(ADC_VREF_INT, ADC_RES_12BIT); //TODO catch failure of adc_sample 
    return vbat;
}

void init_unused_pins(void)
{
    gpio_t unused_pins[] = {
        GPIO_PIN(PORT_A, 0),
        GPIO_PIN(PORT_A, 1),
        GPIO_PIN(PORT_A, 2),
        GPIO_PIN(PORT_A, 3),
        //GPIO_PIN(PORT_A, 4), NSS
        //GPIO_PIN(PORT_A, 5), SCK
        //GPIO_PIN(PORT_A, 6), MISO
        //GPIO_PIN(PORT_A, 7), MOSI
        GPIO_PIN(PORT_A, 8),
        GPIO_PIN(PORT_A, 9),
        GPIO_PIN(PORT_A, 10),
        GPIO_PIN(PORT_A, 11),
        GPIO_PIN(PORT_A, 12),
        GPIO_PIN(PORT_A, 13),
        GPIO_PIN(PORT_A, 14),
        GPIO_PIN(PORT_A, 15),
        GPIO_PIN(PORT_B, 0),
        GPIO_PIN(PORT_B, 1),
        GPIO_PIN(PORT_B, 2),
        GPIO_PIN(PORT_B, 3),
        GPIO_PIN(PORT_B, 4),
        GPIO_PIN(PORT_B, 5),
        GPIO_PIN(PORT_B, 6),
        GPIO_PIN(PORT_B, 7),
        //GPIO_PIN(PORT_B, 8), SCL
        //GPIO_PIN(PORT_B, 9), SDA
        GPIO_PIN(PORT_B, 10),
        GPIO_PIN(PORT_B, 11),
        GPIO_PIN(PORT_B, 12),
        GPIO_PIN(PORT_B, 13),
        GPIO_PIN(PORT_B, 14),
        GPIO_PIN(PORT_B, 15),
        GPIO_PIN(PORT_C, 0),
        GPIO_PIN(PORT_C, 1),
        GPIO_PIN(PORT_C, 2),
        GPIO_PIN(PORT_C, 3),
        GPIO_PIN(PORT_C, 4),
        GPIO_PIN(PORT_C, 5),
        //GPIO_PIN(PORT_C, 6), DIO0
        //GPIO_PIN(PORT_C, 7), DIO1
        //GPIO_PIN(PORT_C, 8), DIO2
        //GPIO_PIN(PORT_C, 9), RESET
        GPIO_PIN(PORT_C, 10),
        GPIO_PIN(PORT_C, 11),
        GPIO_PIN(PORT_C, 12),
        GPIO_PIN(PORT_C, 13),
        //GPIO_PIN(PORT_C, 14), RTC crystal
        //GPIO_PIN(PORT_C, 15), RTC crystal
        //GPIO_PIN(PORT_D, 2), MOSFET
        GPIO_PIN(PORT_H, 0),
        GPIO_PIN(PORT_H, 1)
    };
    for (uint8_t i = 0; i < (sizeof(unused_pins) / sizeof(gpio_t)); i++) {
        gpio_init(unused_pins[i], GPIO_IN_PD);
    }
}

static void set_resolution(uint8_t received_resolution)
{
    if (received_resolution <= 1) {
        resolution = received_resolution;
    }
}

static void set_sending_interval(uint16_t received_interval)
{
    if (received_interval >= 3) {
        sending_interval_s = received_interval * 10;
    }
}