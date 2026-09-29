/*
 * ezkontrol_can_test.c
 *
 * STM32F103 HAL bench-test module for EZkontrol MCU <-> VCU CAN protocol.
 *
 * What it does:
 *   1) Detects EZkontrol startup handshake: ID 0x1801D0EF, 8 bytes of 0x55
 *   2) Replies: ID 0x0C01EFD0, 8 bytes of 0xAA
 *   3) Sends a SAFE HALT frame every 50 ms after handshake
 *   4) Decodes controller feedback: bus voltage/current, phase current, speed,
 *      temperatures, RUN/HALT, and faults
 *   5) If MCU->Meter frames are also broadcast, decodes throttle %, brake,
 *      gear, Cruise/EBS/Hold, and DC contactor state
 *
 * IMPORTANT:
 *   - Motor RUN is compile-time disabled by default.
 *   - Brake/Cruise/Hold are NOT VCU CAN commands in the supplied protocol;
 *     this file only MONITORS their reported state if meter frames are present.
 *   - CN1-10 PWM is a physical signal pin, not a CAN item in the supplied docs.
 *     Measure/condition that signal separately before connecting it to STM32.
 *
 * CubeMX assumption, 500 kbit/s:
 *   CAN clock 36 MHz, Prescaler=4, SJW=1TQ, BS1=15TQ, BS2=2TQ
 *   Enable CAN1 RX1 interrupt.
 *   EZ-Tune setting: CAN protocol 102 = 500 kbit/s
 *
 * For 250 kbit/s with the same segment timing, use Prescaler=8 and
 * EZ-Tune CAN protocol 2.
 *
 * Usage from main.c:
 *   after MX_CAN_Init():
 *       EZK_CAN_Test_Init();
 *
 *   inside while(1):
 *       EZK_CAN_Test_Task();
 *
 * Add prototypes to main.c or a header:
 *   void EZK_CAN_Test_Init(void);
 *   void EZK_CAN_Test_Task(void);
 *
 * Do not define another HAL_CAN_RxFifo1MsgPendingCallback() elsewhere.
 */

#include "main.h"
#include <stdint.h>

extern CAN_HandleTypeDef hcan;

/* -------- EZkontrol extended CAN identifiers, default MCU address 0xEF -------- */
#define EZK_ID_VCU_TO_MCU       0x0C01EFD0UL
#define EZK_ID_MCU_TO_VCU_1     0x1801D0EFUL
#define EZK_ID_MCU_TO_VCU_2     0x1802D0EFUL
#define EZK_ID_MCU_TO_METER_1   0x180117EFUL
#define EZK_ID_MCU_TO_METER_2   0x180217EFUL

#define EZK_COMMAND_PERIOD_MS    50U

/* Safety gate: leave 0 for handshake/status testing. */
#define EZK_ALLOW_RUN_TEST       0

/* ---------------- Live Expressions / debug variables ---------------- */
volatile uint32_t ez_handshake_request_count = 0;
volatile uint32_t ez_handshake_reply_count   = 0;
volatile uint8_t  ez_handshake_reply_sent    = 0;
volatile uint8_t  ez_handshake_established   = 0;

volatile uint32_t ez_status1_count = 0;
volatile uint32_t ez_status2_count = 0;
volatile uint32_t ez_meter1_count  = 0;
volatile uint32_t ez_meter2_count  = 0;
volatile uint32_t ez_last_rx_tick  = 0;
volatile uint32_t ez_can_hal_error = 0;

/* MCU->VCU message I */
volatile uint16_t ez_bus_voltage_0p1V   = 0; /* 1 count = 0.1 V */
volatile int16_t  ez_bus_current_0p1A   = 0; /* 1 count = 0.1 A */
volatile int16_t  ez_phase_current_0p1A = 0; /* 1 count = 0.1 A */
volatile int16_t  ez_speed_rpm          = 0;

/* MCU->VCU message II */
volatile int16_t ez_controller_temp_C = 0;
volatile int16_t ez_motor_temp_C      = 0;
volatile uint8_t ez_controller_running = 0;
volatile uint8_t ez_controller_speed_mode = 0;

volatile uint8_t ez_error_byte3 = 0;
volatile uint8_t ez_error_byte4 = 0;
volatile uint8_t ez_error_byte5 = 0;
volatile uint8_t ez_main_throttle_error = 0;
volatile uint8_t ez_aux_throttle_error  = 0;
volatile uint8_t ez_precharge_error     = 0;
volatile uint8_t ez_can_comm_error      = 0;
volatile uint8_t ez_controller_life     = 0;

/* Optional MCU->Meter message II: useful to visualize analog controls */
volatile uint8_t ez_throttle_percent   = 0;
volatile uint8_t ez_gear_raw           = 0;
volatile uint8_t ez_brake_active       = 0;
volatile uint8_t ez_operation_mode_raw = 0;
volatile uint8_t ez_cruise_active      = 0;
volatile uint8_t ez_ebs_active         = 0;
volatile uint8_t ez_hold_active        = 0;
volatile uint8_t ez_dc_contactor_on    = 0;
volatile int16_t ez_meter_speed_raw    = 0;

/* Complete MCU->Meter Message II monitoring */
volatile int16_t ez_meter_controller_temp_C = 0;
volatile int16_t ez_meter_motor_temp_C      = 0;

volatile uint8_t ez_meter_error_byte4 = 0;
volatile uint8_t ez_meter_error_byte5 = 0;
volatile uint8_t ez_meter_error_byte6 = 0;
volatile uint8_t ez_meter_life        = 0;

/* Byte 4 faults */
volatile uint8_t ez_merr_overcurrent         = 0;
volatile uint8_t ez_merr_overload            = 0;
volatile uint8_t ez_merr_overvoltage         = 0;
volatile uint8_t ez_merr_undervoltage        = 0;
volatile uint8_t ez_merr_controller_overheat = 0;
volatile uint8_t ez_merr_motor_overheat      = 0;
volatile uint8_t ez_merr_motor_stalled       = 0;
volatile uint8_t ez_merr_motor_out_of_phase  = 0;

/* Byte 5 faults */
volatile uint8_t ez_merr_motor_sensor        = 0;
volatile uint8_t ez_merr_motor_aux_sensor    = 0;
volatile uint8_t ez_merr_encoder_misaligned  = 0;
volatile uint8_t ez_merr_anti_runaway        = 0;
volatile uint8_t ez_merr_main_accelerator    = 0;
volatile uint8_t ez_merr_aux_accelerator     = 0;
volatile uint8_t ez_merr_precharge           = 0;
volatile uint8_t ez_merr_dc_contactor        = 0;

/* Byte 6 faults */
volatile uint8_t ez_merr_power_valve    = 0;
volatile uint8_t ez_merr_current_sensor = 0;
volatile uint8_t ez_merr_auto_tune      = 0;
volatile uint8_t ez_merr_rs485          = 0;
volatile uint8_t ez_merr_can            = 0;
volatile uint8_t ez_merr_software       = 0;

/* Editable future RUN test values.
 * Manufacturer says target phase current and target speed should have same sign.
 * Examples: forward +50 (5.0A), +200rpm; reverse -50, -200rpm.
 * Ignored unless EZK_ALLOW_RUN_TEST == 1 AND ez_test_run_request == 1. */
volatile uint8_t ez_test_run_request = 0;
volatile int16_t ez_test_target_phase_current_0p1A = 50; /* 5.0 A */
volatile int16_t ez_test_target_speed_rpm = 200;

static CAN_RxHeaderTypeDef rxh;
static CAN_TxHeaderTypeDef txh;
static uint8_t rxd[8];
static uint32_t tx_mailbox;
static uint8_t life_counter = 0;
static uint32_t last_command_tick = 0;

static uint16_t u16le(const uint8_t *p)
{
    return (uint16_t)p[0] | ((uint16_t)p[1] << 8);
}

/* 0.1A/bit, offset -3200A -> raw = physical_0.1A + 32000 */
static uint16_t encode_phase_current(int32_t current_0p1A)
{
    int32_t raw = current_0p1A + 32000;
    if (raw < 0) raw = 0;
    if (raw > 64000) raw = 64000;
    return (uint16_t)raw;
}

/* 1rpm/bit, offset -32000rpm -> raw = rpm + 32000 */
static uint16_t encode_speed(int32_t rpm)
{
    int32_t raw = rpm + 32000;
    if (raw < 0) raw = 0;
    if (raw > 64000) raw = 64000;
    return (uint16_t)raw;
}

static uint8_t frame_is_8x(uint8_t value)
{
    uint8_t i;
    if (rxh.DLC != 8) return 0;
    for (i = 0; i < 8; i++)
        if (rxd[i] != value) return 0;
    return 1;
}

static void send_handshake_reply(void)
{
    uint8_t tx[8] = {0xAA,0xAA,0xAA,0xAA,0xAA,0xAA,0xAA,0xAA};

    txh.ExtId = EZK_ID_VCU_TO_MCU;
    txh.IDE   = CAN_ID_EXT;
    txh.RTR   = CAN_RTR_DATA;
    txh.DLC   = 8;

    if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan) > 0)
    {
        if (HAL_CAN_AddTxMessage(&hcan, &txh, tx, &tx_mailbox) == HAL_OK)
        {
            ez_handshake_reply_count++;
            ez_handshake_reply_sent = 1;
        }
    }
}

/* Normal VCU control frame.
 * byte0-1 phase current, byte2-3 speed,
 * byte4 bit0 RUN/HALT, bit1 torque/speed mode,
 * byte5-6 reserved, byte7 life counter. */
static void send_control(uint8_t run, int16_t current_0p1A, int16_t speed_rpm)
{
    uint8_t tx[8];
    uint16_t cr = encode_phase_current(current_0p1A);
    uint16_t sr = encode_speed(speed_rpm);

    tx[0] = (uint8_t)(cr & 0xFF);
    tx[1] = (uint8_t)(cr >> 8);
    tx[2] = (uint8_t)(sr & 0xFF);
    tx[3] = (uint8_t)(sr >> 8);

    tx[4] = 0;
    if (run) tx[4] |= (1U << 0);
    tx[4] |= (1U << 1); /* request speed mode */

    tx[5] = 0;
    tx[6] = 0;
    tx[7] = life_counter++;

    txh.ExtId = EZK_ID_VCU_TO_MCU;
    txh.IDE   = CAN_ID_EXT;
    txh.RTR   = CAN_RTR_DATA;
    txh.DLC   = 8;

    if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan) > 0)
        HAL_CAN_AddTxMessage(&hcan, &txh, tx, &tx_mailbox);
}

static void parse_vcu_status1(void)
{
    uint16_t raw;
    ez_status1_count++;

    ez_bus_voltage_0p1V = u16le(&rxd[0]);

    raw = u16le(&rxd[2]);
    ez_bus_current_0p1A = (int16_t)((int32_t)raw - 32000);

    raw = u16le(&rxd[4]);
    ez_phase_current_0p1A = (int16_t)((int32_t)raw - 32000);

    raw = u16le(&rxd[6]);
    ez_speed_rpm = (int16_t)((int32_t)raw - 32000);

    if (ez_handshake_reply_sent) ez_handshake_established = 1;
}

static void parse_vcu_status2(void)
{
    ez_status2_count++;

    ez_controller_temp_C = (int16_t)rxd[0] - 40;
    ez_motor_temp_C      = (int16_t)rxd[1] - 40;

    ez_controller_running    = (rxd[2] >> 0) & 0x01;
    ez_controller_speed_mode = (rxd[2] >> 1) & 0x01;

    ez_error_byte3 = rxd[3];
    ez_error_byte4 = rxd[4];
    ez_error_byte5 = rxd[5];

    ez_main_throttle_error = (ez_error_byte4 >> 4) & 0x01;
    ez_aux_throttle_error  = (ez_error_byte4 >> 5) & 0x01;
    ez_precharge_error     = (ez_error_byte4 >> 6) & 0x01;
    ez_can_comm_error      = (ez_error_byte5 >> 4) & 0x01;

    ez_controller_life = rxd[7];

    if (ez_handshake_reply_sent) ez_handshake_established = 1;
}

static void parse_meter1(void)
{
    ez_meter1_count++;
    /* Kept as raw-decoded field for comparison with VCU status speed. */
    ez_meter_speed_raw = (int16_t)((int32_t)u16le(&rxd[6]) - 32000);
}

static void parse_meter2(void)
{
    ez_meter2_count++;

    /* Byte 0 */
    ez_meter_controller_temp_C =
        (int16_t)rxd[0] - 40;

    /* Byte 1 */
    ez_meter_motor_temp_C =
        (int16_t)rxd[1] - 40;

    /* Byte 2 */
    ez_throttle_percent = rxd[2];

    /* Byte 3 */
    ez_gear_raw =
        rxd[3] & 0x07;

    ez_brake_active =
        (rxd[3] >> 3) & 0x01;

    ez_operation_mode_raw =
        (rxd[3] >> 4) & 0x07;

    ez_dc_contactor_on =
        (rxd[3] >> 7) & 0x01;

    ez_cruise_active =
        (ez_operation_mode_raw == 2);

    ez_ebs_active =
        (ez_operation_mode_raw == 3);

    ez_hold_active =
        (ez_operation_mode_raw == 4);

    /* Bytes 4-6 = fault bytes */
    ez_meter_error_byte4 = rxd[4];
    ez_meter_error_byte5 = rxd[5];
    ez_meter_error_byte6 = rxd[6];

    /* Byte 4 */
    ez_merr_overcurrent =
        (rxd[4] >> 0) & 0x01;

    ez_merr_overload =
        (rxd[4] >> 1) & 0x01;

    ez_merr_overvoltage =
        (rxd[4] >> 2) & 0x01;

    ez_merr_undervoltage =
        (rxd[4] >> 3) & 0x01;

    ez_merr_controller_overheat =
        (rxd[4] >> 4) & 0x01;

    ez_merr_motor_overheat =
        (rxd[4] >> 5) & 0x01;

    ez_merr_motor_stalled =
        (rxd[4] >> 6) & 0x01;

    ez_merr_motor_out_of_phase =
        (rxd[4] >> 7) & 0x01;

    /* Byte 5 */
    ez_merr_motor_sensor =
        (rxd[5] >> 0) & 0x01;

    ez_merr_motor_aux_sensor =
        (rxd[5] >> 1) & 0x01;

    ez_merr_encoder_misaligned =
        (rxd[5] >> 2) & 0x01;

    ez_merr_anti_runaway =
        (rxd[5] >> 3) & 0x01;

    ez_merr_main_accelerator =
        (rxd[5] >> 4) & 0x01;

    ez_merr_aux_accelerator =
        (rxd[5] >> 5) & 0x01;

    ez_merr_precharge =
        (rxd[5] >> 6) & 0x01;

    ez_merr_dc_contactor =
        (rxd[5] >> 7) & 0x01;

    /* Byte 6 */
    ez_merr_power_valve =
        (rxd[6] >> 0) & 0x01;

    ez_merr_current_sensor =
        (rxd[6] >> 1) & 0x01;

    ez_merr_auto_tune =
        (rxd[6] >> 2) & 0x01;

    ez_merr_rs485 =
        (rxd[6] >> 3) & 0x01;

    ez_merr_can =
        (rxd[6] >> 4) & 0x01;

    ez_merr_software =
        (rxd[6] >> 5) & 0x01;

    /* Byte 7 */
    ez_meter_life = rxd[7];
}

void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *phcan)
{
    if (phcan->Instance != CAN1) return;

    if (HAL_CAN_GetRxMessage(phcan, CAN_RX_FIFO1, &rxh, rxd) != HAL_OK)
        return;

    ez_last_rx_tick = HAL_GetTick();

    if (rxh.IDE != CAN_ID_EXT) return;

    switch (rxh.ExtId)
    {
        case EZK_ID_MCU_TO_VCU_1:
            if (frame_is_8x(0x55))
            {
                ez_handshake_request_count++;
                ez_handshake_established = 0;
                send_handshake_reply();
            }
            else
            {
                parse_vcu_status1();
            }
            break;

        case EZK_ID_MCU_TO_VCU_2:
            parse_vcu_status2();
            break;

        case EZK_ID_MCU_TO_METER_1:
            parse_meter1();
            break;

        case EZK_ID_MCU_TO_METER_2:

            if (rxh.DLC == 8)
            {
                parse_meter2();
            }

            break;

        default:
            break;
    }
}

void EZK_CAN_Test_Init(void)
{
    CAN_FilterTypeDef f = {0};

    /* Pass-all during development so both VCU and optional Meter frames are visible. */
    f.FilterBank = 0;
    f.FilterMode = CAN_FILTERMODE_IDMASK;
    f.FilterScale = CAN_FILTERSCALE_32BIT;
    f.FilterIdHigh = 0;
    f.FilterIdLow = 0;
    f.FilterMaskIdHigh = 0;
    f.FilterMaskIdLow = 0;
    f.FilterFIFOAssignment = CAN_FILTER_FIFO1;
    f.FilterActivation = CAN_FILTER_ENABLE;
    f.SlaveStartFilterBank = 14;

    if (HAL_CAN_ConfigFilter(&hcan, &f) != HAL_OK) Error_Handler();
    if (HAL_CAN_Start(&hcan) != HAL_OK) Error_Handler();
    if (HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO1_MSG_PENDING) != HAL_OK)
        Error_Handler();

    last_command_tick = HAL_GetTick();
}

void EZK_CAN_Test_Task(void)
{
    uint32_t now = HAL_GetTick();

    ez_can_hal_error = HAL_CAN_GetError(&hcan);

    if (!ez_handshake_established) return;
    if ((uint32_t)(now - last_command_tick) < EZK_COMMAND_PERIOD_MS) return;

    last_command_tick = now;

#if EZK_ALLOW_RUN_TEST
    if (ez_test_run_request)
        send_control(1, ez_test_target_phase_current_0p1A, ez_test_target_speed_rpm);
    else
        send_control(0, 0, 0); /* safe HALT */
#else
    send_control(0, 0, 0);     /* SAFE DEFAULT */
#endif
}

/*
 * WHAT TO WATCH IN LIVE EXPRESSIONS
 * ---------------------------------
 * Handshake:
 *   ez_handshake_request_count
 *   ez_handshake_reply_count
 *   ez_handshake_established
 *
 * Motor/controller feedback:
 *   ez_bus_voltage_0p1V   -> 720 means 72.0 V
 *   ez_bus_current_0p1A   -> 25 means 2.5 A
 *   ez_phase_current_0p1A
 *   ez_speed_rpm
 *   ez_controller_temp_C
 *   ez_motor_temp_C
 *   ez_precharge_error
 *   ez_can_comm_error
 *
 * Analog-control visualization, ONLY IF meter frame 0x180217EF is present:
 *   ez_meter2_count       -> must increase
 *   ez_throttle_percent   -> 0..100
 *   ez_brake_active       -> 0/1
 *   ez_operation_mode_raw -> 2 Cruise, 3 EBS, 4 Hold (documented values)
 *   ez_cruise_active
 *   ez_hold_active
 *   ez_dc_contactor_on
 *
 * Three-position SpeedH/SpeedL:
 *   The supplied CAN docs do not define a dedicated SpeedH/SpeedL status field.
 *   The easiest practical verification is therefore motor shaft RPM plus EZ-Tune
 *   configuration/status. Compare ez_speed_rpm in LOW / MIDDLE / HIGH positions.
 *
 * PWM CN1-10:
 *   Not represented in the supplied CAN protocol. Measure the physical pin with
 *   a scope first; do not connect directly to a 3.3V STM32 until voltage is known.
 */
