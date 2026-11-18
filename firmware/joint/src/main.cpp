// One joint of a robot, on a NUCLEO-G474RE.
//   - SimpleFOC closes an angle loop around a 3-PWM bridge on TIM4 (PB6, PB7, PB9, 25 kHz), reading the joint angle from
//     an MT6835 magnetic encoder on SPI1 (PA5/PA6/PA7, chip select PA0).
//   - CAN (FDCAN2, PB12 RX / PB13 TX, 500 kbit/s, classic frames):
//       receives  id 0x200 + JOINT_ID, 4 bytes: float32 little-endian, the target angle in rad from the joint's rest pose
//       sends     id 0x100 + JOINT_ID every 10 ms, 8 bytes: float32 angle (rad from rest), float32 speed (rad/s)
//   - The encoder reads ENC_REST_ANGLE at the rest pose. Keep that away from 0 / 2*pi: an encoder with noise at its wrap point
//     makes SimpleFOC's turn counter jump, and the joint runs a full turn.
// Build: firmware/build.sh. Written for this repo.
#include <SimpleFOC.h>
#include <encoders/mt6835/MagneticSensorMT6835.h>

#ifndef JOINT_ID
#define JOINT_ID 1
#endif
#ifndef ENC_REST_ANGLE
#define ENC_REST_ANGLE 3.14159f
#endif
// A deliberate power-up show: before calibrating, spin the joint open-loop at up to CAL_SPIN_RAD_S (a few full turns), brake, then calibrate
// from wherever it stopped. 0 turns it off.
#ifndef CAL_SPIN_RAD_S
#define CAL_SPIN_RAD_S 0
#endif

static const uint32_t CMD_ID = 0x200 + JOINT_ID;
static const uint32_t STATUS_ID = 0x100 + JOINT_ID;
static const uint32_t STATUS_PERIOD_MS = 10;

BLDCMotor motor = BLDCMotor(7);  // pole pairs
// SimpleFOC's STM32 driver searches the timers on these pins and picks the best combination (TIM4 channels 1, 2 and 4 here).
BLDCDriver3PWM driver = BLDCDriver3PWM(PB6, PB7, PB9);
MagneticSensorMT6835 encoder = MagneticSensorMT6835(PA0, SPISettings(1000000, MSBFIRST, SPI_MODE3));

FDCAN_HandleTypeDef hcan;
FDCAN_TxHeaderTypeDef txHeader;
float cmd = 0;               // target from CAN, rad from the rest pose
float restAbs = ENC_REST_ANGLE;  // SimpleFOC's absolute angle (turns included) at the rest pose nearest the arm after calibration

static bool canInit() {
  __HAL_RCC_FDCAN_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  RCC_PeriphCLKInitTypeDef clk = {};
  clk.PeriphClockSelection = RCC_PERIPHCLK_FDCAN;
  clk.FdcanClockSelection = RCC_FDCANCLKSOURCE_PCLK1;
  HAL_RCCEx_PeriphCLKConfig(&clk);
  GPIO_InitTypeDef pins = {};
  pins.Pin = GPIO_PIN_12 | GPIO_PIN_13;
  pins.Mode = GPIO_MODE_AF_PP;
  pins.Pull = GPIO_NOPULL;
  pins.Speed = GPIO_SPEED_FREQ_LOW;
  pins.Alternate = GPIO_AF9_FDCAN2;
  HAL_GPIO_Init(GPIOB, &pins);

  hcan.Instance = FDCAN2;
  hcan.Init.ClockDivider = FDCAN_CLOCK_DIV1;
  hcan.Init.FrameFormat = FDCAN_FRAME_CLASSIC;
  hcan.Init.Mode = FDCAN_MODE_NORMAL;
  hcan.Init.AutoRetransmission = ENABLE;
  hcan.Init.TransmitPause = DISABLE;
  hcan.Init.ProtocolException = DISABLE;
  hcan.Init.NominalPrescaler = HAL_RCC_GetPCLK1Freq() / (500000 * 10);  // 10 time quanta per bit
  hcan.Init.NominalSyncJumpWidth = 1;
  hcan.Init.NominalTimeSeg1 = 7;
  hcan.Init.NominalTimeSeg2 = 2;
  hcan.Init.DataPrescaler = 1;
  hcan.Init.DataSyncJumpWidth = 1;
  hcan.Init.DataTimeSeg1 = 1;
  hcan.Init.DataTimeSeg2 = 1;
  hcan.Init.StdFiltersNbr = 1;
  hcan.Init.ExtFiltersNbr = 0;
  hcan.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
  if (HAL_FDCAN_Init(&hcan) != HAL_OK) return false;

  FDCAN_FilterTypeDef filter = {};
  filter.IdType = FDCAN_STANDARD_ID;
  filter.FilterIndex = 0;
  filter.FilterType = FDCAN_FILTER_MASK;
  filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
  filter.FilterID1 = CMD_ID;
  filter.FilterID2 = 0x7FF;
  if (HAL_FDCAN_ConfigFilter(&hcan, &filter) != HAL_OK) return false;
  HAL_FDCAN_ConfigGlobalFilter(&hcan, FDCAN_REJECT, FDCAN_REJECT, FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE);

  txHeader.Identifier = STATUS_ID;
  txHeader.IdType = FDCAN_STANDARD_ID;
  txHeader.TxFrameType = FDCAN_DATA_FRAME;
  txHeader.DataLength = FDCAN_DLC_BYTES_8;
  txHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
  txHeader.BitRateSwitch = FDCAN_BRS_OFF;
  txHeader.FDFormat = FDCAN_CLASSIC_CAN;
  txHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
  txHeader.MessageMarker = 0;
  return HAL_FDCAN_Start(&hcan) == HAL_OK;
}

static void canPoll() {
  FDCAN_RxHeaderTypeDef rx;
  uint8_t data[8];
  while (HAL_FDCAN_GetRxFifoFillLevel(&hcan, FDCAN_RX_FIFO0) > 0) {
    if (HAL_FDCAN_GetRxMessage(&hcan, FDCAN_RX_FIFO0, &rx, data) != HAL_OK) break;
    memcpy(&cmd, data, sizeof cmd);
  }
}

static void canReport() {
  static uint32_t last = 0;
  if (millis() - last < STATUS_PERIOD_MS) return;
  last = millis();
  uint8_t data[8];
  float angle = encoder.getAngle() - restAbs, speed = encoder.getVelocity();
  memcpy(data, &angle, 4);
  memcpy(data + 4, &speed, 4);
  HAL_FDCAN_AddMessageToTxFifoQ(&hcan, &txHeader, data);  // a full queue just drops this report
}

#if CAL_SPIN_RAD_S > 0
// Open-loop field rotation at 6 V (the driver allows 8): ramp up for 1.5 s, hold to 3.0 s, ramp down to 3.5 s, then hold the field still
// until 3.9 s so the arm stops. The ramp is gentle enough that the gravity-loaded arm stays in step and goes over the top.
static void spinShow() {
  const float vlimit = motor.voltage_limit;
  motor.voltage_limit = 6;
  motor.controller = MotionControlType::velocity_openloop;
  const uint32_t t0 = millis();
  for (;;) {
    float t = (millis() - t0) / 1000.0f;
    float v = t < 1.5f ? CAL_SPIN_RAD_S * t / 1.5f : t < 3.0f ? CAL_SPIN_RAD_S : t < 3.5f ? CAL_SPIN_RAD_S * (3.5f - t) / 0.5f : 0.0f;
    motor.loopFOC();  // in SimpleFOC 2.4 move() only sets the target; loopFOC() writes the PWM, open loop included
    motor.move(v);
    if (t > 3.9f) break;
  }
  motor.voltage_limit = vlimit;
}
#endif

void setup() {
  Serial.begin(115200);
#ifdef WAIT_FOR_SERIAL
  while (!Serial) {}  // the simulator's USB console attaches after boot; a real joint must not wait for a host
#endif
  SimpleFOCDebug::enable(&Serial);
  encoder.init();
  driver.voltage_power_supply = 12;
  driver.pwm_frequency = 25000;
  driver.voltage_limit = 8;
  if (!driver.init()) { Serial.println("JOINT driver init failed"); return; }
  motor.linkDriver(&driver);
  motor.linkSensor(&encoder);
  motor.voltage_limit = 3;
  motor.foc_modulation = FOCModulationType::SinePWM;
  motor.controller = MotionControlType::angle;
  motor.P_angle.P = 6;
  motor.PID_velocity.P = 1.0;
  motor.PID_velocity.I = 2;
  motor.velocity_limit = 20;
  if (!canInit()) { Serial.println("JOINT CAN init failed"); return; }  // up from boot, so a command sent during calibration is kept
  if (!motor.init()) { Serial.println("JOINT motor init failed"); return; }
#if CAL_SPIN_RAD_S > 0
  spinShow();
  motor.controller = MotionControlType::angle;
#endif
  if (!motor.initFOC()) { Serial.println("JOINT motor init failed"); return; }
  // The encoder reads ENC_REST_ANGLE at the rest pose, modulo a turn. SimpleFOC's angle counts whole turns (the power-up spin adds several), so take
  // the rest pose nearest to where the arm is now: commands are then relative to that, and the arm does not unwind the spin.
  encoder.update();
  float a = encoder.getAngle();
  restAbs = a - (remainderf(a - ENC_REST_ANGLE, _2PI));
  Serial.print("JOINT ");
  Serial.print(JOINT_ID);
  Serial.println(" ready");
}

void loop() {
  motor.loopFOC();
  motor.move(restAbs + cmd);
  canPoll();
  canReport();
}
