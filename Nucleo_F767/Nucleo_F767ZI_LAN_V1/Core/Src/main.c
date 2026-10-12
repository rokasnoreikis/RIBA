/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Robot arm UDP motor control
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "lwip.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "lwip/ip_addr.h"
#include "lwip/pbuf.h"
#include "lwip/udp.h"

#include <stdint.h>
#include <stdio.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define STEP_TOGGLE_MS       1U
#define DIR_SETTLE_MS        2U
#define COMMAND_TIMEOUT_MS   150U

/* M2 mechanical break requires 100 ms to fully release */
#define BREAK_RELEASE_MS     100U

#define START_M1_POSITION   (-4088L)
#define START_M2_POSITION   (4650L)
#define START_M3_POSITION   (-4267L)

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

UART_HandleTypeDef huart3;
PCD_HandleTypeDef hpcd_USB_OTG_FS;

/* USER CODE BEGIN PV */

static struct udp_pcb *udp_pcb_global;

/* Manual mode owns M2 break until explicitly disabled. */
static uint8_t manual_mode_active = 0U;

/* Tracking: bounded simultaneous M1/M3 STEP pulses, no M2 motion. */
#define TRACK_MAX_STEPS 10L
#define TRACK_STEP_TOGGLE_MS 2U
static uint8_t track_active = 0U;
static uint8_t track_step_high = 0U;
static uint8_t track_wait_dir = 0U;
static uint32_t track_dir_time = 0U;
static uint32_t track_last_tick = 0U;
static uint32_t track_slots = 0U;
static uint32_t track_slot = 0U;
static uint32_t track_m1_steps = 0U;
static uint32_t track_m3_steps = 0U;
static uint32_t track_m1_acc = 0U;
static uint32_t track_m3_acc = 0U;
static GPIO_PinState track_m1_dir = GPIO_PIN_RESET;
static GPIO_PinState track_m3_dir = GPIO_PIN_RESET;
static ip_addr_t track_reply_addr;
static u16_t track_reply_port = 0U;



/* ============================================================= */
/* MOTOR 1                                                       */
/* ============================================================= */

static uint8_t m1_running = 0U;
static uint32_t m1_last_command_time = 0U;
static uint32_t m1_last_step_time = 0U;

static GPIO_PinState m1_direction = GPIO_PIN_RESET;

static uint32_t m1_dir_change_time = 0U;
static uint8_t m1_waiting_for_dir = 0U;
static uint8_t m1_step_state = 0U;

static int32_t m1_position = 0;


/* ============================================================= */
/* MOTOR 2                                                       */
/* ============================================================= */

static uint8_t m2_running = 0U;
static uint32_t m2_last_command_time = 0U;
static uint32_t m2_last_step_time = 0U;

static GPIO_PinState m2_direction = GPIO_PIN_RESET;

static uint32_t m2_dir_change_time = 0U;
static uint8_t m2_waiting_for_dir = 0U;
static uint8_t m2_step_state = 0U;

static int32_t m2_position = 0;


/* ============================================================= */
/* MOTOR 3                                                       */
/* ============================================================= */

static uint8_t m3_running = 0U;
static uint32_t m3_last_command_time = 0U;
static uint32_t m3_last_step_time = 0U;

static GPIO_PinState m3_direction = GPIO_PIN_RESET;

static uint32_t m3_dir_change_time = 0U;
static uint8_t m3_waiting_for_dir = 0U;
static uint8_t m3_step_state = 0U;

static int32_t m3_position = 0;


/* ============================================================= */
/* AUTOMATIC COORDINATED MOVE                                    */
/* ============================================================= */

static uint8_t auto_move_active = 0U;
static uint8_t auto_move_waiting_for_dir = 0U;
static uint8_t auto_move_step_high = 0U;

static uint32_t auto_move_dir_time = 0U;
static uint32_t auto_move_last_step_time = 0U;

static uint32_t auto_m1_steps = 0U;
static uint32_t auto_m2_steps = 0U;
static uint32_t auto_m3_steps = 0U;

static GPIO_PinState auto_m1_direction = GPIO_PIN_RESET;
static GPIO_PinState auto_m2_direction = GPIO_PIN_RESET;
static GPIO_PinState auto_m3_direction = GPIO_PIN_RESET;

static uint32_t auto_total_slots = 0U;
static uint32_t auto_slot = 0U;

static uint32_t auto_m1_accumulator = 0U;
static uint32_t auto_m2_accumulator = 0U;
static uint32_t auto_m3_accumulator = 0U;

static ip_addr_t auto_reply_addr;
static u16_t auto_reply_port = 0U;
static uint8_t auto_reply_valid = 0U;

/* USER CODE END PV */


/* Private function prototypes -----------------------------------------------*/

void SystemClock_Config(void);
static void MPU_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_USB_OTG_FS_PCD_Init(void);

/* USER CODE BEGIN PFP */

static void Track_Stop(void);
static void Track_Start(int32_t m1_steps, int32_t m3_steps,
                        const ip_addr_t *addr, u16_t port);
static void Track_Process(void);
static void UDP_Init(void);

static void UDP_ReceiveCallback(
    void *arg,
    struct udp_pcb *pcb,
    struct pbuf *p,
    const ip_addr_t *addr,
    u16_t port
);

static void UDP_SendReply(
    const char *message,
    const ip_addr_t *addr,
    u16_t port
);

static void UDP_SendPosition(
    const ip_addr_t *addr,
    u16_t port
);

static void M1_Start(GPIO_PinState direction);
static void M1_Stop(void);
static void M1_Process(void);

static void M2_Start(GPIO_PinState direction);
static void M2_Stop(void);
static void M2_Process(void);

static void M3_Start(GPIO_PinState direction);
static void M3_Stop(void);
static void M3_Process(void);

static void Stop_All_Motors(void);

static void Reset_Position(void);

static void M2_BreakRelease(void);
static void M2_BreakEngage(void);

static void Auto_MoveStart(
    int32_t target_m1,
    int32_t target_m2,
    int32_t target_m3,
    const ip_addr_t *addr,
    u16_t port
);

static void Auto_MoveProcess(void);
static void Auto_MoveStop(void);

static uint32_t Abs_Difference(
    int32_t target,
    int32_t current
);

/* USER CODE END PFP */


/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


/* ============================================================= */
/* MOTOR 2 BREAK                                                 */
/* ============================================================= */

static void M2_BreakRelease(void)
{
    HAL_GPIO_WritePin(
        M2_break_GPIO_Port,
        M2_break_Pin,
        GPIO_PIN_SET
    );
}


static void M2_BreakEngage(void)
{
    HAL_GPIO_WritePin(
        M2_break_GPIO_Port,
        M2_break_Pin,
        GPIO_PIN_RESET
    );
}


/* ============================================================= */
/* TRACKING: bounded coordinated M1/M3 move                     */
/* ============================================================= */
static void Track_Stop(void)
{
    track_active = 0U;
    track_step_high = 0U;
    track_wait_dir = 0U;
    HAL_GPIO_WritePin(M1_step_GPIO_Port, M1_step_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(M3_step_GPIO_Port, M3_step_Pin, GPIO_PIN_RESET);
}

static void Track_Start(int32_t m1_steps, int32_t m3_steps,
                        const ip_addr_t *addr, u16_t port)
{
    uint32_t now;
    /* Never interrupt manual mode, jog movement or GO TO START. */
    if (manual_mode_active || auto_move_active ||
        m1_running || m2_running || m3_running || track_active)
    {
        UDP_SendReply("TRACK_BUSY", addr, port);
        return;
    }

    if (m1_steps > TRACK_MAX_STEPS || m1_steps < -TRACK_MAX_STEPS ||
        m3_steps > TRACK_MAX_STEPS || m3_steps < -TRACK_MAX_STEPS)
    {
        UDP_SendReply("TRACK_INVALID", addr, port);
        return;
    }

    track_m1_steps = (uint32_t)(m1_steps < 0 ? -m1_steps : m1_steps);
    track_m3_steps = (uint32_t)(m3_steps < 0 ? -m3_steps : m3_steps);
    track_slots = track_m1_steps > track_m3_steps ? track_m1_steps : track_m3_steps;
    if (track_slots == 0U)
    {
        UDP_SendReply("TRACK_DONE", addr, port);
        return;
    }

    track_m1_dir = m1_steps >= 0 ? GPIO_PIN_SET : GPIO_PIN_RESET;
    track_m3_dir = m3_steps >= 0 ? GPIO_PIN_SET : GPIO_PIN_RESET;
    HAL_GPIO_WritePin(M1_step_GPIO_Port, M1_step_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(M3_step_GPIO_Port, M3_step_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(M1_dir_GPIO_Port, M1_dir_Pin, track_m1_dir);
    HAL_GPIO_WritePin(M3_dir_GPIO_Port, M3_dir_Pin, track_m3_dir);
    m1_direction = track_m1_dir;
    m3_direction = track_m3_dir;
    track_m1_acc = track_slots / 2U;
    track_m3_acc = track_slots / 2U;
    track_slot = 0U;
    track_step_high = 0U;
    now = HAL_GetTick();
    track_dir_time = now;
    track_last_tick = now;
    track_wait_dir = 1U;
    ip_addr_copy(track_reply_addr, *addr);
    track_reply_port = port;
    track_active = 1U;
}

static void Track_Process(void)
{
    uint32_t now;
    uint8_t pulse1 = 0U, pulse3 = 0U;
    if (!track_active) return;
    now = HAL_GetTick();
    if (track_wait_dir)
    {
        if ((uint32_t)(now - track_dir_time) < DIR_SETTLE_MS) return;
        track_wait_dir = 0U;
        track_last_tick = now;
        return;
    }
    if ((uint32_t)(now - track_last_tick) < TRACK_STEP_TOGGLE_MS) return;
    track_last_tick = now;

    if (!track_step_high)
    {
        track_m1_acc += track_m1_steps;
        if (track_m1_acc >= track_slots)
        {
            track_m1_acc -= track_slots;
            pulse1 = 1U;
        }
        track_m3_acc += track_m3_steps;
        if (track_m3_acc >= track_slots)
        {
            track_m3_acc -= track_slots;
            pulse3 = 1U;
        }
        if (pulse1)
        {
            HAL_GPIO_WritePin(M1_step_GPIO_Port, M1_step_Pin, GPIO_PIN_SET);
            m1_position += (track_m1_dir == GPIO_PIN_SET) ? 1 : -1;
        }
        if (pulse3)
        {
            HAL_GPIO_WritePin(M3_step_GPIO_Port, M3_step_Pin, GPIO_PIN_SET);
            m3_position += (track_m3_dir == GPIO_PIN_SET) ? 1 : -1;
        }
        track_step_high = 1U;
        track_slot++;
    }
    else
    {
        HAL_GPIO_WritePin(M1_step_GPIO_Port, M1_step_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(M3_step_GPIO_Port, M3_step_Pin, GPIO_PIN_RESET);
        track_step_high = 0U;
        if (track_slot >= track_slots)
        {
            track_active = 0U;
            UDP_SendReply("TRACK_DONE", &track_reply_addr, track_reply_port);
        }
    }
}

/* ============================================================= */
/* UDP INITIALIZATION                                            */
/* ============================================================= */

static void UDP_Init(void)
{
    udp_pcb_global = udp_new();

    if (udp_pcb_global == NULL)
    {
        return;
    }

    if (
        udp_bind(
            udp_pcb_global,
            IP_ADDR_ANY,
            5001
        ) != ERR_OK
    )
    {
        udp_remove(udp_pcb_global);

        udp_pcb_global = NULL;

        return;
    }

    udp_recv(
        udp_pcb_global,
        UDP_ReceiveCallback,
        NULL
    );
}


/* ============================================================= */
/* UDP REPLY                                                     */
/* ============================================================= */

static void UDP_SendReply(
    const char *message,
    const ip_addr_t *addr,
    u16_t port
)
{
    struct pbuf *p;
    size_t length;

    if (udp_pcb_global == NULL)
    {
        return;
    }

    if (message == NULL)
    {
        return;
    }

    length = strlen(message);

    p = pbuf_alloc(
        PBUF_TRANSPORT,
        length,
        PBUF_RAM
    );

    if (p == NULL)
    {
        return;
    }

    memcpy(
        p->payload,
        message,
        length
    );

    udp_sendto(
        udp_pcb_global,
        p,
        addr,
        port
    );

    pbuf_free(p);
}


/* ============================================================= */
/* SEND POSITION                                                 */
/* ============================================================= */

static void UDP_SendPosition(
    const ip_addr_t *addr,
    u16_t port
)
{
    char message[100];

    snprintf(
        message,
        sizeof(message),
        "POS,%ld,%ld,%ld",
        (long)m1_position,
        (long)m2_position,
        (long)m3_position
    );

    UDP_SendReply(
        message,
        addr,
        port
    );
}


/* ============================================================= */
/* RESET POSITION                                                */
/* ============================================================= */

static void Reset_Position(void)
{
    m1_position = 0;
    m2_position = 0;
    m3_position = 0;
}


/* ============================================================= */
/* ABSOLUTE DIFFERENCE                                           */
/* ============================================================= */

static uint32_t Abs_Difference(
    int32_t target,
    int32_t current
)
{
    int64_t difference;

    difference =
        (int64_t)target -
        (int64_t)current;

    if (difference < 0)
    {
        difference = -difference;
    }

    return (uint32_t)difference;
}


/* ============================================================= */
/* MOTOR 1 STOP                                                  */
/* ============================================================= */

static void M1_Stop(void)
{
    m1_running = 0U;
    m1_waiting_for_dir = 0U;
    m1_step_state = 0U;

    HAL_GPIO_WritePin(
        M1_step_GPIO_Port,
        M1_step_Pin,
        GPIO_PIN_RESET
    );
}


/* ============================================================= */
/* MOTOR 2 STOP                                                  */
/* ============================================================= */

static void M2_Stop(void)
{
    m2_running = 0U;
    m2_waiting_for_dir = 0U;
    m2_step_state = 0U;

    HAL_GPIO_WritePin(
        M2_step_GPIO_Port,
        M2_step_Pin,
        GPIO_PIN_RESET
    );
}


/* ============================================================= */
/* MOTOR 3 STOP                                                  */
/* ============================================================= */

static void M3_Stop(void)
{
    m3_running = 0U;
    m3_waiting_for_dir = 0U;
    m3_step_state = 0U;

    HAL_GPIO_WritePin(
        M3_step_GPIO_Port,
        M3_step_Pin,
        GPIO_PIN_RESET
    );
}


/* ============================================================= */
/* STOP ALL MANUAL MOTORS                                        */
/* ============================================================= */

static void Stop_All_Motors(void)
{
    M1_Stop();
    M2_Stop();
    M3_Stop();
}


/* ============================================================= */
/* MOTOR 1 START                                                 */
/* ============================================================= */

static void M1_Start(
    GPIO_PinState direction
)
{
    uint32_t now;

    if (auto_move_active != 0U)
    {
        Auto_MoveStop();
    }

    now = HAL_GetTick();

    if (
        (m1_running == 0U) ||
        (m1_direction != direction)
    )
    {
        HAL_GPIO_WritePin(
            M1_step_GPIO_Port,
            M1_step_Pin,
            GPIO_PIN_RESET
        );

        m1_step_state = 0U;

        HAL_GPIO_WritePin(
            M1_dir_GPIO_Port,
            M1_dir_Pin,
            direction
        );

        m1_direction = direction;
        m1_dir_change_time = now;
        m1_last_step_time = now;
        m1_waiting_for_dir = 1U;
    }

    m1_last_command_time = now;
    m1_running = 1U;
}


/* ============================================================= */
/* MOTOR 1 PROCESS                                               */
/* ============================================================= */

static void M1_Process(void)
{
    uint32_t now;

    if (m1_running == 0U)
    {
        return;
    }

    now = HAL_GetTick();

    if (
        (now - m1_last_command_time) >=
        COMMAND_TIMEOUT_MS
    )
    {
        M1_Stop();
        return;
    }

    if (m1_waiting_for_dir != 0U)
    {
        if (
            (now - m1_dir_change_time) <
            DIR_SETTLE_MS
        )
        {
            return;
        }

        m1_waiting_for_dir = 0U;
        m1_last_step_time = now;

        return;
    }

    if (
        (now - m1_last_step_time) >=
        STEP_TOGGLE_MS
    )
    {
        m1_last_step_time = now;

        if (m1_step_state == 0U)
        {
            HAL_GPIO_WritePin(
                M1_step_GPIO_Port,
                M1_step_Pin,
                GPIO_PIN_SET
            );

            m1_step_state = 1U;

            if (m1_direction == GPIO_PIN_SET)
            {
                m1_position++;
            }
            else
            {
                m1_position--;
            }
        }
        else
        {
            HAL_GPIO_WritePin(
                M1_step_GPIO_Port,
                M1_step_Pin,
                GPIO_PIN_RESET
            );

            m1_step_state = 0U;
        }
    }
}


/* ============================================================= */
/* MOTOR 2 START                                                 */
/* ============================================================= */

static void M2_Start(
    GPIO_PinState direction
)
{
    uint32_t now;

    if (auto_move_active != 0U)
    {
        Auto_MoveStop();
    }

    now = HAL_GetTick();

    if (
        (m2_running == 0U) ||
        (m2_direction != direction)
    )
    {
        HAL_GPIO_WritePin(
            M2_step_GPIO_Port,
            M2_step_Pin,
            GPIO_PIN_RESET
        );

        m2_step_state = 0U;

        HAL_GPIO_WritePin(
            M2_dir_GPIO_Port,
            M2_dir_Pin,
            direction
        );

        m2_direction = direction;
        m2_dir_change_time = now;
        m2_last_step_time = now;
        m2_waiting_for_dir = 1U;
    }

    m2_last_command_time = now;
    m2_running = 1U;
}


/* ============================================================= */
/* MOTOR 2 PROCESS                                               */
/* ============================================================= */

static void M2_Process(void)
{
    uint32_t now;

    if (m2_running == 0U)
    {
        return;
    }

    now = HAL_GetTick();

    if (
        (now - m2_last_command_time) >=
        COMMAND_TIMEOUT_MS
    )
    {
        M2_Stop();
        return;
    }

    if (m2_waiting_for_dir != 0U)
    {
        if (
            (now - m2_dir_change_time) <
            DIR_SETTLE_MS
        )
        {
            return;
        }

        m2_waiting_for_dir = 0U;
        m2_last_step_time = now;

        return;
    }

    if (
        (now - m2_last_step_time) >=
        STEP_TOGGLE_MS
    )
    {
        m2_last_step_time = now;

        if (m2_step_state == 0U)
        {
            HAL_GPIO_WritePin(
                M2_step_GPIO_Port,
                M2_step_Pin,
                GPIO_PIN_SET
            );

            m2_step_state = 1U;

            if (m2_direction == GPIO_PIN_SET)
            {
                m2_position++;
            }
            else
            {
                m2_position--;
            }
        }
        else
        {
            HAL_GPIO_WritePin(
                M2_step_GPIO_Port,
                M2_step_Pin,
                GPIO_PIN_RESET
            );

            m2_step_state = 0U;
        }
    }
}


/* ============================================================= */
/* MOTOR 3 START                                                 */
/* ============================================================= */

static void M3_Start(
    GPIO_PinState direction
)
{
    uint32_t now;

    if (auto_move_active != 0U)
    {
        Auto_MoveStop();
    }

    now = HAL_GetTick();

    if (
        (m3_running == 0U) ||
        (m3_direction != direction)
    )
    {
        HAL_GPIO_WritePin(
            M3_step_GPIO_Port,
            M3_step_Pin,
            GPIO_PIN_RESET
        );

        m3_step_state = 0U;

        HAL_GPIO_WritePin(
            M3_dir_GPIO_Port,
            M3_dir_Pin,
            direction
        );

        m3_direction = direction;
        m3_dir_change_time = now;
        m3_last_step_time = now;
        m3_waiting_for_dir = 1U;
    }

    m3_last_command_time = now;
    m3_running = 1U;
}


/* ============================================================= */
/* MOTOR 3 PROCESS                                               */
/* ============================================================= */

static void M3_Process(void)
{
    uint32_t now;

    if (m3_running == 0U)
    {
        return;
    }

    now = HAL_GetTick();

    if (
        (now - m3_last_command_time) >=
        COMMAND_TIMEOUT_MS
    )
    {
        M3_Stop();
        return;
    }

    if (m3_waiting_for_dir != 0U)
    {
        if (
            (now - m3_dir_change_time) <
            DIR_SETTLE_MS
        )
        {
            return;
        }

        m3_waiting_for_dir = 0U;
        m3_last_step_time = now;

        return;
    }

    if (
        (now - m3_last_step_time) >=
        STEP_TOGGLE_MS
    )
    {
        m3_last_step_time = now;

        if (m3_step_state == 0U)
        {
            HAL_GPIO_WritePin(
                M3_step_GPIO_Port,
                M3_step_Pin,
                GPIO_PIN_SET
            );

            m3_step_state = 1U;

            if (m3_direction == GPIO_PIN_SET)
            {
                m3_position++;
            }
            else
            {
                m3_position--;
            }
        }
        else
        {
            HAL_GPIO_WritePin(
                M3_step_GPIO_Port,
                M3_step_Pin,
                GPIO_PIN_RESET
            );

            m3_step_state = 0U;
        }
    }
}


/* ============================================================= */
/* AUTOMATIC MOVE STOP                                           */
/* ============================================================= */

static void Auto_MoveStop(void)
{
    auto_move_active = 0U;
    auto_move_waiting_for_dir = 0U;
    auto_move_step_high = 0U;
    auto_reply_valid = 0U;

    HAL_GPIO_WritePin(
        M1_step_GPIO_Port,
        M1_step_Pin,
        GPIO_PIN_RESET
    );

    HAL_GPIO_WritePin(
        M2_step_GPIO_Port,
        M2_step_Pin,
        GPIO_PIN_RESET
    );

    HAL_GPIO_WritePin(
        M3_step_GPIO_Port,
        M3_step_Pin,
        GPIO_PIN_RESET
    );

    m1_step_state = 0U;
    m2_step_state = 0U;
    m3_step_state = 0U;
}


/* ============================================================= */
/* START AUTOMATIC MOVE                                          */
/* ============================================================= */

static void Auto_MoveStart(
    int32_t target_m1,
    int32_t target_m2,
    int32_t target_m3,
    const ip_addr_t *addr,
    u16_t port
)
{
    uint32_t now;

    Stop_All_Motors();
    Auto_MoveStop();


    /*
     * Calculate movement from CURRENT position
     * to the requested target position.
     */

    auto_m1_steps =
        Abs_Difference(
            target_m1,
            m1_position
        );

    auto_m2_steps =
        Abs_Difference(
            target_m2,
            m2_position
        );

    auto_m3_steps =
        Abs_Difference(
            target_m3,
            m3_position
        );


    if (target_m1 >= m1_position)
    {
        auto_m1_direction = GPIO_PIN_SET;
    }
    else
    {
        auto_m1_direction = GPIO_PIN_RESET;
    }


    if (target_m2 >= m2_position)
    {
        auto_m2_direction = GPIO_PIN_SET;
    }
    else
    {
        auto_m2_direction = GPIO_PIN_RESET;
    }


    if (target_m3 >= m3_position)
    {
        auto_m3_direction = GPIO_PIN_SET;
    }
    else
    {
        auto_m3_direction = GPIO_PIN_RESET;
    }


    auto_total_slots = auto_m1_steps;


    if (auto_m2_steps > auto_total_slots)
    {
        auto_total_slots = auto_m2_steps;
    }


    if (auto_m3_steps > auto_total_slots)
    {
        auto_total_slots = auto_m3_steps;
    }


    /*
     * Robot is already at the requested position.
     */

    if (auto_total_slots == 0U)
    {
        if (!manual_mode_active) M2_BreakEngage();

        UDP_SendReply(
            "MOVE_DONE",
            addr,
            port
        );

        UDP_SendPosition(
            addr,
            port
        );

        return;
    }


    /*
     * Release M2 mechanical break FIRST.
     */

    if (!manual_mode_active) M2_BreakRelease();


    /*
     * IMPORTANT:
     *
     * Give the mechanical break 100 ms to fully disengage
     * before ANY STEP pulse is generated.
     */

    if (!manual_mode_active) HAL_Delay(BREAK_RELEASE_MS);


    /*
     * Set all directions after the break has released.
     */

    HAL_GPIO_WritePin(
        M1_dir_GPIO_Port,
        M1_dir_Pin,
        auto_m1_direction
    );

    HAL_GPIO_WritePin(
        M2_dir_GPIO_Port,
        M2_dir_Pin,
        auto_m2_direction
    );

    HAL_GPIO_WritePin(
        M3_dir_GPIO_Port,
        M3_dir_Pin,
        auto_m3_direction
    );


    m1_direction = auto_m1_direction;
    m2_direction = auto_m2_direction;
    m3_direction = auto_m3_direction;


    /*
     * Ensure all STEP outputs are LOW.
     */

    HAL_GPIO_WritePin(
        M1_step_GPIO_Port,
        M1_step_Pin,
        GPIO_PIN_RESET
    );

    HAL_GPIO_WritePin(
        M2_step_GPIO_Port,
        M2_step_Pin,
        GPIO_PIN_RESET
    );

    HAL_GPIO_WritePin(
        M3_step_GPIO_Port,
        M3_step_Pin,
        GPIO_PIN_RESET
    );


    /*
     * Initialize DDA accumulators.
     */

    auto_m1_accumulator =
        auto_total_slots / 2U;

    auto_m2_accumulator =
        auto_total_slots / 2U;

    auto_m3_accumulator =
        auto_total_slots / 2U;


    auto_slot = 0U;
    auto_move_step_high = 0U;


    now = HAL_GetTick();

    auto_move_dir_time = now;
    auto_move_last_step_time = now;

    auto_move_waiting_for_dir = 1U;


    ip_addr_copy(
        auto_reply_addr,
        *addr
    );


    auto_reply_port = port;
    auto_reply_valid = 1U;

    auto_move_active = 1U;
}


/* ============================================================= */
/* PROCESS AUTOMATIC MOVE                                        */
/* ============================================================= */

static void Auto_MoveProcess(void)
{
    uint32_t now;

    uint8_t m1_pulse;
    uint8_t m2_pulse;
    uint8_t m3_pulse;


    if (auto_move_active == 0U)
    {
        return;
    }


    now = HAL_GetTick();


    /*
     * Wait for DIR signals to settle.
     */

    if (auto_move_waiting_for_dir != 0U)
    {
        if (
            (now - auto_move_dir_time) <
            DIR_SETTLE_MS
        )
        {
            return;
        }

        auto_move_waiting_for_dir = 0U;
        auto_move_last_step_time = now;

        return;
    }


    if (
        (now - auto_move_last_step_time) <
        STEP_TOGGLE_MS
    )
    {
        return;
    }


    auto_move_last_step_time = now;


    /*
     * LOW -> HIGH phase.
     */

    if (auto_move_step_high == 0U)
    {
        m1_pulse = 0U;
        m2_pulse = 0U;
        m3_pulse = 0U;


        auto_m1_accumulator +=
            auto_m1_steps;


        if (
            auto_m1_accumulator >=
            auto_total_slots
        )
        {
            auto_m1_accumulator -=
                auto_total_slots;

            m1_pulse = 1U;
        }


        auto_m2_accumulator +=
            auto_m2_steps;


        if (
            auto_m2_accumulator >=
            auto_total_slots
        )
        {
            auto_m2_accumulator -=
                auto_total_slots;

            m2_pulse = 1U;
        }


        auto_m3_accumulator +=
            auto_m3_steps;


        if (
            auto_m3_accumulator >=
            auto_total_slots
        )
        {
            auto_m3_accumulator -=
                auto_total_slots;

            m3_pulse = 1U;
        }


        /*
         * Generate rising STEP edges.
         * Position is updated only on rising edges.
         */

        if (m1_pulse != 0U)
        {
            HAL_GPIO_WritePin(
                M1_step_GPIO_Port,
                M1_step_Pin,
                GPIO_PIN_SET
            );

            if (
                auto_m1_direction ==
                GPIO_PIN_SET
            )
            {
                m1_position++;
            }
            else
            {
                m1_position--;
            }
        }


        if (m2_pulse != 0U)
        {
            HAL_GPIO_WritePin(
                M2_step_GPIO_Port,
                M2_step_Pin,
                GPIO_PIN_SET
            );

            if (
                auto_m2_direction ==
                GPIO_PIN_SET
            )
            {
                m2_position++;
            }
            else
            {
                m2_position--;
            }
        }


        if (m3_pulse != 0U)
        {
            HAL_GPIO_WritePin(
                M3_step_GPIO_Port,
                M3_step_Pin,
                GPIO_PIN_SET
            );

            if (
                auto_m3_direction ==
                GPIO_PIN_SET
            )
            {
                m3_position++;
            }
            else
            {
                m3_position--;
            }
        }


        auto_slot++;

        auto_move_step_high = 1U;
    }

    /*
     * HIGH -> LOW phase.
     */

    else
    {
        HAL_GPIO_WritePin(
            M1_step_GPIO_Port,
            M1_step_Pin,
            GPIO_PIN_RESET
        );

        HAL_GPIO_WritePin(
            M2_step_GPIO_Port,
            M2_step_Pin,
            GPIO_PIN_RESET
        );

        HAL_GPIO_WritePin(
            M3_step_GPIO_Port,
            M3_step_Pin,
            GPIO_PIN_RESET
        );


        auto_move_step_high = 0U;


        /*
         * Automatic movement finished.
         */

        if (auto_slot >= auto_total_slots)
        {
            auto_move_active = 0U;


            /*
             * M2 is no longer moving.
             * Engage the mechanical break.
             */

            if (!manual_mode_active) M2_BreakEngage();


            if (auto_reply_valid != 0U)
            {
                UDP_SendReply(
                    "MOVE_DONE",
                    &auto_reply_addr,
                    auto_reply_port
                );


                UDP_SendPosition(
                    &auto_reply_addr,
                    auto_reply_port
                );
            }


            auto_reply_valid = 0U;
        }
    }
}


/* ============================================================= */
/* UDP RECEIVE                                                   */
/* ============================================================= */

static void UDP_ReceiveCallback(
    void *arg,
    struct udp_pcb *pcb,
    struct pbuf *p,
    const ip_addr_t *addr,
    u16_t port
)
{
    char command[64];

    uint16_t copy_length;


    if (p == NULL)
    {
        return;
    }


    copy_length = p->tot_len;


    if (copy_length >= sizeof(command))
    {
        copy_length =
            sizeof(command) - 1U;
    }


    pbuf_copy_partial(
        p,
        command,
        copy_length,
        0
    );


    command[copy_length] = '\0';


    /* --------------------------------------------------------- */
    /* CONNECT                                                   */
    /* --------------------------------------------------------- */

    if (
        strcmp(
            command,
            "CONNECT"
        ) == 0
    )
    {
        UDP_SendReply(
            "CONNECT_OK",
            addr,
            port
        );
    }


    /* --------------------------------------------------------- */
    /* MANUAL MODE ON                                            */
    /* --------------------------------------------------------- */

    else if (
        strcmp(
            command,
            "MANUAL_MODE_ON"
        ) == 0
    )
    {
        Track_Stop();
        Auto_MoveStop();

        Stop_All_Motors();


        /*
         * Release M2 break.
         */

        manual_mode_active = 1U;
        M2_BreakRelease();


        /*
         * Wait until the mechanical break has
         * fully disengaged.
         */

        HAL_Delay(
            BREAK_RELEASE_MS
        );


        UDP_SendReply(
            "MANUAL_MODE_ON",
            addr,
            port
        );
    }


    /* --------------------------------------------------------- */
    /* MANUAL MODE OFF                                           */
    /* --------------------------------------------------------- */

    else if (
        strcmp(
            command,
            "MANUAL_MODE_OFF"
        ) == 0
    )
    {
        Track_Stop();
        Auto_MoveStop();

        Stop_All_Motors();

        manual_mode_active = 0U;
        M2_BreakEngage();


        UDP_SendReply(
            "MANUAL_MODE_OFF",
            addr,
            port
        );
    }


    /* --------------------------------------------------------- */
    /* MOTOR 1 LEFT                                              */
    /* --------------------------------------------------------- */

    else if (
        strcmp(
            command,
            "M1_LEFT"
        ) == 0
    )
    {
        Track_Stop();
        Auto_MoveStop();

        M2_Stop();
        M3_Stop();


        M1_Start(
            GPIO_PIN_RESET
        );


        UDP_SendReply(
            "M1_LEFT",
            addr,
            port
        );
    }


    /* --------------------------------------------------------- */
    /* MOTOR 1 RIGHT                                             */
    /* --------------------------------------------------------- */

    else if (
        strcmp(
            command,
            "M1_RIGHT"
        ) == 0
    )
    {
        Track_Stop();
        Auto_MoveStop();

        M2_Stop();
        M3_Stop();


        M1_Start(
            GPIO_PIN_SET
        );


        UDP_SendReply(
            "M1_RIGHT",
            addr,
            port
        );
    }


    /* --------------------------------------------------------- */
    /* MOTOR 2 UP                                                */
    /* --------------------------------------------------------- */

    else if (
        strcmp(
            command,
            "M2_UP"
        ) == 0
    )
    {
        Track_Stop();
        Auto_MoveStop();

        M1_Stop();
        M3_Stop();


        M2_Start(
            GPIO_PIN_SET
        );


        UDP_SendReply(
            "M2_UP",
            addr,
            port
        );
    }


    /* --------------------------------------------------------- */
    /* MOTOR 2 DOWN                                              */
    /* --------------------------------------------------------- */

    else if (
        strcmp(
            command,
            "M2_DOWN"
        ) == 0
    )
    {
        Track_Stop();
        Auto_MoveStop();

        M1_Stop();
        M3_Stop();


        M2_Start(
            GPIO_PIN_RESET
        );


        UDP_SendReply(
            "M2_DOWN",
            addr,
            port
        );
    }


    /* --------------------------------------------------------- */
    /* MOTOR 3 UP                                                */
    /* --------------------------------------------------------- */

    else if (
        strcmp(
            command,
            "M3_UP"
        ) == 0
    )
    {
        Track_Stop();
        Auto_MoveStop();

        M1_Stop();
        M2_Stop();


        M3_Start(
            GPIO_PIN_SET
        );


        UDP_SendReply(
            "M3_UP",
            addr,
            port
        );
    }


    /* --------------------------------------------------------- */
    /* MOTOR 3 DOWN                                              */
    /* --------------------------------------------------------- */

    else if (
        strcmp(
            command,
            "M3_DOWN"
        ) == 0
    )
    {
        Track_Stop();
        Auto_MoveStop();

        M1_Stop();
        M2_Stop();


        M3_Start(
            GPIO_PIN_RESET
        );


        UDP_SendReply(
            "M3_DOWN",
            addr,
            port
        );
    }


    /* --------------------------------------------------------- */
    /* GO TO START                                               */
    /* --------------------------------------------------------- */

    else if (
        strcmp(
            command,
            "GO_TO_START"
        ) == 0
    )
    {
        Track_Stop();
        /*
         * Auto_MoveStart:
         *
         * 1. Calculates current -> START
         * 2. Releases M2 break
         * 3. Waits 100 ms
         * 4. Sets directions
         * 5. Starts coordinated movement
         */

        Auto_MoveStart(
            START_M1_POSITION,
            START_M2_POSITION,
            START_M3_POSITION,
            addr,
            port
        );


        if (auto_move_active != 0U)
        {
            UDP_SendReply(
                "MOVE_STARTED",
                addr,
                port
            );
        }
    }


    /* --------------------------------------------------------- */
    /* TRACK STEP: signed M1/M3 step counts (max 10 each)        */
    /* --------------------------------------------------------- */
    else if (strncmp(command, "TRACK_STEP,", 11) == 0)
    {
        long steps1, steps3;
        char extra;
        if (sscanf(command, "TRACK_STEP,%ld,%ld%c", &steps1, &steps3, &extra) != 2 ||
            steps1 < -TRACK_MAX_STEPS || steps1 > TRACK_MAX_STEPS ||
            steps3 < -TRACK_MAX_STEPS || steps3 > TRACK_MAX_STEPS)
        {
            UDP_SendReply("TRACK_INVALID", addr, port);
        }
        else
        {
            Track_Start((int32_t)steps1, (int32_t)steps3, addr, port);
        }
    }

    /* --------------------------------------------------------- */
    /* STOP                                                      */
    /* --------------------------------------------------------- */

    else if (
        strcmp(
            command,
            "STOP"
        ) == 0
    )
    {
        Track_Stop();
        Auto_MoveStop();
        Stop_All_Motors();

        /* Keep break released during Manual Mode. */
        if (!manual_mode_active) M2_BreakEngage();


        UDP_SendReply(
            "STOP",
            addr,
            port
        );
    }


    /* --------------------------------------------------------- */
    /* ZERO SET                                                  */
    /* --------------------------------------------------------- */

    else if (
        strcmp(
            command,
            "ZERO_SET"
        ) == 0
    )
    {
        Track_Stop();
        Auto_MoveStop();

        Stop_All_Motors();


        Reset_Position();


        UDP_SendReply(
            "ZERO_SET",
            addr,
            port
        );
    }


    /* --------------------------------------------------------- */
    /* GET POSITION                                              */
    /* --------------------------------------------------------- */

    else if (
        strcmp(
            command,
            "GET_POS"
        ) == 0
    )
    {
        UDP_SendPosition(
            addr,
            port
        );
    }


    /* --------------------------------------------------------- */
    /* UNKNOWN                                                   */
    /* --------------------------------------------------------- */

    else
    {
        UDP_SendReply(
            "UNKNOWN_COMMAND",
            addr,
            port
        );
    }


    pbuf_free(p);
}


/* USER CODE END 0 */


/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
    MPU_Config();

    HAL_Init();

    SystemClock_Config();

    MX_GPIO_Init();

    MX_USART3_UART_Init();

    MX_USB_OTG_FS_PCD_Init();

    MX_LWIP_Init();


    /* USER CODE BEGIN 2 */

    Stop_All_Motors();

    Auto_MoveStop();
    Track_Stop();
    manual_mode_active = 0U;

    Reset_Position();

    M2_BreakEngage();

    UDP_Init();

    /* USER CODE END 2 */


    while (1)
    {
        MX_LWIP_Process();

        M1_Process();

        M2_Process();

        M3_Process();

        Auto_MoveProcess();
        Track_Process();
    }
}


/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};

    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};


    __HAL_RCC_PWR_CLK_ENABLE();


    __HAL_PWR_VOLTAGESCALING_CONFIG(
        PWR_REGULATOR_VOLTAGE_SCALE1
    );


    RCC_OscInitStruct.OscillatorType =
        RCC_OSCILLATORTYPE_HSE;


    RCC_OscInitStruct.HSEState =
        RCC_HSE_BYPASS;


    RCC_OscInitStruct.PLL.PLLState =
        RCC_PLL_ON;


    RCC_OscInitStruct.PLL.PLLSource =
        RCC_PLLSOURCE_HSE;


    RCC_OscInitStruct.PLL.PLLM = 4;


    RCC_OscInitStruct.PLL.PLLN = 96;


    RCC_OscInitStruct.PLL.PLLP =
        RCC_PLLP_DIV2;


    RCC_OscInitStruct.PLL.PLLQ = 4;


    if (
        HAL_RCC_OscConfig(
            &RCC_OscInitStruct
        ) != HAL_OK
    )
    {
        Error_Handler();
    }


    if (
        HAL_PWREx_EnableOverDrive()
        != HAL_OK
    )
    {
        Error_Handler();
    }


    RCC_ClkInitStruct.ClockType =
        RCC_CLOCKTYPE_HCLK |
        RCC_CLOCKTYPE_SYSCLK |
        RCC_CLOCKTYPE_PCLK1 |
        RCC_CLOCKTYPE_PCLK2;


    RCC_ClkInitStruct.SYSCLKSource =
        RCC_SYSCLKSOURCE_PLLCLK;


    RCC_ClkInitStruct.AHBCLKDivider =
        RCC_SYSCLK_DIV1;


    RCC_ClkInitStruct.APB1CLKDivider =
        RCC_HCLK_DIV2;


    RCC_ClkInitStruct.APB2CLKDivider =
        RCC_HCLK_DIV1;


    if (
        HAL_RCC_ClockConfig(
            &RCC_ClkInitStruct,
            FLASH_LATENCY_3
        ) != HAL_OK
    )
    {
        Error_Handler();
    }
}


/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{
    huart3.Instance = USART3;

    huart3.Init.BaudRate = 115200;

    huart3.Init.WordLength =
        UART_WORDLENGTH_8B;

    huart3.Init.StopBits =
        UART_STOPBITS_1;

    huart3.Init.Parity =
        UART_PARITY_NONE;

    huart3.Init.Mode =
        UART_MODE_TX_RX;

    huart3.Init.HwFlowCtl =
        UART_HWCONTROL_NONE;

    huart3.Init.OverSampling =
        UART_OVERSAMPLING_16;

    huart3.Init.OneBitSampling =
        UART_ONE_BIT_SAMPLE_DISABLE;

    huart3.AdvancedInit.AdvFeatureInit =
        UART_ADVFEATURE_NO_INIT;


    if (
        HAL_UART_Init(
            &huart3
        ) != HAL_OK
    )
    {
        Error_Handler();
    }
}


/**
  * @brief USB_OTG_FS Initialization Function
  * @param None
  * @retval None
  */
static void MX_USB_OTG_FS_PCD_Init(void)
{
    hpcd_USB_OTG_FS.Instance =
        USB_OTG_FS;

    hpcd_USB_OTG_FS.Init.dev_endpoints =
        6;

    hpcd_USB_OTG_FS.Init.speed =
        PCD_SPEED_FULL;

    hpcd_USB_OTG_FS.Init.dma_enable =
        DISABLE;

    hpcd_USB_OTG_FS.Init.phy_itface =
        PCD_PHY_EMBEDDED;

    hpcd_USB_OTG_FS.Init.Sof_enable =
        ENABLE;

    hpcd_USB_OTG_FS.Init.low_power_enable =
        DISABLE;

    hpcd_USB_OTG_FS.Init.lpm_enable =
        DISABLE;

    hpcd_USB_OTG_FS.Init.vbus_sensing_enable =
        ENABLE;

    hpcd_USB_OTG_FS.Init.use_dedicated_ep1 =
        DISABLE;


    if (
        HAL_PCD_Init(
            &hpcd_USB_OTG_FS
        ) != HAL_OK
    )
    {
        Error_Handler();
    }
}


/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};


    __HAL_RCC_GPIOC_CLK_ENABLE();

    __HAL_RCC_GPIOF_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();

    __HAL_RCC_GPIOB_CLK_ENABLE();

    __HAL_RCC_GPIOE_CLK_ENABLE();

    __HAL_RCC_GPIOD_CLK_ENABLE();

    __HAL_RCC_GPIOG_CLK_ENABLE();


    HAL_GPIO_WritePin(
        GPIOF,
        M1_step_Pin |
        M2_dir_Pin |
        M3_dir_Pin,
        GPIO_PIN_RESET
    );


    HAL_GPIO_WritePin(
        GPIOE,
        M1_dir_Pin |
        M2_break_Pin |
        M2_step_Pin |
        M3_step_Pin,
        GPIO_PIN_RESET
    );


    GPIO_InitStruct.Pin =
        M1_step_Pin |
        M2_dir_Pin |
        M3_dir_Pin;

    GPIO_InitStruct.Mode =
        GPIO_MODE_OUTPUT_PP;

    GPIO_InitStruct.Pull =
        GPIO_NOPULL;

    GPIO_InitStruct.Speed =
        GPIO_SPEED_FREQ_LOW;


    HAL_GPIO_Init(
        GPIOF,
        &GPIO_InitStruct
    );


    GPIO_InitStruct.Pin =
        M1_dir_Pin |
        M2_break_Pin |
        M2_step_Pin |
        M3_step_Pin;

    GPIO_InitStruct.Mode =
        GPIO_MODE_OUTPUT_PP;

    GPIO_InitStruct.Pull =
        GPIO_NOPULL;

    GPIO_InitStruct.Speed =
        GPIO_SPEED_FREQ_LOW;


    HAL_GPIO_Init(
        GPIOE,
        &GPIO_InitStruct
    );
}


/* USER CODE BEGIN 4 */
/* USER CODE END 4 */


/* MPU Configuration */

void MPU_Config(void)
{
    MPU_Region_InitTypeDef MPU_InitStruct = {0};


    HAL_MPU_Disable();


    MPU_InitStruct.Enable =
        MPU_REGION_ENABLE;

    MPU_InitStruct.Number =
        MPU_REGION_NUMBER0;

    MPU_InitStruct.BaseAddress =
        0x0;

    MPU_InitStruct.Size =
        MPU_REGION_SIZE_4GB;

    MPU_InitStruct.SubRegionDisable =
        0x87;

    MPU_InitStruct.TypeExtField =
        MPU_TEX_LEVEL0;

    MPU_InitStruct.AccessPermission =
        MPU_REGION_NO_ACCESS;

    MPU_InitStruct.DisableExec =
        MPU_INSTRUCTION_ACCESS_DISABLE;

    MPU_InitStruct.IsShareable =
        MPU_ACCESS_SHAREABLE;

    MPU_InitStruct.IsCacheable =
        MPU_ACCESS_NOT_CACHEABLE;

    MPU_InitStruct.IsBufferable =
        MPU_ACCESS_NOT_BUFFERABLE;


    HAL_MPU_ConfigRegion(
        &MPU_InitStruct
    );


    HAL_MPU_Enable(
        MPU_PRIVILEGED_DEFAULT
    );
}


/**
  * @brief Error handler
  */
void Error_Handler(void)
{
    __disable_irq();

    while (1)
    {
    }
}


#ifdef USE_FULL_ASSERT

void assert_failed(
    uint8_t *file,
    uint32_t line
)
{
}

#endif
