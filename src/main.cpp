/**
 * Move32 - Naze32 for robotics
 * Main program
 * Copyright (c) 2017 clausgf. For further info, refer to https://github.com/clausgf/move32
 */

#include <stdint.h>
#include <stdbool.h>
#include <ctype.h>
#include <stdlib.h>

#include <algorithm>

#include "board.h"
#include "servos.h"
#include "receiver.h"
#include "cli.h"
#include "main.h"


// **************************************************************************

enum SERVO_MODE_E { SERVO_MODE_HOST, SERVO_MODE_RX, SERVO_MODE_FAILSAFE };
enum SERVO_MODE_E servoMode[SERVOS_MAX];

// fallback (fb) parameters
uint32_t fbTimeoutMs = 2000; // 0: no timeout fallback
uint32_t fbTimeoutStart[SERVOS_MAX];

/// rx channel index for switching to fallback mode
uint8_t fbForceChannel = 0;

/// comparison operator in fbForceChannel _op_ fbForcePulseLenUs: 0: "<"; 1: ">"
uint8_t fbForcePredicate = 0;

/// pulse length for comparison of fbForceChannel; 0 means no fallback by rxChannel
uint16_t fbForcePulseLenUs = 0;

/// rx channel to fall back to when host expired; 255 means do not fallback
uint8_t rxForServo[SERVOS_MAX];

/// 0: off; 1: as frequently as possible; >1 given interval
uint16_t autoTransmitRxIntervalMs = 0;
uint32_t lastAutoTransmitStart;
bool wasLastRxChannelUpdated;


// **************************************************************************
// Servo handling state machine
// **************************************************************************

void updateServoMode(uint8_t index) {
    if (index < SERVOS_MAX) {
        // fallback to rx by channel
        bool isFbForce_ChannelIndexValid = (fbForceChannel < RX_MAX_CHANNEL);
        bool isFbForce_ChannelValid = isFbForce_ChannelIndexValid && (receiver.getChannel(fbForceChannel) > 0);

        bool isFbForce_FunctionActive = (fbForceChannel < RX_MAX_CHANNEL) && (fbForcePulseLenUs != 0);

        bool isFbForce_Predicate =
                ( fbForcePredicate == 0 && receiver.getChannel(fbForceChannel) < fbForcePulseLenUs ) ||
                ( fbForcePredicate == 1 && receiver.getChannel(fbForceChannel) > fbForcePulseLenUs );

        bool isFbForce = isFbForce_FunctionActive && isFbForce_ChannelValid && isFbForce_Predicate;

        bool isFbNotForce = !isFbForce_FunctionActive || ( isFbForce_ChannelValid && !isFbForce_Predicate );

        // fallback to rx by timeout
        bool isFbTimeout_FunctionActive = (fbTimeoutMs > 0);
        bool isHostFreshnessExpired = ( (HAL_GetTick()-fbTimeoutStart[index]) > fbTimeoutMs );
        bool isFbTimeout = isFbTimeout_FunctionActive && isHostFreshnessExpired;

        // is the rx channel for fallback valid?
        uint8_t fbRxChannel = rxForServo[index];
        bool isFb_ChannelIndexValid = (fbRxChannel < RX_MAX_CHANNEL);
        bool isFb_ChannelValid = isFb_ChannelIndexValid && (receiver.getChannel(fbRxChannel) > 0);

        bool isHost = !isHostFreshnessExpired && isFbNotForce;
        bool isFb = (isFbForce || isFbTimeout) && isFb_ChannelValid;

        if (isHost) {
            servoMode[index] = SERVO_MODE_HOST;
        } else if (isFb) {
            servoMode[index] = SERVO_MODE_RX;
            // switching back to host requires fresh data
            fbTimeoutStart[index] = HAL_GetTick() - fbTimeoutMs;
            // set servo position immediately
            uint16_t rxPulseLenMs = receiver.getChannel(fbRxChannel);
            servos[index].set(rxPulseLenMs);
        } else {
            servoMode[index] = SERVO_MODE_FAILSAFE;
            // switching back to host requires fresh data
            fbTimeoutStart[index] = HAL_GetTick() - fbTimeoutMs;
            // set servo position immediately
            servos[index].set(0);
        }
    }
}

void setServoFromHost(uint8_t index, uint16_t pulseLenUs) {
    if (index < SERVOS_MAX) {
        fbTimeoutStart[index] = HAL_GetTick();
        updateServoMode(index);
        if (servoMode[index] == SERVO_MODE_HOST) {
            servos[index].set(pulseLenUs);
        }
    }
}

void rxUpdateCallbackFromIsr(uint8_t rxChannel, uint16_t pulseLenUs) {
    if (rxChannel < RX_MAX_CHANNEL) {
        for (int servo=0; servo<SERVOS_MAX; servo++) {
            if ( (servoMode[servo] == SERVO_MODE_RX) && (rxForServo[servo] == rxChannel)) {
                servos[rxChannel].set(pulseLenUs);
            }
        }
    }
    if (rxChannel == SERVOS_MAX-1) {
        wasLastRxChannelUpdated = true;
    }
}

// **************************************************************************

void transmitRx() {
    printf("ok rx %ld", HAL_GetTick());
    for (uint8_t i=0; i<RX_MAX_CHANNEL; i++) {
        printf(" %d", receiver.getChannel(i));
    }
    printf("\n");
}


// **************************************************************************
// Command handling
// **************************************************************************

bool show_help(const Command & cmd);

Command commands[] {
    Command("servos", SERVOS_MAX,
            "Set pulse length for all servos in HOST_MODE (SERVOS_MAX parameters: pulseLenUs)",
            [](const Command & cmd) {
                bool cmdOk = true;
                for (uint8_t i=0; i<SERVOS_MAX; i++) {
                    cmdOk &= (cmd.getParameter(i) <= 3000 );
                }
                if (cmdOk) {
                    for (uint8_t i=0; i<SERVOS_MAX; i++) {
                        setServoFromHost(i, cmd.getParameter(i));
                    }
                }
                return cmdOk;
            }),

    Command("servo_pulse", 2,
            "set servo pulse length in HOST_MODE (two parameters: servoIndex < SERVOS_MAX, pulseLenUs)",
            [](const Command & cmd) {
                uint8_t servoIndex = cmd.getParameter(0);
                uint32_t pulseLenUs = cmd.getParameter(1);
                bool cmdOk = (servoIndex < SERVOS_MAX && pulseLenUs <= 3000);
                if (cmdOk) {
                    setServoFromHost(servoIndex, pulseLenUs);
                }
                return cmdOk;
            }),

    Command("failsafe_pulse", 2,
            "set constant servo pulse length in FAILSAFE_MODE (two parameters: servoIndex < SERVOS_MAX, pulseLenUs)",
            [](const Command & cmd) {
                uint8_t servoIndex = cmd.getParameter(0);
                uint32_t pulseLenUs = cmd.getParameter(1);
                bool cmdOk = (servoIndex < SERVOS_MAX && pulseLenUs <= 3000);
                if (cmdOk) {
                    servos[servoIndex].setFailsafe(pulseLenUs);
                }
                return cmdOk;
            }),

    Command("fallback_timeout", 1,
            "set timeout for entering fallback mode from host mode when the a servo has not received fresh data from the host (parameter: timeout in ms)",
            [](const Command & cmd) {
                fbTimeoutMs = cmd.getParameter(0);
                return true;
            }),

    Command("fallback_force", 3,
            "receiver channel to force fallback operation (three parameters: rxIndex, predicate (0: <, 1: >), pulseLenUs",
            [](const Command & cmd) {
                uint8_t rxIndex = cmd.getParameter(0);
                uint8_t predicate = cmd.getParameter(1);
                uint16_t pulseLenUs = cmd.getParameter(2);
                bool cmdOk = (rxIndex < RX_MAX_CHANNEL && predicate < 2 && pulseLenUs <= 3000);
                if (cmdOk) {
                    fbForceChannel = rxIndex;
                    fbForcePredicate = predicate;
                    fbForcePulseLenUs = pulseLenUs;
                }
                return cmdOk;
            }),

    Command("servo_from_rx", 1,
            "connect servo to receiver channel (two parameters: servoIndex, rxIndex)",
            [](const Command & cmd) {
                uint8_t servoIndex = cmd.getParameter(0);
                uint8_t rxIndex = cmd.getParameter(1);
                bool cmdOk = (rxIndex < RX_MAX_CHANNEL && servoIndex < SERVOS_MAX);
                if (cmdOk) {
                    rxForServo[servoIndex] = rxIndex;
                }
                return cmdOk;
            }),

    Command("rx", 0,
            "report rc receiver channels",
            [](const Command & cmd) {
                transmitRx();
                return true;
            }),

    Command("auto_rx", 1,
            "automatic rc receiver reporting (parameter: 0: off; 1: onChange; >1: reporting interval in ms)",
            [](const Command & cmd) {
                autoTransmitRxIntervalMs = cmd.getParameter(0);
                lastAutoTransmitStart = board.getTicks() - autoTransmitRxIntervalMs;
                return true;
            }),

    Command("dump", 0,
            "dump configuration",
            [](const Command & cmd) {
                for (uint8_t servoIndex=0; servoIndex<SERVOS_MAX; servoIndex++) {
                    printf("failsafe %d %d\n", (int)servoIndex, (int)servos[servoIndex].getFailsafe());
                }
                printf("fallback_timeout %d\n", (int)fbTimeoutMs);
                printf("fallback_force %d %d %d\n",
                       (int)fbForceChannel,
                       (int)fbForcePredicate,
                       (int)fbForcePulseLenUs);
                for (uint8_t servoIndex=0; servoIndex<SERVOS_MAX; servoIndex++) {
                    printf("servo_from_rx %d %d\n", (int) servoIndex, (int) rxForServo[servoIndex]);
                }
                printf("autorx %d\n", (int)autoTransmitRxIntervalMs);
                return true;
            }),

    Command("reset", 0,
            "system reset",
            [](const Command & cmd) {
                printf("ok reset\n");
                NVIC_SystemReset();
                return true;
            }),

    Command("help", 0,
            "show help text",
            show_help),
};

bool show_help(const Command & cmd) {
    printf("\n\n*** Move32 ***\n");
    printf("\n*Modes (per servo)*\n");
    printf("HOST_MODE: servo pulse is generated from host commands\n"
           "      when host data is not timed out (fallback_timeout) and rc receiver does not force fallback (force_fallback)\n");
    printf("FALLBACK_MODE: servo pulse is generated from rc receiver channel\n"
           "          when host data is timed out (fallback_timeout) or rc receiver does force fallback (force_fallback)\n");
    printf("FAILSAFE_MODE: preprogrammed constant servo pulse\n"
           "          when no other mode is active\n");
    printf("All signal from the receiver time out after %lu ms and fail to their preprogrammed failsafe pulses (failsafe_pulse)\n",
            receiver.getTimeout());
    printf("\n*Commands*\n");
    std::for_each(std::begin(commands), std::end(commands), [&](Command &cmd) {
        printf("%s - %s\n", cmd.getCommand(), cmd.getDescription());
    });
    return true;
}

// **************************************************************************
// Main
// **************************************************************************


int main(void) {
    // Reset of all peripherals, Initializes the Flash interface and Systick.
    // This function calls HAL_MspInit() generated into *_hal_msp.c for
    // board-specific initialization.
    HAL_Init();

    // configure the board including clock, watchdog, leds and serial
    boardInit();
    servosInit();
    receiverInit();
    printf("\nSystemCoreClock = %lu\n", SystemCoreClock);

    // rc input servo channels are now processed in the background
    // by peripherals, in ISRs and in rxUpdateCallback

    printf("\n\nok Move32\n");

    // **********************************************************************

    for (uint8_t i=0; i<SERVOS_MAX; i++) {
        rxForServo[i] = i;
    }

    wasLastRxChannelUpdated = false;
    // main loop
    while (1) {
        // TODO watchdogReset();

        // process commands
        char *received = serial.serialReceive();
        if (received != NULL) {
            bool isAlreadyParsed = false;
            std::for_each(std::begin(commands), std::end(commands), [&](Command &cmd) {
                if (cmd.parse(received)) {
                    if (cmd.execute()) {
                        printf("ok %s\n", cmd.getCommand());
                    } else {
                        printf("error %s\n", cmd.getCommand());
                    }
                    isAlreadyParsed = true;
                };
            });
            if (!isAlreadyParsed) {
                printf("error parsing \"%s\" - enter help for more info\n", received);
            }
        }

        // generate output
        bool isAutoTransmitTimerExpired = (board.getTicks()-lastAutoTransmitStart) >= autoTransmitRxIntervalMs;
        if ( (autoTransmitRxIntervalMs == 1 && wasLastRxChannelUpdated) ) {
            transmitRx();
            lastAutoTransmitStart = board.getTicks();
            wasLastRxChannelUpdated = false;
        }
        if (autoTransmitRxIntervalMs > 1 && isAutoTransmitTimerExpired ) {
            transmitRx();
            lastAutoTransmitStart += autoTransmitRxIntervalMs;
            wasLastRxChannelUpdated = false;
        }

        // determine states
        bool fallback = false;
        bool failsafe = false;
        for (uint8_t index=0; index<SERVOS_MAX; index++) {
            updateServoMode(index);
            fallback |= (servoMode[index] == SERVO_MODE_RX);
            failsafe |= (servoMode[index] == SERVO_MODE_FAILSAFE);
        }

        // led0 (red) output
        if (failsafe) {
            led0.set(true);
        } else if (fallback) {
            led0.set( HAL_GetTick() % 500 < 250 );
        } else { // host mode
            led0.set(false);
        }

        // led1 (green) output
        led1.set(HAL_GetTick() % 500 < 250);
    }

}

// **************************************************************************
