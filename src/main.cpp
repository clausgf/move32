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
// Servo handling state machine
// **************************************************************************

class ServosHandler {
public:

    /// timeout for loosing host signal (0: no timeout)
    uint32_t mFallbackTimeoutMs;

    /// last reception of a signal from the host for timeout supervision
    uint32_t mFallbackTimeoutStart[SERVOS_MAX];

    /// rx channel index for forcing fallback via remote control
    uint8_t mFallbackForceChannel;

    /// comparison operator in fbForceChannel _op_ fbForcePulseLenUs: 0: "<"; 1: ">"
    uint8_t mFallbackForcePredicate = 0;

    /// pulse length for comparison of fbForceChannel; 0 means no fallback by rxChannel
    uint16_t mFallbackForcePulseLenUs = 0;

    /// rx channel for a servo for remote control (i.e. in fallback mode); 255: never fall back to rx
    uint8_t mRxForServo[SERVOS_MAX];

    enum SERVO_MODE { SERVO_MODE_HOST, SERVO_MODE_FALLBACK, SERVO_MODE_FAILSAFE };
    enum SERVO_MODE mServoMode[SERVOS_MAX];


    ServosHandler():
            mFallbackTimeoutMs(2000),
            mFallbackForceChannel(0),
            mFallbackForcePredicate(0),
            mFallbackForcePulseLenUs(0)
    {
        for (uint8_t i=0; i<SERVOS_MAX; i++) {
            mFallbackTimeoutStart[i] = board.getTicks() - mFallbackTimeoutMs;
            mRxForServo[i] = i;
            mServoMode[i] = SERVO_MODE_FAILSAFE;
        }
    }


    void updateForChannel(uint8_t servoChannel) {
        if (servoChannel < SERVOS_MAX) {
            // fallback to rx by channel
            bool isFbForce_ChannelIndexValid = (mFallbackForceChannel < RX_MAX_CHANNEL);
            uint16_t fbChannelValue = receiver.getChannel(mFallbackForceChannel);
            bool isFbForce_ChannelValid   = isFbForce_ChannelIndexValid && (fbChannelValue > 0);
            bool isFbForce_FunctionActive = isFbForce_ChannelIndexValid && (mFallbackForcePulseLenUs != 0);

            bool isFbForce_Predicate =
                    ( mFallbackForcePredicate == 0 && fbChannelValue < mFallbackForcePulseLenUs ) ||
                    ( mFallbackForcePredicate == 1 && fbChannelValue > mFallbackForcePulseLenUs );

            bool isFbForce    =  isFbForce_FunctionActive &&   isFbForce_ChannelValid &&  isFbForce_Predicate;
            bool isFbNotForce = !isFbForce_FunctionActive || ( isFbForce_ChannelValid && !isFbForce_Predicate );

            // fallback to rx by timeout
            bool isFbTimeout_FunctionActive = (mFallbackTimeoutMs > 0);
            bool isHostFreshnessExpired = ( (board.getTicks() - mFallbackTimeoutStart[servoChannel]) > mFallbackTimeoutMs );
            bool isFbTimeout = isFbTimeout_FunctionActive && isHostFreshnessExpired;

            // is the rx channel for fallback valid?
            uint8_t fbRxChannel = mRxForServo[servoChannel];
            bool isFb_ChannelIndexValid = (fbRxChannel < RX_MAX_CHANNEL);
            bool isFb_ChannelValid = isFb_ChannelIndexValid && (receiver.getChannel(fbRxChannel) > 0);

            bool isHost = !isHostFreshnessExpired && isFbNotForce;
            bool isFb   = (isFbForce || isFbTimeout) && isFb_ChannelValid;

            if (isHost) {

                // host mode
                mServoMode[servoChannel] = SERVO_MODE_HOST;

            } else if (isFb) {

                // fallback mode
                mServoMode[servoChannel] = SERVO_MODE_FALLBACK;
                // switching back to host requires fresh data
                mFallbackTimeoutStart[servoChannel] = board.getTicks() - mFallbackTimeoutMs;
                // set servo position immediately
                uint16_t rxPulseLenMs = receiver.getChannel(fbRxChannel);
                servos[servoChannel].set(rxPulseLenMs);

            } else {

                // failsafe mode
                mServoMode[servoChannel] = SERVO_MODE_FAILSAFE;
                // switching back to host requires fresh data
                mFallbackTimeoutStart[servoChannel] = board.getTicks() - mFallbackTimeoutMs;
                // set servo position immediately
                servos[servoChannel].set(0);

            }
        }
    }


    void setServoFromHost(uint8_t index, uint16_t pulseLenUs) {
        if (index < SERVOS_MAX) {
            mFallbackTimeoutStart[index] = board.getTicks();
            updateForChannel(index);
            if (mServoMode[index] == SERVO_MODE_HOST) {
                servos[index].set(pulseLenUs);
            }
        }
    }


    void setServoFromRx(uint8_t rxChannel, uint16_t pulseLenUs) {
        if (rxChannel < RX_MAX_CHANNEL) {
            for (int servoChannel=0; servoChannel<SERVOS_MAX; servoChannel++) {
                if (mRxForServo[servoChannel] == rxChannel) {
                    // this is the servo to update
                    if (mServoMode[servoChannel] == SERVO_MODE_FALLBACK) {
                        servos[rxChannel].set(pulseLenUs);
                    }
                }
            }
        }
    }


    void updateAll() {
        for (uint8_t i=0; i<SERVOS_MAX; i++) {
            updateForChannel(i);
        }
    }


    bool isFallback() const {
        bool fallback = false;
        for (uint8_t i=0; i<SERVOS_MAX; i++) {
            fallback |= (mServoMode[i] == SERVO_MODE_FALLBACK);
        }
        return fallback;
    }


    bool isFailsafe() const {
        bool failsafe = false;
        for (uint8_t i=0; i<SERVOS_MAX; i++) {
            failsafe |= (mServoMode[i] == SERVO_MODE_FAILSAFE);
        }
        return failsafe;
    }

};

ServosHandler servosHandler;


// **************************************************************************


class RxReporter {
public:

    RxReporter() :
        mAutoTransmitIntervalMs(0),
        mLastAutoTransmitStart(board.getTicks() - mAutoTransmitIntervalMs),
        mIsAllChannelsReady(false)
    { }


    void report() const {
        printf("ok rx %ld", board.getTicks());
        for (uint8_t i=0; i<RX_MAX_CHANNEL; i++) {
            printf(" %d", receiver.getChannel(i));
        }
        printf("\n");
    }


    void setAllChannelsReady() {
        mIsAllChannelsReady = true;
    }


    uint16_t getAutoTransmitIntervalMs() { return mAutoTransmitIntervalMs; }

    /// 0: off; 1: as frequently as possible; >1 given interval
    void setAutoTransmitIntervalMs(uint16_t autoTransmitIntervalMs) {
        mAutoTransmitIntervalMs = autoTransmitIntervalMs;
        mLastAutoTransmitStart = board.getTicks() - mAutoTransmitIntervalMs;
    }


    void handle() {
        bool isAutoTransmitTimerExpired = (board.getTicks() - mLastAutoTransmitStart) >= mAutoTransmitIntervalMs;
        if ( (mAutoTransmitIntervalMs == 1 && mIsAllChannelsReady) ) {
            report();
            mLastAutoTransmitStart = board.getTicks();
            mIsAllChannelsReady = false;
        }
        if (mAutoTransmitIntervalMs > 1 && isAutoTransmitTimerExpired ) {
            report();
            mLastAutoTransmitStart += mAutoTransmitIntervalMs;
            mIsAllChannelsReady = false;
        }

    }

private:
    uint16_t mAutoTransmitIntervalMs;
    uint32_t mLastAutoTransmitStart;
    bool mIsAllChannelsReady;
};

RxReporter rxReporter;


// **************************************************************************


void rxUpdateCallbackFromIsr(uint8_t rxChannel, uint16_t pulseLenUs) {
    servosHandler.setServoFromRx(rxChannel, pulseLenUs);
    if (rxChannel == SERVOS_MAX-1) {
        rxReporter.setAllChannelsReady();
    }
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
                        servosHandler.setServoFromHost(i, cmd.getParameter(i));
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
                    servosHandler.setServoFromHost(servoIndex, pulseLenUs);
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
            "set timeout for entering FALLBACK_MODE instead of HOST_MODE when the a servo has not received fresh data from the host (parameter: timeout in ms; 0: deactivate)",
            [](const Command & cmd) {
                servosHandler.mFallbackTimeoutMs = cmd.getParameter(0);
                return true;
            }),

    Command("fallback_force", 3,
            "receiver channel to force fallback operation (three parameters: rxIndex (255: deactivate), predicate (0: <, 1: >), pulseLenUs",
            [](const Command & cmd) {
                uint8_t rxIndex = cmd.getParameter(0);
                uint8_t predicate = cmd.getParameter(1);
                uint16_t pulseLenUs = cmd.getParameter(2);
                bool cmdOk = (rxIndex < RX_MAX_CHANNEL && predicate < 2 && pulseLenUs <= 3000);
                if (cmdOk) {
                    servosHandler.mFallbackForceChannel = rxIndex;
                    servosHandler.mFallbackForcePredicate = predicate;
                    servosHandler.mFallbackForcePulseLenUs = pulseLenUs;
                }
                return cmdOk;
            }),

    Command("fallback_servo_rx", 1,
            "connect servo to receiver channel in FALLBACK_MODE (two parameters: servoIndex, rxIndex)",
            [](const Command & cmd) {
                uint8_t servoIndex = cmd.getParameter(0);
                uint8_t rxIndex = cmd.getParameter(1);
                bool cmdOk = (rxIndex < RX_MAX_CHANNEL && servoIndex < SERVOS_MAX);
                if (cmdOk) {
                    servosHandler.mRxForServo[servoIndex] = rxIndex;
                }
                return cmdOk;
            }),

    Command("rx", 0,
            "report rc receiver channels",
            [](const Command & cmd) {
                rxReporter.report();
                return true;
            }),

    Command("rx_auto", 1,
            "automatic rc receiver reporting (parameter: 0: off; 1: onChange; >1: reporting interval in ms)",
            [](const Command & cmd) {
                rxReporter.setAutoTransmitIntervalMs(cmd.getParameter(0));
                return true;
            }),

    Command("rx_type", 1,
            "rc receiver type (parameter: 0: FULL; 1: CPPM)",
            [](const Command & cmd) {
                uint16_t receiverType = cmd.getParameter(0);
                bool cmdOk = (receiverType >= 0 && receiverType <= 1);
                if (cmdOk) {
                    receiver.setReceiverType((Receiver::RECEIVER_TYPE_E) receiverType);
                }
                return cmdOk;
            }),

    Command("dump", 0,
            "dump configuration",
            [](const Command & cmd) {
                for (uint8_t servoIndex=0; servoIndex<SERVOS_MAX; servoIndex++) {
                    printf("failsafe %d %d\n", (int) servoIndex, (int) servos[servoIndex].getFailsafe());
                }
                printf("fallback_timeout %d\n", (int) servosHandler.mFallbackTimeoutMs);
                printf("fallback_force %d %d %d\n",
                       (int) servosHandler.mFallbackForceChannel,
                       (int) servosHandler.mFallbackForcePredicate,
                       (int) servosHandler.mFallbackForcePulseLenUs);
                for (uint8_t servoIndex=0; servoIndex<SERVOS_MAX; servoIndex++) {
                    printf("fallback_servo_rx %d %d\n",
                            (int) servoIndex, (int) servosHandler.mRxForServo[servoIndex]);
                }
                printf("rx_auto %d\n", (int) rxReporter.getAutoTransmitIntervalMs());
                printf("rx_type %d\n", (int) receiver.getReceiverType());
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
    printf("All signals from the receiver time out after %lu ms and fail to their preprogrammed failsafe pulses (failsafe_pulse).\n",
            receiver.getTimeout());
    printf("Move32 is configured for SERVOS_MAX=%d servo outputs.\n",
           SERVOS_MAX);
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

    // main loop
    while (1) {
        watchdog.reset();

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
        rxReporter.handle();

        // update servo positions/states
        servosHandler.updateAll();

        // led0 (red) output
        if (servosHandler.isFailsafe()) {
            led0.set(true);
        } else if (servosHandler.isFallback()) {
            led0.set( board.getTicks() % 500 < 250 );
        } else { // host mode
            led0.set(false);
        }

        // led1 (green) output
        led1.set(HAL_GetTick() % 500 < 250);
    }

}

// **************************************************************************
