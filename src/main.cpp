/**
 * Move32 - Naze32 for robotics
 * Main program
 * Copyright (c) 2017 clausgf. For further info, refer to https://github.com/clausgf/move32
 */

#include <stdint.h>
#include <stdbool.h>
#include <ctype.h>
#include <stdlib.h>

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

void rxUpdateCallback(uint8_t rxChannel, uint16_t pulseLenUs) {
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


#define CMD_ARGS_MAX 16
uint16_t cmdArgs[CMD_ARGS_MAX];


static bool cliIsCmdChar(char ch) {
    return isalnum(ch) || (ch == '_');
}

/**
 * Tries to match a command line against a given command plus
 * a number of unsigned integer arguments.
 *
 * @param line command line to parse
 * @param cmd command to parse
 * @param numArgs number of integer arguments to parse
 * @return true of the command line matches the command, false otherwise
 */
static bool cliParse(char *cmdLine, const char *cmd, int numArgs) {
    bool isMatch = true;
    int pos = 0;

    // match the command
    while (pos < SERIAL_RX_MAX && cliIsCmdChar(cmdLine[pos]) && cliIsCmdChar(cmd[pos]) ) {
        isMatch &= (tolower(cmdLine[pos]) == tolower(cmd[pos]));
        pos++;
    }
    isMatch &= (cmd[pos] == '\0');

    // match the arguments
    for (int argIndex = 0; argIndex < numArgs && argIndex < CMD_ARGS_MAX; argIndex++) {
        // eat spaces
        while (pos < SERIAL_RX_MAX && cmdLine[pos] == ' ') {
            pos++;
        }
        // parse number
        isMatch &= (pos < SERIAL_RX_MAX && isdigit(cmdLine[pos]));
        int val = atoi(&(cmdLine[pos]));
        isMatch &= (val >= 0 && val <= UINT16_MAX);
        cmdArgs[argIndex] = val;
        while (pos < SERIAL_RX_MAX && isdigit(cmdLine[pos])) {
            pos++;
        }
    }

    // eat spaces
    while (pos < SERIAL_RX_MAX && cmdLine[pos] == ' ') {
        pos++;
    }

    // this must be no remainder on the cmd line
    isMatch &= cmdLine[pos] == '\0';

    return isMatch;
}


/**
 * Returns an argument from the command line previously parsed,
 * @see cliParse
 * @param argIndex index of the command line argument
 * @return value of the command line argument
 */
static uint32_t cliGetArg(uint8_t argIndex) {
    uint32_t ret = 0;
    if (argIndex < CMD_ARGS_MAX) {
        ret = cmdArgs[argIndex];
    }
    return ret;
}


void cliProcess(char *cmdLine) {
    if (cmdLine[0] == '#' || cmdLine[0] == '\0') {

        // ignore comment or empty command

    } else if (cliParse(cmdLine, "servos", SERVOS_MAX)) {

        bool cmdOk = true;
        for (uint8_t i=0; i<SERVOS_MAX; i++) {
            cmdOk &= (cliGetArg(i) <= 3000 );
        }
        if (cmdOk) {
            for (uint8_t i=0; i<SERVOS_MAX; i++) {
                setServoFromHost(i, cliGetArg(i));
            }
            printf("ok servos");
            for (uint8_t i=0; i<SERVOS_MAX; i++) {
                printf(" %d", (int) cliGetArg(i));
            }
            printf("\n");
        } else {
            printf("error %s\n", cmdLine);
        }

    } else if (cliParse(cmdLine, "servo", 2)) {

        uint8_t servoIndex = cliGetArg(0);
        uint32_t pulseLenUs = cliGetArg(1);
        if (servoIndex < SERVOS_MAX && pulseLenUs <= 3000) {
            setServoFromHost(servoIndex, pulseLenUs);
            printf("ok servo %d %d\n", (int)servoIndex, (int)pulseLenUs);
        } else {
            printf("error %s\n", cmdLine);
        }

    } else if (cliParse(cmdLine, "rx", 0)) {

        transmitRx();

    } else if (cliParse(cmdLine, "failsafe", 2)) {

        uint8_t servoIndex = cliGetArg(0);
        uint16_t pulseLenUs = cliGetArg(1);
        if (servoIndex < SERVOS_MAX && pulseLenUs <= 3000) {
            servos[servoIndex].setFailsafe(pulseLenUs);
            printf("ok failsafe %d %d\n", (int)servoIndex, (int)pulseLenUs);
        } else {
            printf("error %s\n", cmdLine);
        }

    } else if (cliParse(cmdLine, "fallback_timeout", 1)) {

        fbTimeoutMs = cliGetArg(0);
        printf("ok fallback_timeout %d\n", (int)fbTimeoutMs);

    } else if (cliParse(cmdLine, "fallback_force", 3)) {

        uint8_t rxIndex = cliGetArg(0);
        uint8_t predicate = cliGetArg(1);
        uint16_t pulseLenUs = cliGetArg(2);
        if (rxIndex < RX_MAX_CHANNEL && predicate < 2 && pulseLenUs <= 3000) {
            fbForceChannel = rxIndex;
            fbForcePredicate = predicate;
            fbForcePulseLenUs = pulseLenUs;
            printf("ok fallback_force %d %d %d\n",
                   (int)fbForceChannel,
                   (int)fbForcePredicate,
                   (int)fbForcePulseLenUs);
        } else {
            printf("error %s\n", cmdLine);
        }

    } else if (cliParse(cmdLine, "servo_from_rx", 2)) {

        uint8_t servoIndex = cliGetArg(0);
        uint8_t rxIndex = cliGetArg(1);
        if (rxIndex < RX_MAX_CHANNEL && servoIndex < SERVOS_MAX) {
            rxForServo[servoIndex] = rxIndex;
            printf("ok servo_from_rx %d %d\n", (int)servoIndex, (int)rxIndex);
        } else {
            printf("error %s\n", cmdLine);
        }

    } else if (cliParse(cmdLine, "autorx", 1)) {

        autoTransmitRxIntervalMs = cliGetArg(0);
        lastAutoTransmitStart = HAL_GetTick() - autoTransmitRxIntervalMs;
        printf("ok autorx %d\n", (int)autoTransmitRxIntervalMs);

    } else if (cliParse(cmdLine, "dump", 0)) {

        printf("ok dump\n");
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

    } else if (cliParse(cmdLine, "help", 0)) {

        printf("ok help\n");
        // TODO

    } else if (cliParse(cmdLine, "reset", 0)) {

        printf("ok reset\n");
        NVIC_SystemReset();

    } else {

        printf("error %s\n", cmdLine);

    }
}


Command cmdServos("servos", SERVOS_MAX, "servos pulseUs0 ... pulseUsN - Set pulse length of all servos");

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
    //servosInit();
    //receiverInit();
    while (1) {
        printf("abc\n");
        HAL_Delay(500);
    }
    printf("\nSystemCoreClock = %lu\n", SystemCoreClock);

    // rc input servo channels are now processed in the background
    // by peripherals, in ISRs and in rxUpdateCallback

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
            cliProcess(received);
        }

        // generate output
        bool isAutoTransmitTimerExpired = (HAL_GetTick()-lastAutoTransmitStart) >= autoTransmitRxIntervalMs;
        if ( (autoTransmitRxIntervalMs == 1 && wasLastRxChannelUpdated) ) {
            transmitRx();
            lastAutoTransmitStart = HAL_GetTick();
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
