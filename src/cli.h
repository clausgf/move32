/**
 * Move32 - Naze32 for robotics
 * Command line interpreter
 * Copyright (c) 2018 clausgf. For further info, refer to https://github.com/clausgf/move32
 */


#ifndef MOVE32_CLI_H
#define MOVE32_CLI_H


#include <stdint.h>
#include <stdbool.h>
#include <ctype.h>
#include <string.h>
#include <stdlib.h>


#define CMD_ARGS_MAX 16


class Command {
public:

    /**
     * Construct a command.
     * @param command name of the command
     * @param numberOfParameters number of integer arguments for parsing
     * @param description helpful description
     */
    Command(const char *command, int numberOfParameters, const char *description) :
        mCommand(command), mNumberOfParameters(numberOfParameters), mDescription(description) { }

    const char * getDescription() { return mDescription; }

    /**
     * Try to match a command line plus a number of unsigned integer arguments
     * against this command, ignoring case.
     *
     * @param line command line to parse
     * @return true of the command line matches the command, false otherwise
     */
    bool parse(const char *cmdLine) {
        bool isMatch = true;
        int pos = 0;

        // match the command
        while (mCommand[pos]) {
            isMatch &= (tolower(cmdLine[pos]) == tolower(mCommand[pos]));
            pos++;
        }
        //isMatch &= (cmd[pos] == '\0');

        // match the arguments
        for (int argIndex = 0; argIndex < mNumberOfParameters && argIndex < CMD_ARGS_MAX; argIndex++) {
            // eat spaces
            while (cmdLine[pos] == ' ') {
                pos++;
            }
            // parse number
            isMatch &= (isdigit(cmdLine[pos]));
            int val = atoi(&(cmdLine[pos]));
            isMatch &= (val >= 0 && val <= UINT16_MAX);
            sCmdArgs[argIndex] = val;
            while (isdigit(cmdLine[pos])) {
                pos++;
            }
        }

        // eat spaces
        while (cmdLine[pos] == ' ') {
            pos++;
        }

        // this must be no remainder on the cmd line
        isMatch &= cmdLine[pos] == '\0';

        return isMatch;
    }

    /**
     * Returns an argument from the command line previously parsed,
     * @see parse
     * @param argIndex index of the command line argument
     * @return value of the command line argument
     */
    uint32_t cliGetArg(uint8_t argIndex) {
        uint32_t ret = 0;
        if (argIndex < mNumberOfParameters) {
            ret = sCmdArgs[argIndex];
        }
        return ret;
    }


private:
    const char *mCommand;
    const int mNumberOfParameters;
    const char *mDescription;

    static uint16_t sCmdArgs[CMD_ARGS_MAX];

};


/**
 * Command Line Interpreter
 */
class Cli {
public:

    Cli() { }

private:

};


#endif //MOVE32_CLI_H
