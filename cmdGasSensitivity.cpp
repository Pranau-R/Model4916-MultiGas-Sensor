/*

Module:	cmdGasSensitivity.cpp

Function:
        Process the gas sensor sensitivity

Copyright and License:
        This file copyright (C) 2023 by

            MCCI Corporation
            3520 Krums Corners Road
            Ithaca, NY  14850

See accompanying LICENSE file for copyright and license information.

Author:
        Pranau R, MCCI Corporation   December 2023

*/

#include "Model4916_cmd.h"
#include "Model4916_cMeasurementLoop.h"
using namespace McciCatena;
using namespace McciModel4916;

extern cMeasurementLoop gMeasurementLoop;

/*

Name:   ::cmdGasSensitivity()

Function:
        Command dispatcher for "setsensitivity" command.

Definition:
        McciCatena::cCommandStream::CommandFn cmdGasSensitivity;

McciCatena::cCommandStream::CommandStatus cmdGasSensitivity(
        cCommandStream *pThis,
        void *pContext,
        int argc,
        char **argv
        );

Description:
        The "setsensitivity" command has the following syntax:


        setsensitivity {gas_sensor} {sensitivity_value}
        Set the {gas_sensor} sensitivity to {sensitivity_value}

        The "getsensitivity" command has the following syntax:


        setsensitivity {gas_sensor}
        Get the {gas_sensor} sensitivity

Returns:
        cCommandStream::CommandStatus::kSuccess if successful.
        Some other value for failure.

*/

auto const pFram = gCatena.getFram();

// argv[0] is "setsensitivity" or "getsensitivity"
// argv[1] is select gas sensor "CO", "O3", "NO2", "SO2"
// argv[2] is sensitivity value of the spec sensor (not applicable for "getsensitivity" command)
cCommandStream::CommandStatus cmdGasSensitivity(
    cCommandStream *pThis,
    void *pContext,
    int argc,
    char **argv
    )
    {
    if (strcmp(argv[0], "setsensitivity") == 0)
        {
        if (argc > 3)
            {
            pThis->printf("too many args\n");
            pThis->printf("syntax: 'setsensitivity {gas_sensor} {sensitivity_value}'\n");
            return cCommandStream::CommandStatus::kInvalidParameter;
            }

        if (argc < 3)
            {
            pThis->printf("less arguments\n");
            pThis->printf("syntax: 'setsensitivity {gas_sensor} {sensitivity_value}'\n");
            return cCommandStream::CommandStatus::kInvalidParameter;
            }

        if (argc == 3)
            {
            const char* gasSensor[4] = {"CO", "O3", "NO2", "SO2"};
            uint32_t sensitivityRead;
            float sensitivityWrite;

            cCommandStream::getuint32(argc, argv, 2, /*radix*/ 0, sensitivityRead, /* default */ 0);
            sensitivityWrite = gMeasurementLoop.getFloat(sensitivityRead);

            if (strcmp(argv[1], "CO") == 0)
                {
                pFram->saveField(
                    cFramStorage::StandardKeys::kCOSensitivity,
                    sensitivityWrite
                    );

                pThis->printf("The %s gas sensitivity is being set to %d.%02d successfully\n",
                    gasSensor[0],
                    (int)sensitivityWrite,
                    gMeasurementLoop.getDecimal(sensitivityWrite)
                    );

                return cCommandStream::CommandStatus::kSuccess;
                }

            else if (strcmp(argv[1], "O3") == 0)
                {
                pFram->saveField(
                    cFramStorage::StandardKeys::kO3Sensitivity,
                    sensitivityWrite
                    );

                pThis->printf("The %s gas sensitivity is being set to %d.%02d successfully\n",
                    gasSensor[1],
                    (int)sensitivityWrite,
                    gMeasurementLoop.getDecimal(sensitivityWrite)
                    );

                return cCommandStream::CommandStatus::kSuccess;
                }

            else if (strcmp(argv[1], "NO2") == 0)
                {
                pFram->saveField(
                    cFramStorage::StandardKeys::kNO2Sensitivity,
                    sensitivityWrite
                    );

                pThis->printf("The %s gas sensitivity is being set to %d.%02d successfully\n",
                    gasSensor[2],
                    (int)sensitivityWrite,
                    gMeasurementLoop.getDecimal(sensitivityWrite)
                    );

                return cCommandStream::CommandStatus::kSuccess;
                }

            else if (strcmp(argv[1], "SO2") == 0)
                {
                pFram->saveField(
                    cFramStorage::StandardKeys::kSO2Sensitivity,
                    sensitivityWrite
                    );

                pThis->printf("The %s gas sensitivity is being set to %d.%02d successfully\n",
                    gasSensor[3],
                    (int)sensitivityWrite,
                    gMeasurementLoop.getDecimal(sensitivityWrite)
                    );

                return cCommandStream::CommandStatus::kSuccess;
                }
            else
                {
                pThis->printf("The gas sensitivity setting is failed, check arguments!\n"
                    );
                return cCommandStream::CommandStatus::kWriteError;
                }
            }
        }

    else if (strcmp(argv[0], "getsensitivity") == 0)
        {
        if (argc > 2)
            {
            pThis->printf("too many args\n");
            pThis->printf("syntax: 'getsensitivity {gas_sensor}'\n");
            return cCommandStream::CommandStatus::kInvalidParameter;
            }

        if (argc < 2)
            {
            pThis->printf("less arguments\n");
            pThis->printf("syntax: 'getsensitivity {gas_sensor}'\n");
            return cCommandStream::CommandStatus::kInvalidParameter;
            }

        if (argc == 2)
            {
            const char* gasSensor[4] = {"CO", "O3", "NO2", "SO2"};
            float sensitivity;

            if (strcmp(argv[1], "CO") == 0)
                {
                pFram->getField(
                    cFramStorage::StandardKeys::kCOSensitivity,
                    sensitivity
                    );

                pThis->printf("The %s gas sensitivity is %d.%02d\n",
                    gasSensor[0],
                    (int)sensitivity,
                    gMeasurementLoop.getDecimal(sensitivity)
                    );

                return cCommandStream::CommandStatus::kSuccess;
                }

            else if (strcmp(argv[1], "O3") == 0)
                {
                pFram->getField(
                    cFramStorage::StandardKeys::kO3Sensitivity,
                    sensitivity
                    );

                pThis->printf("The %s gas sensitivity is %d.%02d\n",
                    gasSensor[1],
                    (int)sensitivity * (-1),
                    gMeasurementLoop.getDecimal(sensitivity)
                    );

                return cCommandStream::CommandStatus::kSuccess;
                }

            else if (strcmp(argv[1], "NO2") == 0)
                {
                pFram->getField(
                    cFramStorage::StandardKeys::kNO2Sensitivity,
                    sensitivity
                    );

                pThis->printf("The %s gas sensitivity is %d.%02d\n",
                    gasSensor[2],
                    (int)sensitivity * (-1),
                    gMeasurementLoop.getDecimal(sensitivity)
                    );

                return cCommandStream::CommandStatus::kSuccess;
                }

            else if (strcmp(argv[1], "SO2") == 0)
                {
                pFram->getField(
                    cFramStorage::StandardKeys::kSO2Sensitivity,
                    sensitivity
                    );

                pThis->printf("The %s gas sensitivity is %d.%02d\n",
                    gasSensor[3],
                    (int)sensitivity,
                    gMeasurementLoop.getDecimal(sensitivity)
                    );

                return cCommandStream::CommandStatus::kSuccess;
                }
            else
                {
                pThis->printf("The gas sensitivity fetching is failed, check arguments!\n"
                    );

                return cCommandStream::CommandStatus::kWriteError;
                }
            }
        }
    else
        {
        return cCommandStream::CommandStatus::kInvalidParameter;
        }
    }