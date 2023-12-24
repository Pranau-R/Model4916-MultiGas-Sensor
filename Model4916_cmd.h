/*

Module: Model4916_cmd.h

Function:
    Linkage for Model4916 command processing.

Copyright:
    See accompanying LICENSE file for copyright and license information.

Author:
    Dhinesh Kumar Pitchai, MCCI Corporation   November 2022
*/

#ifndef _Model4916_cmd_h_
#define _Model4916_cmd_h_

#pragma once

#include <Catena_CommandStream.h>

McciCatena::cCommandStream::CommandFn cmdLog;
McciCatena::cCommandStream::CommandFn cmdDir;
McciCatena::cCommandStream::CommandFn cmdGasSensitivity;

#endif /* _Model4916_cmd_h_ */
