/**
 * @file    web_server.h
 * @brief   Web server
 * @date    19 sty 2015
 * @author  Michal Ksiezopolski
 *
 *
 * @verbatim
 * Copyright (c) 2014 Michal Ksiezopolski.
 * All rights reserved. This program and the
 * accompanying materials are made available
 * under the terms of the GNU Public License
 * v3.0 which accompanies this distribution,
 * and is available at
 * http://www.gnu.org/licenses/gpl.html
 * @endverbatim
 */
#ifndef WEB_SERVER_H_
#define WEB_SERVER_H_

#include <inttypes.h>

uint8_t HTTP_Event(void);
void HTTP_Init(void);

#endif
