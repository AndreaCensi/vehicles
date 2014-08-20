/*
 * $Id: debug.h,v 1.5 2006/01/30 23:07:57 mclark Exp $
 *
 * Copyright (c) 2004, 2005 Metaparadigm Pte. Ltd.
 * Michael Clark <michael@metaparadigm.com>
 *
 * This library is free software; you can redistribute it and/or modify
 * it under the terms of the MIT license. See COPYING for details.
 *
 */

#ifndef _DEBUG_H_
#define _DEBUG_H_

void mc_set_debug(int debug);
int mc_get_debug(void);

void mc_set_syslog(int syslog);
void mc_abort(const char *msg, ...);
void mc_debug(const char *msg, ...);
void mc_error(const char *msg, ...);
void mc_info(const char *msg, ...);

#endif
