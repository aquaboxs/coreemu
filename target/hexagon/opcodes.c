/*
 *  Copyright (c) 2019 Qualcomm Innovation Center, Inc. All Rights Reserved.
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, see <http://www.gnu.org/licenses/>.
 */

/*
 * opcodes.c
 *
 * data tables generated automatically
 * Maybe some functions too
 */

#include <stdio.h>
#include <stdlib.h>
#include <strings.h>
#include <string.h>
#include <stdarg.h>
#include <ctype.h>
#include "qemu/osdep.h"
#include "opcodes.h"
#include "imported/decode.h"

#define VEC_DESCR(A, B, C) DESCR(A, B, C)
#define DONAME(X) #X

const char *opcode_names[] = {
#define OPCODE(IID) DONAME(IID)
#include "opcodes_def_generated.h"
    NULL
#undef OPCODE
};

const char *opcode_reginfo[] = {
#define IMMINFO(TAG, SIGN, SIZE, SHAMT, SIGN2, SIZE2, SHAMT2)    /* nothing */
#define REGINFO(TAG, REGINFO, RREGS, WREGS) REGINFO,
#include "op_regs_generated.h"
    NULL
#undef REGINFO
#undef IMMINFO
};


const char *opcode_rregs[] = {
#define IMMINFO(TAG, SIGN, SIZE, SHAMT, SIGN2, SIZE2, SHAMT2)    /* nothing */
#define REGINFO(TAG, REGINFO, RREGS, WREGS) RREGS,
#include "op_regs_generated.h"
    NULL
#undef REGINFO
#undef IMMINFO
};


const char *opcode_wregs[] = {
#define IMMINFO(TAG, SIGN, SIZE, SHAMT, SIGN2, SIZE2, SHAMT2)    /* nothing */
#define REGINFO(TAG, REGINFO, RREGS, WREGS) WREGS,
#include "op_regs_generated.h"
    NULL
#undef REGINFO
#undef IMMINFO
};

const char *opcode_short_semantics[] = {
#define OPCODE(X)              NULL
#include "opcodes_def_generated.h"
#undef OPCODE
    NULL
};


size4u_t
    opcode_attribs[XX_LAST_OPCODE][(A_ZZ_LASTATTRIB / ATTRIB_WIDTH) + 1] = {0};

static void init_attribs(int tag, ...)
{
    va_list ap;
    int attr;
    va_start(ap, tag);
    while ((attr = va_arg(ap, int)) != 0) {
        opcode_attribs[tag][attr / ATTRIB_WIDTH] |= 1 << (attr % ATTRIB_WIDTH);
    }
}

void opcode_init(void)
{
    init_attribs(0, 0);

#define ATTRIBS(...) , ## __VA_ARGS__, 0
#define OP_ATTRIB(TAG, ARGS) init_attribs(TAG ARGS);
#include "op_attribs_generated.h"
#undef OP_ATTRIB
#undef ATTRIBS

    decode_init();

#define DEF_QEMU(TAG, SHORTCODE, HELPER, GENFN, HELPFN) \
    opcode_short_semantics[TAG] = #SHORTCODE;
#include "qemu_def_generated.h"
#undef DEF_QEMU
}


#define NEEDLE "IMMEXT("

int opcode_which_immediate_is_extended(opcode_t opcode)
{
    const char *p;
    if (opcode >= XX_LAST_OPCODE) {
        g_assert_not_reached();
        return 0;
    }
    if (!GET_ATTRIB(opcode, A_EXTENDABLE)) {
        g_assert_not_reached();
        return 0;
    }
    p = opcode_short_semantics[opcode];
    p = strstr(p, NEEDLE);
    if (p == NULL) {
        g_assert_not_reached();
        return 0;
    }
    p += strlen(NEEDLE);
    while (isspace(*p)) {
        p++;
    }
    /* lower is always imm 0, upper always imm 1. */
    if (islower(*p)) {
        return 0;
    } else if (isupper(*p)) {
        return 1;
    } else {
        g_assert_not_reached();
    }
}