/*
 * $FILE: bitop.h
 *
 * Some basic bit operations
 *
 * $VERSION: 1.0
 *
 * Author: Miguel Masmano <mimastel@doctor.upv.es>
 *
 * $LICENSE:  
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
 *
 */

#ifndef _BITOP_H_
#define _BITOP_H_

/*#ifndef _KERNEL_
#error Internal file. Do not include it in your sources.
#endif*/

/*
 * both, _ffs and _fls have been implemented for a 32 bit arch
 * TODO: Implementing it for a generic arch
 */

#ifndef ARCH_HAS_FFS

//------//
// _ffs //
//------//

static inline int _ffs (int x) {
  int r = 0;

  if (!x)
    return -1;

  if (!(x & 0xffff)) {
    x >>= 16;
    r += 16;
  }

  if (!(x & 0xff)) {
    x >>= 8;
    r += 8;
  }

  if (!(x & 0xf)) {
    x >>= 4;
    r += 4;
  }

  if (!(x & 0x3)) {
    x >>= 2;
    r += 2;
  }

  if (!(x & 0x1)) {
    x >>= 1;
    r += 1;
  }

  return r;
}

#endif

#ifndef ARCH_HAS_FLS

//------//
// _fls //
//------//

static inline int _fls (int x) {
  int r = 31;

  if (!x)
    return -1;
  
  if (!(x & 0xffff0000)) {
    x <<= 16;
    r -= 16;
  }
  if (!(x & 0xff000000)) {
    x <<= 8;
    r -= 8;
  }
  if (!(x & 0xf0000000)) {
    x <<= 4;
    r -= 4;
  }
  if (!(x & 0xc0000000)) {
    x <<= 2;
    r -= 2;
  }
  if (!(x & 0x80000000)) {
    x <<= 1;
    r -= 1;
  }
  return r;
}

#endif

#ifndef ARCH_HAS_SET_BIT

//----------//
// _set_bit //
//----------//

static inline void _set_bit(int nr, volatile unsigned long *addr) {
  (*addr) |= (1 << nr);
}

#endif

#ifndef ARCH_HAS_CLEAR_BIT

//------------//
// _clear_bit //
//------------//

static inline void _clear_bit(int nr, volatile unsigned long *addr) {
  (*addr) &= ~(1 << nr);
}

#endif

#endif
