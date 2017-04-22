                                                                                            
 /*
  * Copyright (C) 2002     Manuel Novoa III
  * Copyright (C) 2000-2005 Erik Andersen <andersen@uclibc.org>
  *
  * Licensed under the LGPL v2.1, see the file COPYING.LIB in this tarball.
  */
#include <stddef.h> 

void *memset(void *s, int c, size_t n)
 {
     register unsigned char *p = (unsigned char *) s;
 
     while (n) {
         *p++ = (unsigned char) c;
         --n;
     }
 
     return s;
 }

