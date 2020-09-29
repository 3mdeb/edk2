#include <Include/PiDxe.h>
#include "own.h"

VOID *memset (VOID *ptr, int value, __SIZE_TYPE__ num ) {
  for(__SIZE_TYPE__ offset; offset < num; ++offset) {
    ((CHAR8 *)ptr)[offset] = (CHAR8)value;
  }
}
