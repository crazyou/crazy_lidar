#ifndef _TIME_DELAY_H_
#define _TIME_DELAY_H_

#include <unistd.h>

namespace NS_NaviCommon
{
  
  static inline void
  delay (unsigned long ms)
  {
    while (ms >= 1000)
    {
      usleep (1000 * 1000);
      ms -= 1000;
    };
    if (ms != 0)
      usleep (ms * 1000);
  }
  
// TODO: the highest timer interface should be clock_gettime
  
  unsigned long
  getUs ();
  unsigned int
  getMs ();
  
  static inline time_t
  getTimeStamp ()
  {
    time_t timestamp;
    time (&timestamp);
    return timestamp;
  }

}

#endif
