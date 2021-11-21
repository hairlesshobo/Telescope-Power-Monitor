#include "Helpers.h"

float getAvgReading(float* readingArray, uint8_t avgCount)
{
  int count = 0;
  float total = 0.0;

  for (int i = 0; i < avgCount; i++)
  {
    total += readingArray[i];
    count++;
  }

  return (float)total/count;
}

// float getMinReading(float* readingArray, uint8_t avgCount)
// {
//   float lastMin = 0;
  
//   for (int i = 0; i < avgCount; i++)
//   {
//     if (readingArray[i] <= lastMin || lastMin == 0)
//       lastMin = readingArray[i];
//   }

//   return lastMin;
// }

// float getMaxReading(float* readingArray, uint8_t avgCount)
// {
//   float lastMax = 0.0;
  
//   for (int i = 0; i < avgCount; i++)
//   {
//     if (readingArray[i] >= lastMax)
//       lastMax = readingArray[i];
//   }

//   return lastMax;
// }

#ifdef __arm__
// should use uinstd.h to define sbrk but Due causes a conflict
extern "C" char *sbrk(int incr);
#else  // __ARM__
extern char *__brkval;
#endif // __arm__

int getFreeMemory()
{
    char top;
#ifdef __arm__
    return &top - reinterpret_cast<char *>(sbrk(0));
#elif defined(CORE_TEENSY) || (ARDUINO > 103 && ARDUINO != 151)
    return &top - __brkval;
#else  // __arm__
    return __brkval ? &top - __brkval : &top - __malloc_heap_start;
#endif // __arm__
}