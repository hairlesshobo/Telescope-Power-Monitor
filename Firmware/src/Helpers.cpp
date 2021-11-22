#include "Helpers.h"

void write_p(Print &target, const char* str)
{
    char buf[strlen_P(str)+1];
    strcpy_P(buf, str);

	target.print(buf);
}

void writeLine_p(Print &target, const char* str)
{
    char buf[strlen_P(str)+1];
    strcpy_P(buf, str);

	target.println(buf);
}

boolean startsWith_p(const char *search, const char *str, uint8_t offset = 0)
{
	return strncasecmp_P(search + offset, str, strlen_P(str)) == 0;
}

void getTimestamp(DateTime *dt, char *dest)
{
	sprintf(dest, "%u-%02d-%02dT%02d:%02d:%02d", dt->year(), dt->month(), dt->day(), dt->hour(), dt->minute(), dt->second());
}

// void Write(Print &target, int8_t val)
// {
// 	Write(target, (int8_t)val);
// }

// void Write(Print &target, int16_t val)
// {
// 	Write(target, (int32_t)val);
// }

// void Write(Print &target, int32_t val)
// {
// 	char buf[12];
// 	ltoa(val, buf, 10);

// 	target.print(buf);
// }

// void Write(Print &target, uint8_t val)
// {
// 	Write(target, (uint32_t)val);
// }

// void Write(Print &target, uint16_t val)
// {
// 	Write(target, (uint32_t)val);
// }

// void Write(Print &target, uint32_t val)
// {
// 	char buf[11];
// 	ultoa(val, buf, 10);

// 	target.print(buf);
// }

// void Write(Print &target, float val, uint8_t precision = 2)
// {
// 	char buf[11];

// 	// Max precision of 3 for size sake
// 	if (precision > 3)
// 		precision = 3;
		
// 	itoa((int)val, buf, 10);

// 	if (precision > 0)
// 	{
// 		// add the decimal place
// 		strcat(buf, ".");

// 		val -= (int)val;

// 		ltoa((long)(val * pow(10, precision)), &buf[strlen(buf)], 10);
// 	}

// 	target.print(buf);
// }

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
