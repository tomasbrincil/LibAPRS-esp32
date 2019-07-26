#define cli() do{}while(0)
#define sei() do{}while(0)

#define _BV(b) (1UL << (b))


#include <string.h>
#include <stdlib.h>


#define F(x) (x)


#define DEC 10

class HardwareSerial {
public:

	// size_t print(const __FlashStringHelper *);
	// size_t print(const string &);
	size_t print(const char[]);
	size_t print(char);
	size_t print(unsigned char, int = DEC);
	size_t print(int, int = DEC);
	size_t print(unsigned int, int = DEC);
	size_t print(long, int = DEC);
	size_t print(unsigned long, int = DEC);
	size_t print(double, int = 2);
	// size_t print(const Printable&);
	// size_t print(struct tm * timeinfo, const char * format = NULL);

	// size_t println(const __FlashStringHelper *);
	// size_t println(const string &s);
	size_t println(const char[]);
	size_t println(char);
	size_t println(unsigned char, int = DEC);
	size_t println(int, int = DEC);
	size_t println(unsigned int, int = DEC);
	size_t println(long, int = DEC);
	size_t println(unsigned long, int = DEC);
	size_t println(double, int = 2);
	// size_t println(const Printable&);
	// size_t println(struct tm * timeinfo, const char * format = NULL);
	size_t println(void);
};

extern HardwareSerial Serial;