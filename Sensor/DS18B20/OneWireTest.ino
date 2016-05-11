// Подключение заголовочных файлов 1Wire библиотеки.
#include "Arduino.h"

#include "OWIPolled.h"
#include "OWIHighLevelFunctions.h"
#include "OWIBitFunctions.h"
#include "OWIcrc.h"


// Код семейства и коды команд датчика DS18B20.
/*#define DS18B20_FAMILY_ID                0x28
 #define DS18B20_CONVERT_T                0x44
 #define DS18B20_READ_SCRATCHPAD          0xbe
 #define DS18B20_WRITE_SCRATCHPAD         0x4e
 #define DS18B20_COPY_SCRATCHPAD          0x48
 #define DS18B20_RECALL_E                 0xb8
 #define DS18B20_READ_POWER_SUPPLY        0xb4*/

#define DS18B20_SKIP_ROM                 0xcc
#define DS18B20_CONVERT_T                0x44
#define DS18B20_READ_SCRATCHPAD          0xbe

// Номер вывода, к которому подключен датчик.
#define BUS   OWI_PIN_2

unsigned char scratchpad[9];

unsigned int tmp = 0;
unsigned char temperature;

void setup()
{
  OWI_Init(BUS);

  Serial.begin(115200);
}

void BCD_1Lcd(unsigned char value)
{
  value += 48;
  Serial.print(value);                  // ones
}

#define SYMB_NULL 32

void BCD_3Lcd(unsigned char value)
{
  unsigned char high = 0;
  unsigned char flag = 0;

  if (value >= 100) flag = 48;
  else flag = SYMB_NULL;
  while (value >= 100)                // Count hundreds
  {
    high++;
    value -= 100;
  }
  if (high) high += 48;
  else high = SYMB_NULL;
  Serial.print(high);

  high = 0;
  while (value >= 10)                 // Count tens
  {
    high++;
    value -= 10;
  }
  if (high) high += 48;
  else high = flag;
  Serial.print(high );

  value += 48;
  Serial.print(value);                  // Add ones
}

// Переменные для определения максимального времени цикла.
uint32_t timeCycleBegin = 0;
uint32_t timeCycle = 0;
uint32_t timeCycleMax = 0;

// Переменные для опроса часов 1 раз в сек.
uint32_t interval = 1000;
uint32_t previousMillis = 0;

uint32_t interval2 = 750;
uint32_t previousMillis2 = 0;

uint8_t temp = 0;

void loop()
{
  // Начальное время цикла.
  timeCycleBegin = micros();

  /* Подаем сигнал сброса
   команду для адресации всех устройств на шине
   подаем команду - запук преобразования */
  uint32_t currentMillis = millis();
  if (!temp && (currentMillis - previousMillis > interval))
  {
    previousMillis2 = previousMillis = currentMillis;

    OWI_DetectPresence(BUS);
    OWI_SkipRom(BUS);
    OWI_SendByte(DS18B20_CONVERT_T , BUS);

    temp = 1;
  }
  
  /* Ждем, когда датчик завершит преобразование. */
  //while(!OWI_ReadBit(BUS));

  /* Подаем сигнал сброса
   команду для адресации всех устройств на шине
   команду - чтение внутренней памяти
   затем считываем внутреннюю память датчика в массив.
   */

  currentMillis = millis();
  if (temp && (currentMillis - previousMillis2 > interval2))
  {
    OWI_DetectPresence(BUS);
    OWI_SkipRom(BUS);
    OWI_SendByte(DS18B20_READ_SCRATCHPAD, BUS);
    scratchpad[0] = OWI_ReceiveByte(BUS);
    scratchpad[1] = OWI_ReceiveByte(BUS);

    /* Выводим знак и преобразуем число, если оно отрицательное. */;
    if ((scratchpad[1] & 128) == 0) 
    {
      Serial.print('+');
    }
    else 
    {
      Serial.print('-');
      tmp = ((unsigned int)scratchpad[1] << 8) | scratchpad[0];
      tmp = ~tmp + 1;
      scratchpad[0] = tmp;
      scratchpad[1] = tmp >> 8;
    }

    /* Выводим значение целое знач. температуры. */
    temperature = (scratchpad[0] >> 4) | ((scratchpad[1] & 7) << 4);
    Serial.print(temperature);

    /* Выводим дробную часть знач. температуры. */
    temperature = (scratchpad[0] & 15);
    temperature = (temperature << 1) + (temperature << 3);
    temperature = (temperature >> 4);
    Serial.print('.');
    Serial.print(temperature);
    Serial.print('\n');

    temp = 0;
  }


  // Определяем скорость работы.
  timeCycle = GetDifferenceULong(timeCycleBegin, micros());

  // Если текущее значение больше, последнего максимального, отображаем его.
  if (timeCycle > timeCycleMax)
  {
    timeCycleMax = timeCycle;

    Serial.print("Max - ");
    Serial.println(timeCycleMax);
  }
}

// Переменные для определения разницы между 2 uint32
#define unsignedLongMax 4294967295UL
#define digitalOne 1UL

// Функция которая возвращает разницу между 2-мя uint32.
uint32_t GetDifferenceULong(uint32_t BeginTime, uint32_t EndTime)
{
  // Защита от переполнения
  if (EndTime < BeginTime)
  {
    return unsignedLongMax - BeginTime + EndTime + digitalOne;
  }
  else
  {
    return EndTime - BeginTime;
  }
}


