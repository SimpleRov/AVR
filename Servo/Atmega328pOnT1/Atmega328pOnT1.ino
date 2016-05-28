// Пример для Atmega328p на T1.

/// <summary>
/// Настройка при запуске Аrduino.
/// </summary>
void setup() 
{
  // Подключаем 9 пин ардуино, B1 для Atmega328p.  
  DDRB |=(1<<PB1);     

  // TCCR1A - регистр управления A.
  TCCR1A = 0;

  // TCCR1B - регистр управления B.
  TCCR1B = 0;

  // TCCR1A - регистр управления A.
  // TCCR1B - регистр управления B.
  // Биты WGM13 (4) , WGM12 (3) регистра TCCR1B и биты WGM11 (1) , WGM10 (0) регистра TCCR1A устанавливают режим работы таймера/счетчика T1:
  // 0000 - обычный режим
  // 0001 - коррекция фазы PWM, 8-бит
  // 0010 - коррекция фазы PWM, 9-бит
  // 0011 - коррекция фазы PWM, 10-бит
  // 0100 - режим счета импульсов (OCR1A) (сброс при совпадении)
  // 0101 - PWM, 8-бит
  // 0110 - PWM, 9-бит
  // 0111 - PWM, 10-бит
  // 1000 - коррекция фазы и частоты PWM (ICR1)
  // 1001 - коррекция фазы и частоты PWM (OCR1A)
  // 1010 - коррекция фазы PWM (ICR1)
  // 1011 - коррекция фазы и частоты PWM (OCR1A)
  // 1100 - режим счета импульсов (ICR1) (сброс при совпадении)
  // 1101 - резерв
  // 1110 - PWM (ICR1)
  // 1111 - PWM (OCR1A)

  // 1110 - PWM (ICR1)
  TCCR1A |= (1 << WGM11) | (0 << WGM10);
  TCCR1B |= (1 << WGM12) | (1 << WGM13);

  // ICR1 - регистр захвата (16 бит)
  ICR1 = 5000;

  // OCR1A - регистр сравнения A (16 бит)
  OCR1A = 150;

  // TCCR1A - регистр управления A.
  // Биты COM1A1 (7) и COM1A0 (6) влияют на то, какой сигнал появится на выводе OC1A (15 ножка) при совпадении с A (совпадение значения счетного регистра TCNT1 со значением регистра сравнения OCR1A):
  /* 1. Обычный режим
  00 - вывод OC1A не функционирует
  01 - изменение состояния вывода OC1A на противоположное при совпадении с A
  10 - сброс вывода OC1A в 0 при совпадении с A
  11 - установка вывода OC1A в 1 при совпадении с A
      2. Режим ШИМ
  00 - вывод OC1A не функционирует
  01 - если биты WGM13 - WGM10 установлены в (0000 - 1101), вывод OC1A не функционирует
  01 - если биты WGM13 - WGM10 установлены в 1110 или 1111, изменение состояния вывода OC0A на противоположное при совпадении с A
  10 - сброс вывода OC1A в 0 при совпадении с A, установка  вывода OC1A в 1 если регистр TCNT1 принимает значение 0x00 (неинверсный режим)
  11 - установка вывода OC1A в 1 при совпадении с A, установка  вывода OC1A в 0 если регистр TCNT1 принимает значение 0x00  (инверсный режим)
      3. Режим коррекции фазы ШИМ
  00 - вывод OC1A не функционирует
  01 - если биты WGM13 - WGM10 установлены в (0000 - 1100, 1010, 1100 - 1111), вывод OC1A не функционирует
  01 - если биты WGM13 - WGM10 установлены в 1101 или 1011, изменение состояния вывода OC1A на противоположное при совпадении с A
  10 - сброс вывода OC1A в 0 при совпадении с A во время увеличения значения счетчика, установка  вывода OC1A в 1  при совпадении с A во время уменьшения значения счетчика
  11 - установка вывода OC1A в 1 при совпадении с A во время увеличения значения счетчика, сброс  вывода OC1A в 0  при совпадении с A во время уменьшения значения счетчика
  */
  TCCR1A |= (1 << COM1A1) | (0 << COM1A0);

  // TCCR1B - регистр управления B.
  // Биты CS12 (2), CS11 (1), CS10 (0) регистра TCCR1B устанавливают режим тактирования и предделителя тактовой частоты таймера/счетчика T1:
  // 000 - таймер/счетчик T1 остановлен
  // 001 - тактовый генератор CLK
  // 010 - CLK/8
  // 011 - CLK/64
  // 100 - CLK/256
  // 101 - CLK/1024
  // 110 - внешний источник на выводе T1 (11 ножка) по спаду сигнала
  // 111 - внешний источник на выводе T1 (11 ножка) по возрастанию сигнала
  // 011 - CLK/64
  TCCR1B |= (0 << CS12) | (1 << CS11) | (1 << CS10);
}

/// <summary>
/// Основной цикл.
/// </summary>
void loop() 
{
  
}

