Здесь представлены программы для двух МК для теста модуля SSP по протоколу SPI.
Для запуска на плату нужно создать проект в Keil5 и добавить туда main.c из соответствующих папок.
Кроме того, в файле _config.h нужно закомментировать строку с JTAG-B, так как по умолчанию он занимает порты, необходимые для работы SSP2
Подключение:
  - Connect SSP1 FSS pin (1.X13. 8) to SSP2 FSS pin (2.X14.21).
  - Connect SSP1 CLK pin (1.X13.10) to SSP2 CLK pin (2.X14.20).
  - Connect SSP1 RXD pin (1.X13. 7) to SSP2 TXD pin (2.X14.19).
  - Connect SSP1 TXD pin (1.X13.11) to SSP2 RXD pin (2.X14.22).