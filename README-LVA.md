# Модернизированный [Sonoff-Tasmota](https://github.com/arendst/Sonoff-Tasmota)

Текущий номер релиза Sonoff-Tasmota 6.2.1.13 20181008

Необходимость данного форка вызвана тем, что для автоматизации дома на базе
**OpenHub** потребовалось изменить некоторые модули и написать новые модули для
поддержки новых устройств.

Прошивка обновляется по мере наличия свободного времени от строительства дома.

## Чем отличается от оригинала

1. *xsns_05_ds18x20_legacy.ino* - увеличено количество датчиков до 24. Более
    компактные поля вывода MQTT, т.к. не хватало длины стандартного MQTT
    сообщения.

2. Добавлена поддержка нескольких `ADS1115` (для автоматизации дома требуется
    16 ЦАП - обработка обратной связи с приводов воздушных заслонок).

3. На базе драйвера *xdrv_15_pca9685.ino* сделан сенсор
    *xsns_33_pca9685_lva.ino* в нем добавлен вывод информации на WEBGUI.

4. Сделан драйвер MODBUS to MQTT для `Danfoss FC51`.

Для облегчения поддержки и обновлений до текущих версий *Sonoff-Tasmota* в
оригинальные файлы вносятся минимальные изменения и добавляются собственные.

## Ближайшие задачи

- DS18B20 добавить поддержку двух ветвей 1-ware датчиков на одном устройстве.

- Сделать драйвер для счетчика `Меркурий 231` (MODBUS RTU -\> MQTT )

## Основные изменения оригинальных файлов Sonoff-Tasmota

### Список измененных файлов и причина их изменения

- **patformio. ini** - отключены лишние варианты сборок, в команде сборки включен ключ определения \*\*\*_LVA\*\*\* активизирующий все мои изменения (без него должен собираться оригинальный *Sonoff-Tasmota*), настроен ком.порт.

- **xdrv_02_webserver.ino** - добавлена возможность выделения цветом ячеек. К станадрному ключу замены `{m}` добавлены ключи `{mr}` и `{mr}`

- **sanoff/xdrv_interface.ino** - был добавлен дебаг вызова модулей, требуется при отладке собственных модулей

- **sanoff/xsns_12_ads1115_ada.ino** - переделан вывод строк.

- **sanoff/sonoff.ino** - добавлен `DEBUG` для MQTT сообщений. Подключается в *lva_post.h*.

- **user_config.h** - не все можно выключить (например `#define USE_I2C`) через **user_config_override.h**, по этому правим user_config.h

- **user_config_override.h** - мои переопределение \#define по умолчанию. Здесь надо указать под вариант моей сборки. Включаем стандартную сборку *Sonoff-Tasmota* `USE_CLASSIC`.

- **sanoff/xdrv_interface.ino** - добавлен `LVA_DEBUG` на сериал при выполнении функции `XdrvCommand`.

- **sanoff/xsns_05_ds18x20_legacy.ino** - увеличено количество датчиков до 24 (зачем столько, если их всего у меня максимум на шине 16, а вспомнил для двух шин ;) ), сокращен вывод информации на webpage и MQTT. (*надо довести до ума, бросил*.)

- **sanoff/xsns_12_ads1115.ino** - стандартная библиотека переделанная на сокращенный вывод MQTT. Только одну микросхему поддерживает. Устарела надо использовать *xsns_12_ads1115_lva.ino*. \__\_ Во всех откорректированных оригинальных файлах в пятой строке добавлен текст `UPDATED LVA`

Изменения в самом тексте обрамлены строками:

```md
// LVA <--
#ifndef \_LVA
#else
#endif
//  LVA  -->
```

### Новые файлы

- **sanoff/lva_post.h** - здесь отключаем все лишнее из `USE_CLASSIC`, и
    делаем свои компоновки. Запускается последним из конфигурационных ".h".
    Файлом *lva_post.h* уточняем последними \#define (если чего-то надо
    переопределить).

- **sanoff/xsns_12_ads1115_lva.ino** - собственная библиотека поддержки 4
    микросхем, надо проверять на совместимость с последней версией *Tasmota*.

- **sanoff/xsns_91mcp_lva.ino** - собственная библиотека для `MCP23017`, ранее
    в *Tasmota* не было поддержки `MCP23017`, сейчас появилась
    (**xsns_29_mcp230xx.ino**) надо разбираться. Возможно надо убирать.

- **xsns_34_DanfosFC51.ino** - собственный драйвер для частотника `Danfos
    FC51` умеет включать, выключать, изменять частоту подаваемую на привод.
    Частота задается в процентах от 0 до 100% от максимально установленной в
    параметре 3-30. Драйвер умеет работать как с `МАX485` (есть задаваемый в
    настройках пин для дерганья в момент передачи) так и с более продвинутыми
    микросхемами-драйверами RS485. Пока тестировался только на скорости `19200`,
    при больших скоростях возможно надо будет добавить время на реакцию `DFC51`.

### Разбираем как работает *Sonoff-Tasmota* (шпаргалка, чтобы запомнить)

#### MODBUS

Новая версия *Sonoff-Tasmota* научилась пуллить по MODBUS разные устройства
используя программный сериал на скорости 9600.

- **xsns_23\_ sdm120 .ino** - Eastron SDM120 -Modbus-измеритель мощности. По
    умолчанию включено варианте сборки **sensors**. Поддержка включается в
    **sonoff_post.h**. Функция `SDM120250ms` опрашивает устройство и читает 8
    адресов по порядку, причет в функции сначала проверяется чтение, а только
    потом посылается запрос устройству. Только чтение и публикация по MQTT 8
    параметров.

- **xnrg_05_pzem2.ino** - PZEM-003,017 и PZEM-014,016 Поддержка датчика
    энергопотребления Modbus.

- **xsns_17_senseair.ino** - Поддержка датчика CO2 SenseAir K30, K70 и S8.

все эти датчики не плохой вариант считывать меркурий 230, думаю должны быть
очень похожие таблицы.

Сравнивая эти модули, думаю самым аккуратно написанным является **xsns_23\_
sdm120.ino**

Соединения на время отладки

```md
esp D6 -> TX -> mcp D6
esp D7 -> RX -> mcp D7
esp D5 -> TX_E ->mcp D5
```

#### Порядок обработки *\#define*

сначала обрабатывается *sonoff.ino*, где включается *user_config.h* и т.д.

#### Название модулей

- _xdr__ - драйвера

- _xdsp\__ - модули для работы с дисплеями

- _xnr__ - модули для работы с сенсорами мощности

- _xsns\__ - модули для работы с прочими сенсорами

### Сейчас работает два модуля ESP (WEMOS d1 mini V2)

1. **Камин.** Датчики температуры (5 шт.) подключены к `D4`.

2. **Отопление.** Датчики температуры (11 шт) подключены к `D4` ) модуль
    отопления постоянно висит, горит светодиод `GPIO2 (D4)` или он все таки на
    `ТХ (GPI01)` пине?

Надо сделать по принципу оригинала: разные ключи под прошивки для разных
устройств:

- **камин** (температурные датчики 5 шт., электрический счетчик отопления)

- **отопление** (температурные датчики, расширитель портов для контроля
    напряжений, управления АВР дизеля)

- **Меркурий** (для счетчика на столбе)

- **чердак** (данфосы, de-icing, температурные датчики, управление приводами
    вентиляции)

### BE_MINIMAL + user_config_overgrive

```md
DATA:    [======    ]  58.5% (used 47952 bytes from 81920 bytes)
PROGRAM: [===       ]  33.2% (used 347915 bytes from 1048576 bytes)
```

!!! При этом выключили USE_KNX_WEB_MENU

## xsns_29_mcp230xx.ino

Взято от [сюда](https://github.com/arendst/Sonoff-Tasmota/wiki/MCP23008-MCP23017).

*на WebGui выводятся только порты которые сконфигурированы на OUT !!!* Формат
команды в консоли (как в MQTT пока не понятно)

### спросить состояние пина D9

```md
sensor29 9,?
15:06:25 RSL: RESULT = {"Sensor29_D9":{"MODE":1,"PULL_UP":"OFF","INT_MODE":"DISABLED","STATE":"OFF"}}
```

### Настройка пинов, общий синтаксис

```md
sensor29 pin,pinmode,pullup
```

где

- `pin`: от 0 до 15
- `pinmode`:
- 1 = `INPUT` (данные телеметрической передачи с плавающей точкой будут
    отправляться в соответствии с интервалами конфигурации прошивки Tasmota
- 2 = `INPUT` с `INTERRUPT` на `CHANGE` (Будет отправлен вывод MQTT при
    изменении состояния от `LOW` до `HIGH` и `HIGH` до `LOW`)
- 3 = `INPUT` с `INTERRUPT` на `CHANGE` до `LOW` (отправит вывод MQTT с
    изменением состояния только от `HIGH` до `LOW`)
- 4 = `INPUT` с `INTERRUPT` на `CHANGE` до `HIGH` (выведет выход MQTT с
    изменением состояния только от `LOW` до `HIGH`)
- 5 = `OUTPUT` (если включено с `#define USE_MCP230xx_OUTPUT`)
- `pullup` = Рабочий режим штыря следующим образом:
- 0 = подтяжка ОТКЛЮЧЕНА
- 1 = подтяжка ВКЛЮЧЕНА
- `intmode` = Необязательно указать режим представления прерываний следующим
    образом - по умолчанию 0, если не указано
- 0 = Отчет о немедленном прерывании с использованием TELEMETRY и EVENT

- 1 = Только немедленное СОБЫТИЕ (телеметрия не указана)

- 2 = Непосредственная ТЕЛЕМЕТРИЯ (не вызвано событием)

Примеры некоторых вариантов конфигурации контактов:

```md
sensor29 4,1,0_ - Включит `D4` для `INPUT` без внутреннего подтягивающего резистора
sensor29 3,1,1_ - Включит `D3` для `INPUT` с внутренним подтягивающим резистором `ENABLED`
sensor29 5,2,1_ - Включит `D5` для `INPUT` и сообщит о состоянии изменения от `LOW до HIGH` и `HIGH до LOW` через MQTT
sensor29 6,3,1_ - Включит `D6` для `INPUT` и сообщит о состоянии изменения от `HIGH` до `LOW` (также включена подсветка)
sensor29 2,4,0_ - Включит `D2` для `INPUT` и сообщит о состоянии изменения от `LOW до HIGH` (заметка подтягивания не включена)
```

### Сброс и настройка всех пинов сразу

```md
sensor29 reset    // Reset all pins INPUT, no interrupt, no pull-up by default
sensor29 reset1   // Reset all pins INPUT, no interrupt, no pull-up by default
sensor29 reset2   // Reset all pins INT on CHANGE, with pull-up enabled by default
sensor29 reset3   // Reset all pins INT on LOW, with pull-up enabled by default
sensor29 reset4   // Reset all pins INT on HIGH, with pull-up enabled by default
sensor29 reset5   // Reset all pins to OUTPUT mode (if enabled by #define USE_MCP230xx_OUTPUT)
```

### Примеры

```md
sensor29 0,5,0  // Configure pin 0 as OUTPUT and default to OFF on reset/power-up
sensor29 0,5,1  // Configure pin 0 as OUTPUT and default to ON on reset/power-up
sensor29 0,6,0  // Configure pin 0 as INVERTED OUTPUT and default to ON on reset/power-up
sensor29 0,6,1  // Configure pin 0 as INVERTED OUTPUT and default to OFF on reset/power-up
sensor29 9,5,0 - пин на OUT по умолчанию выключен
sensor29 9,5,1 - пин на OUT по умолчанию включен
sensor29 9,ON   // Turn pin ON (HIGH if pinmode 5 or LOW if pinmode 6(inverted))
sensor29 9,OFF  // Turn pin OFF (LOW if pinmode 5 or HIGH if pinmode 6(inverted))
sensor29 9,T    // Toggle the current state of pin from ON to OFF, or OFF to ON
sensor29 reset2
sensor29 9,5,0
sensor29 9,ON
sensor29 9,OFF
sensor29 9,T
```

### Напоминалка

1. На отладочной плате кнопка на D4, светодиоды на: D8 (снят резестор), D9, D10.

2. Полезная опция *I2Cscan*

## PCA9685: 16-channel, 12-bit PWM I²C-bus LED controller

### В оригинале (нет вывода информации на WEbGUI)

```md
driver15 pwmf,frequency   // where frequency is the PWM frequency from 24 to 1526 in Hz
driver15 pwm,pin,pwmvalue // where pin = LED pin 0 through 15 and pwmvalue is the pulse width between 0 and 4096
driver15 pwm,pin,ON       // Fully turn a specific pin/LED ON
driver15 pwm,pin,OFF      // Fully turn a specific pin/LED OFF
driver15 reset            // Reset to power-up settings - i.e. F=50hz and all pins in OFF state
driver15 pwm,15,OFF
driver15 pwm,15,ON
driver15 pwm,15,1024
driver15 pwm,15,10
//driver15
```

### У меня

```md
sensor33 pwm,15,ON
sensor33 pwm,15,3900
sensor33 pwm,15,OFF
sensor33 pwm,15,ON
sensor33 pwm,15,ON
sensor33 pwm,15,4096
sensor33 pwm,15,4095
sensor33 pwm,15,4095
sensor33 pwm,13,4096
sensor33 pwm,14,40
```

## FC51 Danfos

Текущая основная проблема, у меня микросхема-драйвер `MAX485`, она требует
переключения прием/передача, надо третьей ногой включать/выключать передачу.
Править библиотеку TasmotaSerial не надо, достаточно перед записью в порт
опускать ногу, а по окончании поднимать. см. ниже раздел `дергаем ногой`.

### Уже умеем :)

1. Поддержка нескольких частотников (легко увеличить достаточно изменить
    `FC51_DEVICES`, тестировалось на 3!).

2. Расширенная диагностика статуса частотника с выводом информации на WebGUI и
    MQTT (возможно лишнее, но это легко отключить).

3. Включать выключать мотор Доступны ключи: Для включения `START`, `ON`, `2`.
    Для выключения `STOP`, `OFF`, `1`. Регистр неважен (`Stop`, `stop stoP` -
    все пройдет).

4. Управлять скоростью привода. Скорость задается ключом `SPEED` в процентах (0...100%)
    от максимальной (`MaxFRQ`), только целые значения! (дробная часть отбрасывается) Для
    того, чтобы вычислить истинную скорость мотора в диагностической информации
    имеется параметр `MaxFRQ`, с размерностью в Hz. Если подать изменение
    скорости, то после задания новой частоты, если привод был выключен, то он
    включится. Если задать скорость `0`, то привод выключится, при этом задание
    новой скорости не произойдет (сделано для упрощения написания скриптов в
    *OpenHub*). Ключ `SPEED`

5. Для включения MODBUS необходимо задать ноги `MODBUS Tx` и `MODBUS Rx`.
    Опционально можно задать ноку для `MODBUS TX ENABLE` (если у вас как у меня
    MAX485, которая не умеет сама следить за передачей).

6. Проверена интеграция *OpenHub*. Работает: отображение статусов, включение
    выключение привода, изменение скорости привода.

7. Добавлена подсветка статуса в WebGui: зеленый - стоп, красный - пуск.

8. Оставлено много комментариев и закомментированной отладочной информации т.к.
    версия драйвера предварительная и будет дорабатываться, особенно после
    интеграции с *OpenHub*.

### Не подучилось

Не удалось сделать массив структур для хранения данных (можно было бы сэкономить
память), код был бы красивее. Пока заготовки оставил, может доведу до ума.

### Примеры команд

#### вытяжка кухня

```md
sensor94 1 start
sensor94 1 STOP
```

#### вытяжка СУ

```md
sensor94 2 start
sensor94 2 STOP
sensor94 2 SPEED 0
sensor94 2 SPEED 10
sensor94 2 SPEED 100
```

#### приточка

```md
sensor94 3 ON
sensor94 3 OFF
```

#### Настройки *Openhub*

##### items

TBD

#### #sitemaps

TBD

### Как делал

#### дергаем ногой

*sonoff_template.h* : 1. в *enum UserSelectablePins* добавляем название
переменной (`GPIO_MODBUS_TX, GPIO_MODBUS_RX, GPIO_MODBUS_TX_ENABLE`) 2. *const
char kSensorNames[] PROGMEM* добавляем имя шаблона во ФЛЕШ памяти для
отображаемого наименования пина (`D_SENSOR_MODBUS_TX "|" D_SENSOR_MODBUS_RX "|"
D_SENSOR_MODBUS_TX_ENABLE "|"`) 3. *const uint8_t kGpioNiceList[GPIO_SENSOR_END]
PROGMEM* добавляем название переменной (`GPIO_MODBUS_TX, GPIO_MODBUS_RX,
GPIO_MODBUS_TX_ENABLE`) 4. в языковом файле объявляем шаблон:

```cpp
#define D_SENSOR_MODBUS_TX "MODBUS Tx"
#define D_SENSOR_MODBUS_RX "MODBUS Rx"
#define D_SENSOR_MODBUS_TX_ENABLE "MODBUS ENABLE"
```

#### подкрашиваем статус

В оригинальном файле `xdrv_02_webserver.ino` добавляем два ключа `{mg}` и `{mg}`
путем замены строчки

```cpp
    // LVA EDIT
    //"var s=x.responseText.replace(/{t}/g,\"<table style='width:100%'>\").replace(/{s}/g,\"<tr><th>\").replace(/{m}/g,\"</th><td>\").replace(/{e}/g,\"</td></tr>\").replace(/{c}/g,\"%'><div style='text-align:center;font-weight:\");"
    "var s=x.responseText.replace(/{t}/g,\"<table style='width:100%'>\").replace(/{s}/g,\"<tr><th>\").replace(/{m}/g,\"</th><td>\").replace(/{mr}/g,\"</th><td bgcolor=OrangeRed>\").replace(/{mg}/g,\"</th><td bgcolor=Lime>\").replace(/{e}/g,\"</td></tr>\").replace(/{c}/g,\"%'><div style='text-align:center;font-weight:\");"
    // END LVA 
```

Пример использования в теле модуля ключей подстановки

```cpp
const char HTTP_FC51_STR_4[] PROGMEM =  "%s" "{s}" D_FC51 ":%i %s:" "{m}%s"        "{e}"; обычный
const char HTTP_FC51_STR_4R[] PROGMEM = "%s" "{s}" D_FC51 ":%i %s:" "{mr}%s"       "{e}"; красный
const char HTTP_FC51_STR_4G[] PROGMEM = "%s" "{s}" D_FC51 ":%i %s:" "{mg}%s"       "{e}"; зеленый
```

Памятка **надо делать команду на RESET !!! (что была ;) )**

## Прочие шпаргалки

- ПРИМЕР ВСТАВКИ КАРТИНКИ

- Как найти в файлах всей папки - сочетание клавиш Shift + Ctrl + F. Замена во
    всех файлах проекта также делается.

- [Правила оформления файла README.MD на GITHUB]
    (http://webdesign.ru.net/article/pravila-oformleniya-fayla-readmemd-na-github.html)

### Особенности лога в консоль

Надо чтобы был установлен уровень **LOG_LEVEL_INFO**

```cpp
snprintf_P(log_data, sizeof(log_data), PSTR(D_LOG_DEBUG "MODBUS inited"));
AddLog(LOG_LEVEL_INFO);
```

### Diptrace

китайская кнопка ese-20c(d)4-3.step (только здесь 4 контакта)
