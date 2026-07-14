# Sensor de suelo 7-en-1 (RS485) — fallos del nodo N01

Diagnóstico del 9 de julio de 2026 y parche de firmware.

## Archivos

| Archivo | Qué es |
|---|---|
| `n01_original.ino` | Copia exacta de `Tesis/Tesis/codes_esp.txt`, tal como está flasheado hoy. Contiene el firmware del nodo N01 y, más abajo en el mismo archivo, el del gateway N03. |
| `n01_corregido.ino` | El mismo archivo con los dos arreglos aplicados. Solo cambia la parte del nodo N01; el gateway queda intacto. |

## El síntoma

Alertas de WhatsApp por "humedad de suelo = 0 %". El nodo transmitía `0.0`, que no es el sentinela `-99` de sensor caído, así que el puente lo guardaba como una medición legítima y el evaluador de alertas lo comparaba contra el umbral mínimo de 15 %.

El 9 de julio, de 291 tramas, el bloque de suelo falló en 191 (**65,6 %**), frente al 18,6 % histórico. El BME280, el SCD40 y el ADC de batería estuvieron al 0 % de fallo ese mismo día.

## Lo que NO era

**No era el MOSFET.** Los cuatro sensores cuelgan del mismo transistor de GPIO 7 (el 7-en-1 con su elevador a 12 V y su MAX485 incluidos). Si el riel fallara, el BME280 y el SCD40 caerían con él, y estuvieron perfectos.

**No era la batería.** Voltaje medio de 4,02 V en las tramas buenas contra 3,99 V en las fallidas. Hacia las 16 h el porcentaje de fallo *bajó* mientras el voltaje seguía cayendo.

**El BH1750 al 100 % de NULL no es un fallo:** ese sensor ya no está conectado físicamente.

## Lo que sí era

En las tramas corruptas la conductividad no es ruido, es una **rampa cuantizada**:

| CE observada | pH | N / P / K | humedad |
|---|---|---|---|
| 171 | 6,38–6,42 | 22 / 31 / 99 | 0 |
| 311–312 | 6,39–6,42 | 35 / 49 / 104 | 0 |
| 498–499 | 6,39–6,42 | 37 / 52 / 104 | 0 |
| 524–542 | 6,40–6,41 | 38 / 54 / 108 | 0 |
| **540 (sana)** | **6,40** | **38 / 54 / 108** | **21–24** |

La secuencia 171 → 312 → 499 → 542 converge al valor sano. El NPK la sigue paso a paso porque en estos sensores se deriva de la conductividad. El pH, el registro más estable, siempre sale bien.

Es un **sensor al que se le pregunta antes de que termine de asentarse**. Sale de reset detrás de su elevador de voltaje, y en `encenderSensores()` solo transcurren unos 2,9 s entre encender el MOSFET y la primera lectura Modbus (`delay(800)` + inicialización I2C + `delay(2000)`). En la sesión de madrugada alcanzó (52 de 52 lecturas buenas); en la de la tarde, no.

## Bug de firmware que amplificaba el daño

En `leerSensorSuelo()` las siete variables se inicializan en `0` y **solo se comprueba el retorno de la primera lectura**, la de humedad:

```c
uint16_t v_hum=0, v_temp=0, v_ec=0, ...;      // todas en 0
bool ok = leerRegistroSuelo(0x0012, v_hum);
if (!ok) { ok_suelo = false; return; }
leerRegistroSuelo(0x0013, v_temp);             // retorno ignorado
leerRegistroSuelo(0x0015, v_ec);               // retorno ignorado
...
ok_suelo = true;                               // válido pase lo que pase
```

Si un registro no responde tras sus tres reintentos, su variable se queda en `0` y se transmite como dato real en lugar del sentinela. De ahí las filas de la BD con `temp_suelo = 0` o `N = 0, P = 3, K = 34`.

## Los dos arreglos

**1. Esperar a que el sensor se asiente.** Nueva función `esperarSensorAsentado()`, llamada al inicio de `leerSensorSuelo()`. Sondea la CE y la humedad cada 400 ms hasta que la CE deja de subir (dos lecturas consecutivas difieren en menos de 10) y la humedad deja de marcar 0 exacto, con un tope de 10 s. Si no se asienta, la trama de suelo se descarta entera.

Es una espera *condicional*, no un `delay()` más largo: en cuanto el sensor está listo continúa, así que no castiga la batería en los ciclos buenos.

**2. Comprobar los siete retornos.** Cada registro tiene ahora su propio flag (`ok_s_hum`, `ok_s_temp`, …) y viaja al gateway con su propio sentinela `INV_U16` / `INV_I16`, que el gateway traduce a `-99` y el puente convierte en `NULL`. Un registro mudo ya no puede disfrazarse de cero.

Además, `humedad == 0` exacto y `pH == 0` exacto se marcan como inválidos: en tierra ninguno de los dos es físicamente 0.00 (la escala real del sensor va de ~10 % en seco a ~35 % en tierra saturada). Y si la humedad no es fiable, se descarta el bloque de suelo completo, porque la CE viene sin asentar y el NPK se deriva de ella.

## Cómo flashear

Arduino IDE, placa Heltec WiFi LoRa 32 V4. Copia el contenido de la sección del nodo N01 de `n01_corregido.ino` (todo lo anterior al segundo bloque `#define INV_U8`, que ya es el gateway). Las librerías no cambian: RadioLib, U8g2, SparkFun SCD4x, ModbusMaster, Wire.

Tras flashear, en el monitor serie deberías ver `[SUELO] asentado en NNNN ms` en cada ciclo bueno. Ese número te dice cuánto tarda realmente el elevador en levantar; si se acerca a los 10 s del timeout, el MT3608 está arrancando demasiado lento y ahí sí hay que mirar el hardware.

## Nota sobre el puente

Aparte de esto, `puente/puente_final.py` ya filtra el síntoma en la Raspberry: el rango físico de `sh` pasó de `(0, 100)` a `(0.01, 100)` y una guarda nueva anula el bloque de suelo entero cuando la humedad llega en 0 exacto. Eso protege la BD aunque el nodo siga con el firmware viejo, pero **no cura el origen**: las lecturas se siguen perdiendo, solo que ahora se guardan como NULL en vez de como ceros falsos.

Quedan **212 filas históricas** con `humedad_suelo = 0` ya en la base de datos, anteriores al filtro, sin limpiar.
