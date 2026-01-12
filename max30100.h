#ifndef MAX3010X_H
#define MAX3010X_H


// ----------------- Configuração de hardware/sistema -----------------
#define I2C_PORT    i2c1     // Porta I2C usada (Pico tem i2c0 e i2c1)
#define SDA_PIN     2        // GP2 como SDA (I2C1)
#define SCL_PIN     3        // GP3 como SCL (I2C1)
#define MAX_ADDR    0x57     // Endereço I2C do MAX30102
#define FS_HZ       100.0f   // Taxa de amostragem desejada (100 Hz)

// ---------- Registradores do MAX30102 ----------
#define REG_INT_ST1     0x00
#define REG_INT_ST2     0x01
#define REG_INT_EN1     0x02
#define REG_INT_EN2     0x03
#define REG_FIFO_WR_PTR 0x04
#define REG_OVF_CNT     0x05
#define REG_FIFO_RD_PTR 0x06
#define REG_FIFO_DATA   0x07
#define REG_FIFO_CFG    0x08
#define REG_MODE_CFG    0x09
#define REG_SPO2_CFG    0x0A
#define REG_LED1_PA     0x0C   // Corrente do LED Vermelho (RED)
#define REG_LED2_PA     0x0D   // Corrente do LED Infravermelho (IR)
#define REG_PART_ID     0xFF   // Deve ler 0x15 no MAX30102

// ---------- Ajustes rápidos de operação ----------
/*
 * REG_SPO2_CFG (0x0A):
 *  - Bits [6:5] SPO2_ADC_RGE: 0b10 => 8192 nA (0x47) | 0b11 => 16384 nA (0x67)
 *  - Bits [4:2] SPO2_SR:      0b010 => 100 Hz
 *  - Bits [1:0] LED_PW:       0b11  => 18 bits (maior resolução)
 *
 * 0x47 = 0b 01 001 11  => range=8192 nA, SR=100 Hz, 18 bits
 * Se estiver saturando com facilidade, troque para 0x67 (range maior).
 */
#define SPO2_CFG_RANGE_SR_PW   0x47

/*
 * Corrente dos LEDs:
 *  - REG_LED1_PA (RED) e REG_LED2_PA (IR) medem ~0,2 mA por LSB.
 *  - 0x45 (69 decimal) => ~13,8 mA. Faixa típica: 0x30..0x60 (~9,6 a 19,2 mA).
 */
#define LED_PA  0x45

// ----- Heurísticas para decidir “dedo presente” -----
// DC muito baixo => dedo mal posicionado; DC muito alto => saturação/pressão excessiva
#define FINGER_DC_MIN     8000.0f
#define FINGER_DC_MAX     240000.0f
// SNR (= AC/DC) mínimo para considerar que há pulso confiável (0,30%)
#define FINGER_SNR_MIN    0.0030f

// ----- Parâmetros da detecção de batimento -----
// Refratário (em segundos): evita contar “ecos” 
// Nota: REFRACTORY_S impõe um teto prático de FC ≈ 60 / REFRACTORY_S.
#define REFRACTORY_S      0.55f     // 550 ms => FC máx prática ~109 bpm
// Limites de plausibilidade de RR (intervalo entre batidas)
#define RR_MIN_S          0.45f     // 133 bpm máx (checagem adicional)
#define RR_MAX_S          1.60f     // 37.5 bpm mín
// Rejeição de outliers relativos à média histórica (evita contagem dupla)
#define RR_LOWER_FRAC     0.80f     // rejeita RR < 80% da média (meia batida)
#define RR_UPPER_FRAC     1.60f     // rejeita RR > 160% (outlier lento)

#endif