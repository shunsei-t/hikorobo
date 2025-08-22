#define LED_R LED_RED
#define LED_G LED_BLUE
#define LED_B LED_GREEN

#define PIN_PIT D3
#define PIN_ELE D9// SV3
#define PIN_AIL_L D7// SV1
#define PIN_AIL_R D8// SV2
#define PIN_SW D6

#define PULSE_WIDTH 1500

// BME280関連のパラメータ
// https://trac.switch-science.com/wiki/BME280
#define I2C_ADDR 0x76
#define OSRS_T 1             //Temperature oversampling
#define OSRS_P 1             //Pressure oversampling
#define OSRS_H 1             //Humidity oversampling
#define BME280MODE 3         //mode
#define T_SB 6               //Tstandby
#define FILTER 0             //Filter
#define SPI3W_EN 0           //3-wire SPI