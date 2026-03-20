#include <Arduino.h>
#include <SPI.h>
#include "ldc1101.h"

#define LDC_MOSI   9
#define LDC_MISO   8
#define LDC_SCK    7
#define LDC_CS     5


#define F_CLKIN 16000000.0f //This is based on the 16MHz crystal selected by Alex


// Registers
#define REG_RP_SET        0x01// This register sets the range of RP values that we want, there is a default that is used for a wide range of uses most likely what the GUI use // You set RPmin and Rpmax as well as account for inductors with high Q, Q>50 , Used the inductor charts given to estimate this
#define REG_TC1           0x02// This register is for time constant 1, you choose the capacitance which correlates to a time constant, there is an equation with R1 and C1 that you configure in this register, used fmin as the frequency x 0.75 to e conservative a the inductance will change the frequency 
#define REG_TC2           0x03// Uses C_snesor, Rp_min fom register 1, and C1 and R1 are then chosen from that, choosing the larger capacitor will help with noise 
#define REG_DIG_CONF      0x04// This sets minimum frequency we are being conservative 0.75 * frequency, so for our uses 2MHz to 1.5MHz, also sets response time higher reponse time is said to be more accurate 
#define REG_ALT_CONFIG    0x05// Allows for shutdowns and L_optimization only really used if using LHR mode
#define REG_INTB_MODE     0x0A// Allows interrupt mode where we can continously do something else and then revieve RP + L measurments when done not used but has to be reset
#define REG_START_CONFIG  0x0B// Sets the mode that we are in Active conversion, sleep, and  shutdown. In order to change registers for set up it must be in sleep however there are some you can change in active conversion mode
#define REG_CHIP_ID       0x3F// register with the ID we read just to make sure SPI connection is correct 

// These nest registers are the ones we care to read there are around 10 more we can change for RP + L but that can be used for actual detection such as RPthreshholds and whatnot 
#define REG_RP_DATA_MSB  0x22// Raw data of RP, there is a conversion to usbale data
#define REG_RP_DATA_LSB  0x21

#define REG_L_DATA_MSB   0x24// Raw data of L, there is a conversion to usable data
#define REG_L_DATA_LSB   0x23

#define REG_STATUS       0x20// We use this register in order to see if there is a measurment ready if so then wen take the data


typedef struct { float resistance; uint8_t bits; } rp_entry_t;
typedef struct { uint8_t reg; float rp_min_ohms; float rp_max_ohms; uint8_t high_q; } rp_config_t;
typedef struct { uint8_t reg; float response_time_periods; float conversion_time_sec; } dig_conf_result_t;

static rp_config_t rp_cfg_global;
static dig_conf_result_t dig_cfg_global;


static const rp_entry_t rp_table[8] = {
    {96000.0f, 0b000},
    {48000.0f, 0b001},
    {24000.0f, 0b010},
    {12000.0f, 0b011},
    {6000.0f,  0b100},
    {3000.0f,  0b101},
    {1500.0f,  0b110},
    {750.0f,   0b111}
};




static rp_config_t ldc1101_make_rp_set(float L_h,float Q,float C_sensor)
{
    //takes tand calculates the nominal rp and uses safety margins to find the min and max rp with a look up table 
    rp_config_t result;

    float f = 1.0f / (2.0f * M_PI * sqrtf(L_h * C_sensor));
    float Rp_est = Q * 2.0f * M_PI * f * L_h;

    uint8_t high_q_bit = (Q > 50.0f) ? 1 : 0;

   float upper = 1.25f * Rp_est;
  

    uint8_t rp_max_bits = rp_table[0].bits;
    uint8_t rp_min_bits = rp_table[7].bits;

    float rp_max_val = rp_table[0].resistance;
    float rp_min_val = rp_table[7].resistance;

  

    for (int i = 7; i >=0; i--) {
        if (rp_table[i].resistance >= upper) {
            rp_max_bits = rp_table[i].bits;
            rp_max_val = rp_table[i].resistance;
            break;
        }
    }

    result.reg =
        (high_q_bit << 7) |
        (rp_max_bits << 4) |
        (rp_min_bits);

    result.rp_min_ohms = rp_min_val;
    result.rp_max_ohms = rp_max_val;
    result.high_q = high_q_bit;

    return result;
}


static uint8_t ldc1101_make_tc1(float L_h, float C_sensor)
{
    //uses data sheet equation to figure out the time constant with the least amount of noise
    const float VAMP = 0.6f;
    const float C1 = 6e-12f;//this is hard coded but gives the best noise performance
    const uint8_t C1_bits = 0b11;

    float f = 1.0f / (2.0f * M_PI * sqrtf(L_h * C_sensor));
    float f_min = 0.9f * f;

    float R1_required =
        sqrtf(2.0f) /
        (M_PI * VAMP * f_min * C1);

    float code_f = (417000.0f - R1_required) / 12770.0f;
    int code = (int)roundf(code_f);

    if (code < 0) code = 0;
    if (code > 31) code = 31;

    return (C1_bits << 6) | code;
}

static uint8_t ldc1101_make_tc2(float C_sensor, float rp_min_ohms)// takes in the minimum rp value and sensor to calculate the time constant witht the least ammount of noise
{
    const float C2 = 24e-12f;
    const uint8_t C2_bits = 0b11;

    float R2_required =
        (2.0f * rp_min_ohms * C_sensor) / C2;

    float code_f = (835000.0f - R2_required) / 12770.0f;
    int code = (int)roundf(code_f);

    if (code < 0) code = 0;
    if (code > 63) code = 63;

    return (C2_bits << 6) | code;
}

// The function will figure out the dig_conf register and set minimum ferquency and conversion time
static dig_conf_result_t ldc1101_make_dig_conf(float L_h,float C_sensor)
{
    dig_conf_result_t result;

    float f_sensor = 0.8f *(1.0f / (2.0f * M_PI * sqrtf(L_h * C_sensor)));
                                                        //from data sheet
    float min_freq_calc = 16.0f - (8000000.0f / f_sensor);

    if (min_freq_calc < 0) min_freq_calc = 0;
    if (min_freq_calc > 15) min_freq_calc = 15;

    uint8_t min_freq_bits = (uint8_t)min_freq_calc;

    uint8_t resp_time_bits = 0b111;//hard coded but the most accurate 
    float response_time = 6144.0f;// I chose to have the longest response time as that is the most accurate

    float conversion_time =
        response_time / (3.0f * f_sensor);

    result.reg =
        (min_freq_bits << 4) |
        resp_time_bits;

    result.response_time_periods = response_time;
    result.conversion_time_sec = conversion_time;

    return result;
}

static void spi_init(void)
{
    SPI.begin(LDC_SCK, LDC_MISO, LDC_MOSI, LDC_CS);

    pinMode(LDC_CS, OUTPUT);
    digitalWrite(LDC_CS, HIGH);
}


static uint8_t ldc_read(uint8_t addr)
{
    SPI.beginTransaction(SPISettings(100000, MSBFIRST, SPI_MODE0));
    digitalWrite(LDC_CS, LOW);

    SPI.transfer(addr | 0x80);
    uint8_t val = SPI.transfer(0x00);

    digitalWrite(LDC_CS, HIGH);
    SPI.endTransaction();

    return val;
}


static void ldc_write(uint8_t addr, uint8_t val)
{
    SPI.beginTransaction(SPISettings(100000, MSBFIRST, SPI_MODE0));
    digitalWrite(LDC_CS, LOW);

    SPI.transfer(addr & 0x7F);
    SPI.transfer(val);

    digitalWrite(LDC_CS, HIGH);
    SPI.endTransaction();
}



void ldc1101_configure(float L_h, float C_sensor, float Q)
{
    rp_cfg_global = ldc1101_make_rp_set(L_h, Q, C_sensor);

    uint8_t tc1_reg = ldc1101_make_tc1(L_h, C_sensor);

    uint8_t tc2_reg =
        ldc1101_make_tc2(C_sensor, rp_cfg_global.rp_min_ohms);

    dig_cfg_global = ldc1101_make_dig_conf(L_h, C_sensor);

    // Configure chip
    ldc_write(REG_START_CONFIG, 0x01); // sleep

    ldc_write(REG_RP_SET, rp_cfg_global.reg);
    ldc_write(REG_TC1, tc1_reg);
    ldc_write(REG_TC2, tc2_reg);
    ldc_write(REG_DIG_CONF, dig_cfg_global.reg);

    ldc_write(REG_ALT_CONFIG, 0x00);
    ldc_write(REG_INTB_MODE, 0x00);

    ldc_write(REG_START_CONFIG, 0x00); // active mode
}

ldc1101_measurement_t ldc1101_read(float C_sensor)
{
    ldc1101_measurement_t result;

    // Wait for measurement
    while (ldc_read(REG_STATUS) & (1 << 6))
        delay(1);

    // Read raw data
    uint8_t rp_lsb = ldc_read(REG_RP_DATA_LSB);
    uint8_t rp_msb = ldc_read(REG_RP_DATA_MSB);
    uint16_t rp_raw = ((uint16_t)rp_msb << 8) | rp_lsb;

    uint8_t l_lsb = ldc_read(REG_L_DATA_LSB);
    uint8_t l_msb = ldc_read(REG_L_DATA_MSB);
    uint16_t l_raw = ((uint16_t)l_msb << 8) | l_lsb;

    // Convert Rp
    float x = (float)rp_raw / 65535.0f;

    float Rp_ohms;

    if (rp_cfg_global.high_q)
    {
        Rp_ohms = rp_cfg_global.rp_min_ohms / (1.0f - x);
    }
    else
    {
        Rp_ohms =
            (rp_cfg_global.rp_max_ohms *
             rp_cfg_global.rp_min_ohms) /
            (rp_cfg_global.rp_max_ohms * (1.0f - x) +
             rp_cfg_global.rp_min_ohms * x);
    }

    // Convert L
    float f =
        (F_CLKIN * dig_cfg_global.response_time_periods) /
        (3.0f * l_raw);

    float L_val =
        1.0f /
        ((2.0f * M_PI * f) *
         (2.0f * M_PI * f) *
         C_sensor);

    result.Rp_ohms = Rp_ohms;
    result.L_uH = L_val * 1e6;
    result.timestamp_ms = millis();

    return result;
}

void ldc1101_init(void)
{
    spi_init();
   delay(100);

    Serial.print("CHIP_ID = 0x");
    Serial.println(ldc_read(REG_CHIP_ID), HEX);
}