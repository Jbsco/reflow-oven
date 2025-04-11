// ST7789 using 8-bit Parallel

#define USER_SETUP_ID 303

#define ST7789_2_DRIVER
#define INIT_SEQUENCE_3 // Using this initialisation sequence improves the display image

#define CGRAM_OFFSET
#define TFT_RGB_ORDER TFT_RGB  // Colour order Red-Green-Blue
//#define TFT_RGB_ORDER TFT_BGR // Colour order Blue-Green-Red

#define TFT_INVERSION_ON

#define TFT_WIDTH 135
#define TFT_HEIGHT 240

#define TFT_MOSI 35
#define TFT_SCLK 36
#define TFT_CS  42
#define TFT_DC  40
#define TFT_RST 41

#define TFT_BL 45
#define TFT_BACKLIGHT_ON HIGH
#define TFT_ROTATION 1

#define SPI_FREQUENCY   60000000
// #define SPI_READ_FREQUENCY  20000000
// #define SPI_TOUCH_FREQUENCY 2500000

#define LOAD_GLCD
#define LOAD_FONT2
#define LOAD_FONT4
#define LOAD_FONT6
#define LOAD_FONT7
#define LOAD_FONT8
#define LOAD_GFXFF

#define SMOOTH_FONT
