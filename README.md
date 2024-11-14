# GPIO-pin-control

raspi-gpio get 22
raspi-gpio set 22 op - For configuring it as output
raspi-gpio set 22 pn - for configuring pull = None

// GPIO Base Physical Address for Raspberry Pi 4
#define GPIO_BASE               0xFE200000

// Function Select Registers (GPFSELn)
// Each GPIO pin has 3 bits in GPFSEL, values: 000(input), 001(output), 100(alt0), 101(alt1), 110(alt2), 111(alt3), 011(alt4), 010(alt5)
#define GPFSEL0                (GPIO_BASE + 0x00)  // GPIO 0-9
#define GPFSEL1                (GPIO_BASE + 0x04)  // GPIO 10-19
#define GPFSEL2                (GPIO_BASE + 0x08)  // GPIO 20-29
#define GPFSEL3                (GPIO_BASE + 0x0C)  // GPIO 30-39
#define GPFSEL4                (GPIO_BASE + 0x10)  // GPIO 40-49
#define GPFSEL5                (GPIO_BASE + 0x14)  // GPIO 50-53

// Pin Output Set Registers (GPSETn)
// Writing 1 to a bit will set the corresponding GPIO pin high
#define GPSET0                 (GPIO_BASE + 0x1C)  // GPIO 0-31
#define GPSET1                 (GPIO_BASE + 0x20)  // GPIO 32-53

// Pin Output Clear Registers (GPCLRn)
// Writing 1 to a bit will set the corresponding GPIO pin low
#define GPCLR0                 (GPIO_BASE + 0x28)  // GPIO 0-31
#define GPCLR1                 (GPIO_BASE + 0x2C)  // GPIO 32-53

// Pin Level Registers (GPLEVn)
// Returns the actual value on the pin (0 or 1)
#define GPLEV0                 (GPIO_BASE + 0x34)  // GPIO 0-31
#define GPLEV1                 (GPIO_BASE + 0x38)  // GPIO 32-53

// Event Detect Status Registers (GPEDSn)
// Pin event detection status
#define GPEDS0                 (GPIO_BASE + 0x40)  // GPIO 0-31
#define GPEDS1                 (GPIO_BASE + 0x44)  // GPIO 32-53

// Rising Edge Detect Enable Registers (GPRENn)
#define GPREN0                 (GPIO_BASE + 0x4C)  // GPIO 0-31
#define GPREN1                 (GPIO_BASE + 0x50)  // GPIO 32-53

// Falling Edge Detect Enable Registers (GPFENn)
#define GPFEN0                 (GPIO_BASE + 0x58)  // GPIO 0-31
#define GPFEN1                 (GPIO_BASE + 0x5C)  // GPIO 32-53

// High Detect Enable Registers (GPHENn)
#define GPHEN0                 (GPIO_BASE + 0x64)  // GPIO 0-31
#define GPHEN1                 (GPIO_BASE + 0x68)  // GPIO 32-53

// Low Detect Enable Registers (GPLENn)
#define GPLEN0                 (GPIO_BASE + 0x70)  // GPIO 0-31
#define GPLEN1                 (GPIO_BASE + 0x74)  // GPIO 32-53

// Asynchronous Rising Edge Detect Registers (GPARENn)
#define GPAREN0                (GPIO_BASE + 0x7C)  // GPIO 0-31
#define GPAREN1                (GPIO_BASE + 0x80)  // GPIO 32-53

// Asynchronous Falling Edge Detect Registers (GPAFENn)
#define GPAFEN0                (GPIO_BASE + 0x88)  // GPIO 0-31
#define GPAFEN1                (GPIO_BASE + 0x8C)  // GPIO 32-53

// Pull-up/down Register
#define GPPUD                  (GPIO_BASE + 0x94)  // Controls pull-up/down for all pins

// Pull-up/down Clock Registers
#define GPPUDCLK0              (GPIO_BASE + 0x98)  // GPIO 0-31
#define GPPUDCLK1              (GPIO_BASE + 0x9C)  // GPIO 32-53

// For BCM2711 (Raspberry Pi 4), additional pull-up/down registers
// These replace GPPUD and GPPUDCLKn on newer models
#define GPIO_PUP_PDN_CNTRL_REG0 (GPIO_BASE + 0xE4)  // GPIO 0-15
#define GPIO_PUP_PDN_CNTRL_REG1 (GPIO_BASE + 0xE8)  // GPIO 16-31
#define GPIO_PUP_PDN_CNTRL_REG2 (GPIO_BASE + 0xEC)  // GPIO 32-47
#define GPIO_PUP_PDN_CNTRL_REG3 (GPIO_BASE + 0xF0)  // GPIO 48-53

// Pull up/down states for BCM2711
#define GPIO_PULL_NONE        0b00
#define GPIO_PULL_UP          0b01
#define GPIO_PULL_DOWN        0b10
#define GPIO_PULL_RESERVED    0b11

// GPIO Function Select values
#define GPIO_FSEL_INPUT       0b000
#define GPIO_FSEL_OUTPUT      0b001
#define GPIO_FSEL_ALT0        0b100
#define GPIO_FSEL_ALT1        0b101
#define GPIO_FSEL_ALT2        0b110
#define GPIO_FSEL_ALT3        0b111
#define GPIO_FSEL_ALT4        0b011
#define GPIO_FSEL_ALT5        0b010
