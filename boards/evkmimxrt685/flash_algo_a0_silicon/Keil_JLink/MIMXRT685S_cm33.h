#include <stdbool.h>
#include <string.h>

/////////////////////////////////////////////////////////////////////////////////

#define     __I     volatile const       /*!< Defines 'read only' permissions */
#define     __O     volatile             /*!< Defines 'write only' permissions */
#define     __IO    volatile             /*!< Defines 'read / write' permissions */

/* ----------------------------------------------------------------------------
   -- SYSCTL0 Peripheral Access Layer
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup SYSCTL0_Peripheral_Access_Layer SYSCTL0 Peripheral Access Layer
 * @{
 */

/** SYSCTL0 - Register Layout Typedef */
typedef struct {
  __IO uint32_t UPDATELCKOUT;                      /**< update clock lock out, offset: 0x0 */
       uint8_t RESERVED_0[8];
  __IO uint32_t DSPSTALL;                          /**< DSP stall register, offset: 0xC */
  __IO uint32_t AHBMATRIXPRIOR;                    /**< AHB matrix priority, offset: 0x10 */
  __IO uint32_t PACKERENABLE;                      /**< , offset: 0x14 */
       uint8_t RESERVED_1[8];
  __IO uint32_t AHBBRIDGEBUFFER[2];                /**< AHB bridge buffer N, array offset: 0x20, array step: 0x4 */
       uint8_t RESERVED_2[8];
  __IO uint32_t M33NMISRCSEL;                      /**< M33 nmi source selection, offset: 0x30 */
  __IO uint32_t SYSTEM_STICK_CALIB;                /**< system stick calibration, offset: 0x34 */
  __IO uint32_t SYSTEM_NSTICK_CALIB;               /**< system nstick calibration, offset: 0x38 */
  __O  uint32_t CONFIG_LCKOUT;                     /**< config lockout, offset: 0x3C */
  __O  uint32_t SRAMCFGENABLE0;                    /**< sram config enable 0, offset: 0x40 */
  __O  uint32_t SRAMCFGENABLE1;                    /**< sram config enable 1, offset: 0x44 */
       uint8_t RESERVED_3[4];
  __IO uint32_t SOFTPARTCFG;                       /**< , offset: 0x4C */
  __IO uint32_t PERICFGENABLE0;                    /**< periperhal config enable 0, offset: 0x50 */
  __IO uint32_t PERICFGENABLE1;                    /**< periperhal config enable 1, offset: 0x54 */
  __IO uint32_t PERICFGENABLE2;                    /**< periperhal config enable 2, offset: 0x58 */
       uint8_t RESERVED_4[4];
  __IO uint32_t PRODUCT_ID;                        /**< product ID, offset: 0x60 */
  __I  uint32_t SILICONREV_ID;                     /**< SILICONREV ID, offset: 0x64 */
  __I  uint32_t JTAG_ID;                           /**< jtag ID, offset: 0x68 */
       uint8_t RESERVED_5[4];
  __IO uint32_t NSGPIO_PSYNC;                      /**< NSGPIO psync, offset: 0x70 */
  __IO uint32_t SGPIO_PSYNC;                       /**< sgpio psync, offset: 0x74 */
       uint8_t RESERVED_6[8];
  __IO uint32_t AUTOCLKGATEOVERRIDE0;              /**< auto clock gating override 0, offset: 0x80 */
  __IO uint32_t AUTOCLKGATEOVERRIDE1;              /**< auto clock gating override 1, offset: 0x84 */
       uint8_t RESERVED_7[24];
  __IO uint32_t CLKGATEOVERRIDE0;                  /**< , offset: 0xA0 */
       uint8_t RESERVED_8[844];
  __IO uint32_t SCRATCH0;                          /**< scratch 0, offset: 0x3F0 */
  __IO uint32_t SCRATCH1;                          /**< scratch 1, offset: 0x3F4 */
       uint8_t RESERVED_9[20];
  __IO uint32_t USBCLKCTRL;                        /**< USB clock control, offset: 0x40C */
  __I  uint32_t USBCLKSTAT;                        /**< USB clock status, offset: 0x410 */
  __IO uint32_t USBPHYPLL0LOCKTIME;                /**< , offset: 0x414 */
       uint8_t RESERVED_10[488];
  __IO uint32_t PDSLEEPCFG0;                       /**< , offset: 0x600 */
  __IO uint32_t PDSLEEPCFG1;                       /**< , offset: 0x604 */
  __IO uint32_t PDSLEEPCFG2;                       /**< , offset: 0x608 */
  __IO uint32_t PDSLEEPCFG3;                       /**< , offset: 0x60C */
  __IO uint32_t PDRUNCFG0;                         /**< , offset: 0x610 */
  __IO uint32_t PDRUNCFG1;                         /**< , offset: 0x614 */
  __IO uint32_t PDRUNCFG2;                         /**< , offset: 0x618 */
  __IO uint32_t PDRUNCFG3;                         /**< , offset: 0x61C */
  __O  uint32_t PDRUNCFG0_SET;                     /**< , offset: 0x620 */
  __O  uint32_t PDRUNCFG1_SET;                     /**< , offset: 0x624 */
  __O  uint32_t PDRUNCFG2_SET;                     /**< , offset: 0x628 */
  __O  uint32_t PDRUNCFG3_SET;                     /**< , offset: 0x62C */
  __O  uint32_t PDRUNCFG0_CLR;                     /**< , offset: 0x630 */
  __O  uint32_t PDRUNCFG1_CLR;                     /**< , offset: 0x634 */
  __O  uint32_t PDRUNCFG2_CLR;                     /**< , offset: 0x638 */
  __O  uint32_t PDRUNCFG3_CLR;                     /**< , offset: 0x63C */
       uint8_t RESERVED_11[64];
  __IO uint32_t STARTEN0;                          /**< , offset: 0x680 */
  __IO uint32_t STARTEN1;                          /**< , offset: 0x684 */
       uint8_t RESERVED_12[24];
  __O  uint32_t STARTEN0_SET;                      /**< , offset: 0x6A0 */
  __O  uint32_t STARTEN1_SET;                      /**< , offset: 0x6A4 */
       uint8_t RESERVED_13[24];
  __O  uint32_t STARTEN0_CLR;                      /**< , offset: 0x6C0 */
  __O  uint32_t STARTEN1_CLR;                      /**< , offset: 0x6C4 */
       uint8_t RESERVED_14[184];
  __IO uint32_t HWWAKE;                            /**< , offset: 0x780 */
       uint8_t RESERVED_15[1672];
  __IO uint32_t TEMPSENSORCTL;                     /**< tempsensor ctrl, offset: 0xE0C */
  __IO uint32_t DSPMEMSPEEDCTL;                    /**< , offset: 0xE10 */
       uint8_t RESERVED_16[232];
  __IO uint32_t SDIOPADCTL;                        /**< sdio pad ctrl, offset: 0xEFC */
  __IO uint32_t DICEHWREG[8];                      /**< DICE General Purpose 32-Bit Data Register, array offset: 0xF00, array step: 0x4 */
       uint8_t RESERVED_17[16];
  __O  uint32_t S0AES_UV;                          /**< , offset: 0xF30 */
  __O  uint32_t S0AES_UV_WLOCK;                    /**< , offset: 0xF34 */
  __O  uint32_t S1UDS_UV;                          /**< , offset: 0xF38 */
  __O  uint32_t S1UDS_UV_WLOCK;                    /**< , offset: 0xF3C */
  __I  uint32_t UDS_SEED;                          /**< , offset: 0xF40 */
  __O  uint32_t UDS_SEED_RLOCK;                    /**< , offset: 0xF44 */
       uint8_t RESERVED_18[8];
  __IO uint32_t UUID[4];                           /**< UUIDn 32-Bit Data Register, array offset: 0xF50, array step: 0x4 */
  __IO uint32_t UUID_WLOCK;                        /**< UUID write lock register, offset: 0xF60 */
       uint8_t RESERVED_19[28];
  __IO uint32_t AESKEY_SRCSEL;                     /**< , offset: 0xF80 */
  __IO uint32_t OTFADKEY_SRCSEL;                   /**< , offset: 0xF84 */
       uint8_t RESERVED_20[24];
  __IO uint32_t DBG_LOCKEN;                        /**< Debug Write Lock registers, offset: 0xFA0 */
  __IO uint32_t DBG_FEATURES;                      /**< , offset: 0xFA4 */
  __IO uint32_t DBG_FEATURES_DP;                   /**< , offset: 0xFA8 */
  __IO uint32_t HWUNLOCK_DISABLE;                  /**< , offset: 0xFAC */
  __IO uint32_t CS_PROTTEST;                       /**< , offset: 0xFB0 */
  __IO uint32_t CS_PROTCPU0;                       /**< , offset: 0xFB4 */
  __IO uint32_t CS_PROTCPU1;                       /**< , offset: 0xFB8 */
       uint8_t RESERVED_21[4];
  __IO uint32_t DBG_AUTH_SCRATCH;                  /**< , offset: 0xFC0 */
       uint8_t RESERVED_22[12];
  __IO uint32_t KEY_BLOCK;                         /**< , offset: 0xFD0 */
} SYSCTL0_Type;

#define SYSCTL0_PERICFGENABLE0_OSPI_OTFAD_EN_MASK (0x10000U)

#define SYSCTL0_PDRUNCFG1_OSPI_SRAM_APD_MASK     (0x4U)
#define SYSCTL0_PDRUNCFG1_OSPI_SRAM_PPD_MASK     (0x8U)

/** Peripheral SYSCTL0 base address */
#define SYSCTL0_BASE                             (0x50002000u)
/** Peripheral SYSCTL0 base pointer */
#define SYSCTL0                                  ((SYSCTL0_Type *)SYSCTL0_BASE)


/* ----------------------------------------------------------------------------
   -- CLKCTL0 Peripheral Access Layer
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup CLKCTL0_Peripheral_Access_Layer CLKCTL0 Peripheral Access Layer
 * @{
 */

/** CLKCTL0 - Register Layout Typedef */
typedef struct {
  __IO uint32_t UPDATELCKOUT;                      /**< update clock lock out, offset: 0x0 */
       uint8_t RESERVED_0[12];
  __IO uint32_t PSCCTL0;                           /**< clock control register 0, offset: 0x10 */
  __IO uint32_t PSCCTL1;                           /**< clock control register 1, offset: 0x14 */
  __IO uint32_t PSCCTL2;                           /**< clock control register 2, offset: 0x18 */
       uint8_t RESERVED_1[36];
  __O  uint32_t PSCCTL0_SET;                       /**< clock set register 0, offset: 0x40 */
  __O  uint32_t PSCCTL1_SET;                       /**< clock set register 1, offset: 0x44 */
  __O  uint32_t PSCCTL2_SET;                       /**< clock set register 2, offset: 0x48 */
       uint8_t RESERVED_2[36];
  __O  uint32_t PSCCTL0_CLR;                       /**< clock clear register 0, offset: 0x70 */
  __O  uint32_t PSCCTL1_CLR;                       /**< clock clear register 1, offset: 0x74 */
  __O  uint32_t PSCCTL2_CLR;                       /**< clock clear register 2, offset: 0x78 */
       uint8_t RESERVED_3[132];
  __IO uint32_t FFROCTL0;                          /**< FFRO control 0, offset: 0x100 */
  __IO uint32_t FFROCTL1;                          /**< FFRO control 1, offset: 0x104 */
       uint8_t RESERVED_4[40];
  __IO uint32_t SFROCTL0;                          /**< SFRO control 0, offset: 0x130 */
       uint32_t SFROCTL1;                          /**< SFRO control 1, offset: 0x134 */
       uint8_t RESERVED_5[40];
  __IO uint32_t SYSOSCCTL0;                        /**< system oscillator control 0, offset: 0x160 */
       uint32_t SYSOSCCTL1;                        /**< system oscillator control 1, offset: 0x164 */
  __IO uint32_t SYSOSCBYPASS;                      /**< system oscillator bypass, offset: 0x168 */
       uint8_t RESERVED_6[36];
  __IO uint32_t LPOSCCTL0;                         /**< low power oscillator control 0, offset: 0x190 */
       uint32_t LPOSCCTL1;                         /**< low power oscillator control 1, offset: 0x194 */
       uint8_t RESERVED_7[40];
  __IO uint32_t OSC32KHZCTL0;                      /**< 32k oscillator control0, offset: 0x1C0 */
       uint32_t OSC32KHZCTL1;                      /**< 32k oscillator control1, offset: 0x1C4 */
       uint8_t RESERVED_8[56];
  __IO uint32_t SYSPLL0CLKSEL;                     /**< system pll0 clock selection, offset: 0x200 */
  __IO uint32_t SYSPLL0CTL0;                       /**< system pll0 control0, offset: 0x204 */
  __IO uint32_t SYSPLL0SS;                         /**< syspll0 SS, offset: 0x208 */
  __IO uint32_t SYSPLL0LOCKTIME;                   /**< system pll0 lock time, offset: 0x20C */
  __IO uint32_t SYSPLL0NUM;                        /**< system pll0 number, offset: 0x210 */
  __IO uint32_t SYSPLL0DENOM;                      /**< system pll0 denom, offset: 0x214 */
  __IO uint32_t SYSPLL0PFD;                        /**< sys pll0 PFD, offset: 0x218 */
       uint8_t RESERVED_9[484];
  __IO uint32_t SYSCPUAHBCLKDIV;                   /**< system cpu AHB clock divider, offset: 0x400 */
       uint8_t RESERVED_10[44];
  __IO uint32_t MAINCLKSELA;                       /**< main clock selection A, offset: 0x430 */
  __IO uint32_t MAINCLKSELB;                       /**< main clock selection B, offset: 0x434 */
  __IO uint32_t INVERTMAINCLK;                     /**< inverter main clock, offset: 0x438 */
       uint8_t RESERVED_11[196];
  __IO uint32_t PFCDIV[8];                         /**< PFC divider register N, array offset: 0x500, array step: 0x4 */
       uint8_t RESERVED_12[256];
  __IO uint32_t OSPIFFCLKSEL;                      /**< OSPI FCLK selection, offset: 0x620 */
  __IO uint32_t OSPIFCLKDIV;                       /**< OSPI FCLK divider, offset: 0x624 */
       uint8_t RESERVED_13[24];
  __IO uint32_t SCTFCLKSEL;                        /**< SCT FCLK selection, offset: 0x640 */
  __IO uint32_t SCTFCLKDIV;                        /**< SCT fclk divider, offset: 0x644 */
       uint8_t RESERVED_14[24];
  __IO uint32_t USBHSFCLKSEL;                      /**< USBHS Fclk selection, offset: 0x660 */
  __IO uint32_t USBHSFCLKDIV;                      /**< USBHS Fclk divider, offset: 0x664 */
       uint8_t RESERVED_15[24];
  __IO uint32_t SDIO0FCLKSEL;                      /**< SDIO0 FCLK selection, offset: 0x680 */
  __IO uint32_t SDIO0FCLKDIV;                      /**< SDIO0 FCLK divider, offset: 0x684 */
       uint8_t RESERVED_16[8];
  __IO uint32_t SDIO1FCLKSEL;                      /**< SDIO1 FCLK selection, offset: 0x690 */
  __IO uint32_t SDIO1FCLKDIV;                      /**< SDIO1 FCLK divider, offset: 0x694 */
       uint8_t RESERVED_17[20];
  __IO uint32_t CAPTPLLCLKDIV;                     /**< CAPT pll clock divider, offset: 0x6AC */
  __IO uint32_t CAPTFCLKSEL;                       /**< CAPT fclk selection, offset: 0x6B0 */
       uint8_t RESERVED_18[28];
  __IO uint32_t ADC0FCLKSEL0;                      /**< ADC0 fclk selection 0, offset: 0x6D0 */
  __IO uint32_t ADC0FCLKSEL1;                      /**< ADC0 fclk selection 1, offset: 0x6D4 */
  __IO uint32_t ADC0FCLKDIV;                       /**< ADC0 fclk divider, offset: 0x6D8 */
       uint8_t RESERVED_19[36];
  __IO uint32_t UTICKFCLKSEL;                      /**< UTICK fclk selection, offset: 0x700 */
       uint8_t RESERVED_20[28];
  __IO uint32_t WDT0FCLKSEL;                       /**< wdt clock selection, offset: 0x720 */
       uint8_t RESERVED_21[12];
  __IO uint32_t WAKECLK32KHZSEL;                   /**< 32k wake clock selection, offset: 0x730 */
  __IO uint32_t WAKECLK32KHZDIV;                   /**< 32k wake clock divider, offset: 0x734 */
  __IO uint32_t F32KHZWAKEDIVTSTCLKSEL;            /**< 32k wake clock divider st clock selection, offset: 0x738 */
       uint8_t RESERVED_22[36];
  __IO uint32_t SYSTICKFCLKSEL;                    /**< system tick fclk selection, offset: 0x760 */
  __IO uint32_t SYSTICKFCLKDIV;                    /**< system tick fclk divider, offset: 0x764 */
} CLKCTL0_Type;

/** Peripheral CLKCTL0 base address */
#define CLKCTL0_BASE                             (0x50001000u)
/** Peripheral CLKCTL0 base pointer */
#define CLKCTL0                                  ((CLKCTL0_Type *)CLKCTL0_BASE)

/////////////////////////////////////////////////////////////////////////////////