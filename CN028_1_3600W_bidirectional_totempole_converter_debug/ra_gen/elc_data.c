/* generated ELC source file - do not edit */
#include "r_elc_api.h"
const elc_cfg_t g_elc_cfg = { .link[ELC_PERIPHERAL_GPT_A] = ELC_EVENT_NONE, /* No allocation */
.link[ELC_PERIPHERAL_GPT_B] = ELC_EVENT_NONE, /* No allocation */
.link[ELC_PERIPHERAL_GPT_C] = ELC_EVENT_NONE, /* No allocation */
.link[ELC_PERIPHERAL_GPT_D] = ELC_EVENT_NONE, /* No allocation */
.link[ELC_PERIPHERAL_GPT_E] = ELC_EVENT_NONE, /* No allocation */
.link[ELC_PERIPHERAL_GPT_F] = ELC_EVENT_NONE, /* No allocation */
.link[ELC_PERIPHERAL_GPT_G] = ELC_EVENT_NONE, /* No allocation */
.link[ELC_PERIPHERAL_GPT_H] = ELC_EVENT_NONE, /* No allocation */
.link[ELC_PERIPHERAL_DAC0] = ELC_EVENT_NONE, /* No allocation */
.link[ELC_PERIPHERAL_DAC1] = ELC_EVENT_NONE, /* No allocation */
.link[ELC_PERIPHERAL_IOPORTB] = ELC_EVENT_NONE, /* No allocation */
.link[ELC_PERIPHERAL_IOPORTC] = ELC_EVENT_NONE, /* No allocation */
.link[ELC_PERIPHERAL_IOPORTD] = ELC_EVENT_NONE, /* No allocation */
.link[ELC_PERIPHERAL_IOPORTE] = ELC_EVENT_NONE, /* No allocation */
.link[ELC_PERIPHERAL_ADC0] = ELC_EVENT_NONE, /* No allocation */
.link[ELC_PERIPHERAL_ADC0_B] = ELC_EVENT_NONE, /* No allocation */
.link[ELC_PERIPHERAL_ADC0_C] = ELC_EVENT_NONE, /* No allocation */
.link[ELC_PERIPHERAL_ADC1] = ELC_EVENT_NONE, /* No allocation */
.link[ELC_PERIPHERAL_ADC1_B] = ELC_EVENT_NONE, /* No allocation */
.link[ELC_PERIPHERAL_ADC1_C] = ELC_EVENT_NONE, /* No allocation */
.link[ELC_PERIPHERAL_DAC2] = ELC_EVENT_NONE, /* No allocation */
.link[ELC_PERIPHERAL_DAC3] = ELC_EVENT_NONE, /* No allocation */
};

#if BSP_TZ_SECURE_BUILD

        void R_BSP_ElcCfgSecurityInit(void);

        /* Configure ELC Security Attribution. */
        void R_BSP_ElcCfgSecurityInit(void)
        {
 #if (2U == BSP_FEATURE_ELC_VERSION)
            uint32_t elcsarbc = UINT32_MAX;


            /* Write the settings to ELCSARn Registers. */
            R_ELC->ELCSARA = 0xFFFFFFFEU;
            R_ELC->ELCSARB = elcsarbc;
 #else
            uint16_t elcsarbc[2] = {0xFFFFU, 0xFFFFU};

            /* Write the settins to ELCSARn Registers. */
            R_ELC->ELCSARA = 0xFFFEU;
            R_ELC->ELCSARB = elcsarbc[0];
            R_ELC->ELCSARC = elcsarbc[1];
 #endif
        }
#endif
