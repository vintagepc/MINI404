#ifndef STM32_TYPES_H
#define STM32_TYPES_H

// Helpers for enforcing type name consistency.
#define _STM_VAR_PART(var,part) "stm32"#var"-"#part
#define _STM_COM_PART(x) _STM_VAR_PART(com,x)
#define _STM_F030_PART(x) _STM_VAR_PART(f030,x)
#define _STM_G070_PART(x) _STM_VAR_PART(g070,x)
#define _STM_F2xx_PART(x) _STM_VAR_PART(f2xx,x)
#define _STM_F4xx_PART(x) _STM_VAR_PART(f4xx,x)

#define _STM32_SOC(var) (var "-soc")

#define TYPE_STM32_MACHINE "stm32-machine"

// Base names:
#define TYPE_STM32F030xx "stm32f030xx"
#define TYPE_STM32F030x4 "stm32f030x4"
#define TYPE_STM32F030x6 "stm32f030x6"
#define TYPE_STM32F030x8 "stm32f030x8"
#define TYPE_STM32F030xC "stm32f030xC"

#define TYPE_STM32G070xx "stm32g070xx"
#define TYPE_STM32G070xB "stm32g070xB"

#define TYPE_STM32F4xx "stm32f4xx"
#define TYPE_STM32F407xE "stm32f407xE"
#define TYPE_STM32F407xG "stm32f407xG"

#define TYPE_STM32F427xE "stm32f427xE"
#define TYPE_STM32F427xG "stm32f427xG"
#define TYPE_STM32F427xI "stm32f427xI"

// Chip types:

#define TYPE_STM32F030x4_SOC _STM32_SOC(TYPE_STM32F030x4)
#define TYPE_STM32F030x6_SOC _STM32_SOC(TYPE_STM32F030x6)
#define TYPE_STM32F030x8_SOC _STM32_SOC(TYPE_STM32F030x8)
#define TYPE_STM32F030xC_SOC _STM32_SOC(TYPE_STM32F030xC)

#define TYPE_STM32G070xB_SOC _STM32_SOC(TYPE_STM32G070xB)

#define TYPE_STM32F407xE_SOC _STM32_SOC(TYPE_STM32F407xE)
#define TYPE_STM32F407xG_SOC _STM32_SOC(TYPE_STM32F407xG)

#define TYPE_STM32F427xE_SOC _STM32_SOC(TYPE_STM32F427xE)
#define TYPE_STM32F427xG_SOC _STM32_SOC(TYPE_STM32F427xG)
#define TYPE_STM32F427xI_SOC _STM32_SOC(TYPE_STM32F427xI)


// Conveniences for naming consistency
#define STM32F030_STRUCT_NAME(part) _JOIN3R(STM32F030,part,State)
#define STM32G070_STRUCT_NAME(part) _JOIN3R(STM32G070,part,State)
#define STM32F2XX_STRUCT_NAME(part) _JOIN3R(STM32F2xx,part,State)
#define STM32F4XX_STRUCT_NAME(part) _JOIN3R(STM32F4xx,part,State)
#define COM_STRUCT_NAME(part) _JOIN3R(STM32COM,part,State)
#define COM_CLASS_NAME(part) _JOIN3R(STM32COM,part,Class)

// These are the internal/abstract types.
#define TYPE_STM32COM_ADC _STM_COM_PART(adc)
#define TYPE_STM32COM_ADCC _STM_COM_PART(adcc)
#define TYPE_STM32COM_CRC _STM_COM_PART(crc)
#define TYPE_STM32COM_DBG _STM_COM_PART(dbg)
#define TYPE_STM32COM_DMA _STM_COM_PART(dma)
#define TYPE_STM32COM_DMAMUX _STM_COM_PART(dmamux)
#define TYPE_STM32COM_GPIO _STM_COM_PART(gpio)
#define TYPE_STM32COM_IWDG _STM_COM_PART(iwdg)
#define TYPE_STM32COM_SPI _STM_COM_PART(spi)
#define TYPE_STM32COM_SYSCFG _STM_COM_PART(syscfg)
#define TYPE_STM32COM_USART _STM_COM_PART(usart)
#define TYPE_STM32COM_RCC _STM_COM_PART(rcc)
#define TYPE_STM32COM_RCC_IF _STM_COM_PART(rcc)
#define TYPE_STM32_PERIPHERAL _STM_COM_PART(peripheral)
#define TYPE_STM32_SOC _STM_COM_PART(soc)
#define TYPE_STM32F030XX_BASE _STM_F030_PART(base)
#define TYPE_STM32G070XX_BASE _STM_G070_PART(base)
#define TYPE_STM32F4XX_BASE _STM_F4xx_PART(base)
#define TYPE_STM32COM_OTP _STM_COM_PART(otp)

// These are the specific variants known/defined.
// Consider adding explicit entries for new variants you discover
// so it's clearer what might be impacted by changes.
#define TYPE_STM32F030_RCC _STM_F030_PART(rcc)
#define TYPE_STM32G070_RCC _STM_G070_PART(rcc)
#define TYPE_STM32F2xx_RCC _STM_F2xx_PART(rcc)
#define TYPE_STM32F4xx_RCC _STM_F4xx_PART(rcc)
#define TYPE_STM32F407_RCC _STM_VAR_PART(f407,rcc)
#define TYPE_STM32F427_RCC _STM_VAR_PART(f427,rcc)

#define TYPE_STM32F4xx_FINT _STM_F4xx_PART(fint)
#define TYPE_STM32F40x_F41x_FINT _STM_VAR_PART(f40x_f41x,fint)
#define TYPE_STM32F42x_F43x_FINT _STM_VAR_PART(f42x_f43x,fint)
#define TYPE_STM32G070_FINT _STM_G070_PART(fint)

#define TYPE_STM32F030_ADC _STM_F030_PART(adc)
#define TYPE_STM32G070_ADC _STM_G070_PART(adc)
#define TYPE_STM32F4xx_ADC _STM_F4xx_PART(adc)

#define TYPE_STM32F030_ADCC _STM_F030_PART(adcc)
#define TYPE_STM32G070_ADCC _STM_G070_PART(adcc)

#define TYPE_STM32F030_CRC _STM_F030_PART(crc)
#define TYPE_STM32G070_CRC _STM_G070_PART(crc)
#define TYPE_STM32F2xx_CRC _STM_F2xx_PART(crc)
#define TYPE_STM32F4xx_CRC _STM_F4xx_PART(crc)

#define TYPE_STM32G070_DBG _STM_G070_PART(dbg)
#define TYPE_STM32F40x_F41x_DBG _STM_VAR_PART(f40x_f41x,dbg)
#define TYPE_STM32F42x_F43x_DBG _STM_VAR_PART(f42x_f43x,dbg)

#define TYPE_STM32F4xx_DWT _STM_F4xx_PART(dwt)

#define TYPE_STM32G070_EXTI _STM_G070_PART(exti)

#define TYPE_STM32F030_IWDG _STM_F030_PART(iwdg)
#define TYPE_STM32G070_IWDG _STM_G070_PART(iwdg)
#define TYPE_STM32F4xx_IWDG _STM_F4xx_PART(iwdg)

#define TYPE_STM32F030_DMA _STM_F030_PART(dma)
#define TYPE_STM32G070_DMA _STM_G070_PART(dma)
#define TYPE_STM32F2xx_DMA _STM_F2xx_PART(dma)
#define TYPE_STM32F4xx_DMA _STM_F4xx_PART(dma)

#define TYPE_STM32G070_DMAMUX _STM_G070_PART(dmamux)

#define TYPE_STM32F030_GPIO _STM_F030_PART(gpio)
#define TYPE_STM32G070_GPIO _STM_G070_PART(gpio)
#define TYPE_STM32F2xx_GPIO _STM_F2xx_PART(gpio)
#define TYPE_STM32F4xx_GPIO _STM_F4xx_PART(gpio)

#define TYPE_STM32F4xx_SYSCFG _STM_COM_PART(f4xx-syscfg)
#define TYPE_STM32F030_SYSCFG _STM_F030_PART(syscfg)
#define TYPE_STM32G070_SYSCFG _STM_G070_PART(fsyscfg)
#define TYPE_STM32F40x_F41x_SYSCFG _STM_F4xx_PART(f40x_f41x_syscfg)
#define TYPE_STM32F42x_F43x_SYSCFG _STM_F4xx_PART(f42x_f43x_syscfg)

#define TYPE_STM32F4xx_ITM _STM_F4xx_PART(itm)

#define TYPE_STM32F030_SPI _STM_F030_PART(spi)
#define TYPE_STM32G070_SPI _STM_G070_PART(spi)
// Tweaked because it conflicts with the public one
#define TYPE_STM32F2xx_SPI _STM_F2xx_PART(spi-v)
#define TYPE_STM32F4xx_SPI _STM_F4xx_PART(spi)

#define TYPE_STM32F030_USART _STM_F030_PART(usart)
#define TYPE_STM32G070_USART _STM_G070_PART(usart)

#define TYPE_STM32G070_OTP _STM_G070_PART(otp)
#define TYPE_STM32F4xx_OTP _STM_F4xx_PART(otp)

#endif
