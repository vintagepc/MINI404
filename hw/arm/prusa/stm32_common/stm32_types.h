#ifndef STM32_TYPES_H
#define STM32_TYPES_H

// Helpers for enforcing type name consistency.
#define _STM_VAR_PART(var,part) "stm32"#var"-"#part
#define _STM_COM_PART(x) _STM_VAR_PART(com,x)
#define _STM_F2xx_PART(x) _STM_VAR_PART(f2xx,x)
#define _STM_F4xx_PART(x) _STM_VAR_PART(f4xx,x)

// Chip types:
#define TYPE_STM32F407xE_SOC "stm32f407xE-soc"
#define TYPE_STM32F407xG_SOC "stm32f407xG-soc"


// Conveniences for naming consistency
#define STM32F2XX_STRUCT_NAME(part) _JOIN3R(STM32F2xx,part,State)
#define STM32F4XX_STRUCT_NAME(part) _JOIN3R(STM32F4xx,part,State)
#define COM_STRUCT_NAME(part) _JOIN3R(STM32COM,part,State)
#define COM_CLASS_NAME(part) _JOIN3R(STM32COM,part,Class)

// These are the internal/abstract types.
#define TYPE_STM32COM_RCC _STM_COM_PART(rcc)
#define TYPE_STM32COM_RCC_IF _STM_COM_PART(rcc)
#define TYPE_STM32_PERIPHERAL _STM_COM_PART(peripheral)
#define TYPE_STM32_SOC _STM_COM_PART(soc)
#define TYPE_STM32F4XX_BASE _STM_F4xx_PART(base)

// These are the specific variants known/defined.
// Consider adding explicit entries for new variants you discover
// so it's clearer what might be impacted by changes.
#define TYPE_STM32F2xx_RCC _STM_F2xx_PART(rcc)
#define TYPE_STM32F4xx_RCC _STM_F4xx_PART(rcc)

#define TYPE_STM32F4xx_FINT _STM_F4xx_PART(fint)

#define TYPE_STM32F2xx_DMA _STM_F2xx_PART(dma)
#define TYPE_STM32F4xx_DMA _STM_F4xx_PART(dma)

#endif
