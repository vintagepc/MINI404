#ifndef STM32_UTIL_H
#define STM32_UTIL_H

#define BYTE_ACCESS_SIZE 1
#define HALFWORD_ACCESS_SIZE 2
#define WORD_ACCESS_SIZE 4

# define STM32_BAD_REG(offset, size)       \
        printf("%s: Bad register 0x%x - size %u\n", __FUNCTION__, (int)offset, size)

#define ENUM_STRING(x) #x

#endif //STM32_UTIL_H
