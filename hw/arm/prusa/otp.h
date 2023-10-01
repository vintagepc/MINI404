#ifndef __OTP_H__
#define __OTP_H__

typedef struct {
    uint16_t product_id;
    uint8_t revision;
    uint32_t supplier_id;
    uint16_t prodution_year;
    uint8_t production_month;
    uint8_t production_day;
    uint16_t daily_serial_number;
} datamatrix_t;

typedef struct {
    uint8_t mac[6];
} MAC_addr;

#pragma pack(push, 1)
typedef struct {
    uint8_t version; // Data structure version (1 bytes)
    uint16_t size; // Data structure size (uint16_t little endian)
    uint8_t bomID; // BOM ID (1 bytes)
    uint32_t timestamp; // UNIX Timestamp from 1970 (uint32_t little endian)
    uint8_t datamatrix[24]; // DataMatrix ID 1 (24 bytes)
    MAC_addr mac_address; // MAC address (6 bytes)
	uint16_t _padding;
} OTP_v4;
#pragma pack(pop)

#endif
