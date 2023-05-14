#ifndef CHECKSUM_H
#define CHECKSUM_H

#include <stdint.h>
#include <stdbool.h>

uint16_t GetCrc16Checksumm(uint8_t *pcBlock, uint16_t len);
bool IsCrc16ChecksummCorrect(uint8_t *pcBlock, uint16_t len);
void AddCrc16Checksumm(uint8_t *pcBlock, uint16_t len);

bool IsChecksumm16bCorrect(uint8_t *msg, uint16_t length);
void AddChecksumm16b(uint8_t *msg, uint16_t length);

bool IsChecksumm8bCorrect(uint8_t *msg, uint16_t length);
void AddChecksumm8b(uint8_t *msg, uint16_t length);

bool IsChecksumm8bCorrectVma(uint8_t *msg, uint16_t length);
void AddChecksumm8bVma(uint8_t *msg, uint16_t length);

void AddChecksum16bS(uint8_t *msg, uint16_t length);
bool IsChecksum16bSCorrect(uint8_t *msg, uint16_t length);

uint8_t ComputeChecksum8b(uint8_t *msg, uint16_t length);

void CompChecksum(uint8_t *upbyte, uint8_t *lowbyte, uint8_t *msg, uint8_t size);

int16_t MergeBytes(uint8_t *pos);
uint16_t MergeUBytes(uint8_t most, uint8_t least);
float FloatFromUint8(uint8_t *buff, uint8_t high_byte_pos);
float FloatFromUint8Reverse(uint8_t *buff, uint8_t high_byte_pos);
void Uint8FromFloat(float input, uint8_t *outArray);
void nullIntArray(uint8_t *array, uint8_t size);
bool PickBit(uint8_t input, uint8_t bit);
void SetBit(uint8_t *byte, uint8_t bit, bool state);
void writeBit(uint8_t *byte, uint8_t value, uint8_t biteNumb);

#endif
