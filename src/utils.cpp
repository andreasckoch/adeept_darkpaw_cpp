#include "utils.h"

const std::string bit_rep[16] = {
    "0000", "0001", "0010", "0011",
    "0100", "0101", "0110", "0111",
    "1000", "1001", "1010", "1011",
    "1100", "1101", "1110", "1111",
};

std::string get_binary_byte_string(uint8_t byte)
{
    return bit_rep[byte >> 4] + bit_rep[byte & 0x0F];
}
