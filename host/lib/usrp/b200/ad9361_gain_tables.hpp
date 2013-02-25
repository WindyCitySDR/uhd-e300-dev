#ifndef INCLUDED_AD9361_GAIN_TABLES_HPP
#define INCLUDED_AD9361_GAIN_TABLES_HPP

#include <boost/cstdint.hpp>

boost::uint8_t gain_table_sub_1300mhz[77][5] = { {0,0x00,0x00,0x20,1},
    {1,0x00,0x00,0x00,0}, {2,0x00,0x00,0x00,0}, {3,0x00,0x01,0x00,0},
    {4,0x00,0x02,0x00,0}, {5,0x00,0x03,0x00,0}, {6,0x00,0x04,0x00,0},
    {7,0x00,0x05,0x00,0}, {8,0x01,0x03,0x20,1}, {9,0x01,0x04,0x00,0},
    {10,0x01,0x05,0x00,0}, {11,0x01,0x06,0x00,0}, {12,0x01,0x07,0x00,0},
    {13,0x01,0x08,0x00,0}, {14,0x01,0x09,0x00,0}, {15,0x01,0x0A,0x00,0},
    {16,0x01,0x0B,0x00,0}, {17,0x01,0x0C,0x00,0}, {18,0x01,0x0D,0x00,0},
    {19,0x01,0x0E,0x00,0}, {20,0x02,0x09,0x20,1}, {21,0x02,0x0A,0x00,0},
    {22,0x02,0x0B,0x00,0}, {23,0x02,0x0C,0x00,0}, {24,0x02,0x0D,0x00,0},
    {25,0x02,0x0E,0x00,0}, {26,0x02,0x0F,0x00,0}, {27,0x02,0x10,0x00,0},
    {28,0x02,0x2B,0x20,1}, {29,0x02,0x2C,0x00,0}, {30,0x04,0x27,0x20,1},
    {31,0x04,0x28,0x00,0}, {32,0x04,0x29,0x00,0}, {33,0x04,0x2A,0x00,0},
    {34,0x04,0x2B,0x00,1}, {35,0x24,0x21,0x20,0}, {36,0x24,0x22,0x00,1},
    {37,0x44,0x20,0x20,0}, {38,0x44,0x21,0x00,0}, {39,0x44,0x22,0x00,0},
    {40,0x44,0x23,0x00,0}, {41,0x44,0x24,0x00,0}, {42,0x44,0x25,0x00,0},
    {43,0x44,0x26,0x00,0}, {44,0x44,0x27,0x00,0}, {45,0x44,0x28,0x00,0},
    {46,0x44,0x29,0x00,0}, {47,0x44,0x2A,0x00,0}, {48,0x44,0x2B,0x00,0},
    {49,0x44,0x2C,0x00,0}, {50,0x44,0x2D,0x00,0}, {51,0x44,0x2E,0x00,0},
    {52,0x44,0x2F,0x00,0}, {53,0x44,0x30,0x00,0}, {54,0x44,0x31,0x00,0},
    {55,0x64,0x2E,0x20,1}, {56,0x64,0x2F,0x00,0}, {57,0x64,0x30,0x00,0},
    {58,0x64,0x31,0x00,0}, {59,0x64,0x32,0x00,0}, {60,0x64,0x33,0x00,0},
    {61,0x64,0x34,0x00,0}, {62,0x64,0x35,0x00,0}, {63,0x64,0x36,0x00,0},
    {64,0x64,0x37,0x00,0}, {65,0x64,0x38,0x00,0}, {66,0x65,0x38,0x20,1},
    {67,0x66,0x38,0x20,1}, {68,0x67,0x38,0x20,1}, {69,0x68,0x38,0x20,1},
    {70,0x69,0x38,0x20,1}, {71,0x6A,0x38,0x20,1}, {72,0x6B,0x38,0x20,1},
    {73,0x6C,0x38,0x20,1}, {74,0x6D,0x38,0x20,1}, {75,0x6E,0x38,0x20,1},
    {76,0x6F,0x38,0x20,1}};


boost::uint8_t gain_table_1300mhz_to_4000mhz[77][5] = {   {0,0x00,0x00,0x20,1},
    {1,0x00,0x00,0x00,0}, {2,0x00,0x00,0x00,0}, {3,0x00,0x01,0x00,0},
    {4,0x00,0x02,0x00,0}, {5,0x00,0x03,0x00,0}, {6,0x00,0x04,0x00,0},
    {7,0x00,0x05,0x00,0}, {8,0x01,0x03,0x20,1}, {9,0x01,0x04,0x00,0},
    {10,0x01,0x05,0x00,0}, {11,0x01,0x06,0x00,0}, {12,0x01,0x07,0x00,0},
    {13,0x01,0x08,0x00,0}, {14,0x01,0x09,0x00,0}, {15,0x01,0x0A,0x00,0},
    {16,0x01,0x0B,0x00,0}, {17,0x01,0x0C,0x00,0}, {18,0x01,0x0D,0x00,0},
    {19,0x01,0x0E,0x00,0}, {20,0x02,0x09,0x20,1}, {21,0x02,0x0A,0x00,0},
    {22,0x02,0x0B,0x00,0}, {23,0x02,0x0C,0x00,0}, {24,0x02,0x0D,0x00,0},
    {25,0x02,0x0E,0x00,0}, {26,0x02,0x0F,0x00,0}, {27,0x02,0x10,0x00,0},
    {28,0x02,0x2B,0x20,1}, {29,0x02,0x2C,0x00,0}, {30,0x04,0x28,0x20,1},
    {31,0x04,0x29,0x00,0}, {32,0x04,0x2A,0x00,0}, {33,0x04,0x2B,0x00,0},
    {34,0x24,0x20,0x20,0}, {35,0x24,0x21,0x00,1}, {36,0x44,0x20,0x20,0},
    {37,0x44,0x21,0x00,1}, {38,0x44,0x22,0x00,0}, {39,0x44,0x23,0x00,0},
    {40,0x44,0x24,0x00,0}, {41,0x44,0x25,0x00,0}, {42,0x44,0x26,0x00,0},
    {43,0x44,0x27,0x00,0}, {44,0x44,0x28,0x00,0}, {45,0x44,0x29,0x00,0},
    {46,0x44,0x2A,0x00,0}, {47,0x44,0x2B,0x00,0}, {48,0x44,0x2C,0x00,0},
    {49,0x44,0x2D,0x00,0}, {50,0x44,0x2E,0x00,0}, {51,0x44,0x2F,0x00,0},
    {52,0x44,0x30,0x00,0}, {53,0x44,0x31,0x00,0}, {54,0x44,0x32,0x00,0},
    {55,0x64,0x2E,0x20,1}, {56,0x64,0x2F,0x00,0}, {57,0x64,0x30,0x00,0},
    {58,0x64,0x31,0x00,0}, {59,0x64,0x32,0x00,0}, {60,0x64,0x33,0x00,0},
    {61,0x64,0x34,0x00,0}, {62,0x64,0x35,0x00,0}, {63,0x64,0x36,0x00,0},
    {64,0x64,0x37,0x00,0}, {65,0x64,0x38,0x00,0}, {66,0x65,0x38,0x20,1},
    {67,0x66,0x38,0x20,1}, {68,0x67,0x38,0x20,1}, {69,0x68,0x38,0x20,1},
    {70,0x69,0x38,0x20,1}, {71,0x6A,0x38,0x20,1}, {72,0x6B,0x38,0x20,1},
    {73,0x6C,0x38,0x20,1}, {74,0x6D,0x38,0x20,1}, {75,0x6E,0x38,0x20,1},
    {76,0x6F,0x38,0x20,1}};


boost::uint8_t gain_table_4000mhz_to_6000mhz[77][5] = { {0,0x00,0x00,0x20,1},
    {1,0x00,0x00,0x00,0}, {2,0x00,0x00,0x00,0}, {3,0x00,0x00,0x00,0},
    {4,0x00,0x00,0x00,0}, {5,0x00,0x01,0x00,0}, {6,0x00,0x02,0x00,0},
    {7,0x00,0x03,0x00,0}, {8,0x01,0x01,0x20,1}, {9,0x01,0x02,0x00,0},
    {10,0x01,0x03,0x00,0}, {11,0x01,0x04,0x20,1}, {12,0x01,0x05,0x00,0},
    {13,0x01,0x06,0x00,0}, {14,0x01,0x07,0x00,0}, {15,0x01,0x08,0x00,0},
    {16,0x01,0x09,0x00,0}, {17,0x01,0x0A,0x00,0}, {18,0x01,0x0B,0x00,0},
    {19,0x01,0x0C,0x00,0}, {20,0x02,0x08,0x20,1}, {21,0x02,0x09,0x00,0},
    {22,0x02,0x0A,0x00,0}, {23,0x02,0x0B,0x20,1}, {24,0x02,0x0C,0x00,0},
    {25,0x02,0x0D,0x00,0}, {26,0x02,0x0E,0x00,0}, {27,0x02,0x0F,0x00,0},
    {28,0x02,0x2A,0x20,1}, {29,0x02,0x2B,0x00,0}, {30,0x04,0x27,0x20,1},
    {31,0x04,0x28,0x00,0}, {32,0x04,0x29,0x00,0}, {33,0x04,0x2A,0x00,0},
    {34,0x04,0x2B,0x00,0}, {35,0x04,0x2C,0x00,0}, {36,0x04,0x2D,0x00,0},
    {37,0x24,0x20,0x20,1}, {38,0x24,0x21,0x00,0}, {39,0x24,0x22,0x00,0},
    {40,0x44,0x20,0x20,1}, {41,0x44,0x21,0x00,0}, {42,0x44,0x22,0x00,0},
    {43,0x44,0x23,0x00,0}, {44,0x44,0x24,0x00,0}, {45,0x44,0x25,0x00,0},
    {46,0x44,0x26,0x00,0}, {47,0x44,0x27,0x00,0}, {48,0x44,0x28,0x00,0},
    {49,0x44,0x29,0x00,0}, {50,0x44,0x2A,0x00,0}, {51,0x44,0x2B,0x00,0},
    {52,0x44,0x2C,0x00,0}, {53,0x44,0x2D,0x00,0}, {54,0x44,0x2E,0x00,0},
    {55,0x64,0x2E,0x20,1}, {56,0x64,0x2F,0x00,0}, {57,0x64,0x30,0x00,0},
    {58,0x64,0x31,0x00,0}, {59,0x64,0x32,0x00,0}, {60,0x64,0x33,0x00,0},
    {61,0x64,0x34,0x00,0}, {62,0x64,0x35,0x00,0}, {63,0x64,0x36,0x00,0},
    {64,0x64,0x37,0x00,0}, {65,0x64,0x38,0x00,0}, {66,0x65,0x38,0x20,1},
    {67,0x66,0x38,0x20,1}, {68,0x67,0x38,0x20,1}, {69,0x68,0x38,0x20,1},
    {70,0x69,0x38,0x20,1}, {71,0x6A,0x38,0x20,1}, {72,0x6B,0x38,0x20,1},
    {73,0x6C,0x38,0x20,1}, {74,0x6D,0x38,0x20,1}, {75,0x6E,0x38,0x20,1},
    {76,0x6F,0x38,0x20,1}};


#endif /* INCLUDED_AD9361_GAIN_TABLES_HPP */
