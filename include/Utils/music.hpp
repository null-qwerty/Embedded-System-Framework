#pragma once
#include "buzzer.hpp"

// clang-format off
#define L1      FREQUENCY_TO_ARR(262)   // 低调　do 的频率
#define L1s     FREQUENCY_TO_ARR(277)   // 低调　do# 的频率
#define L2b     L1s                     // 低调　reb 的频率
#define L2      FREQUENCY_TO_ARR(294)   // 低调　re 的频率
#define L2s     FREQUENCY_TO_ARR(311)   // 低调　re# 的频率
#define L3b     L2s                     // 低调　mib 的频率
#define L3      FREQUENCY_TO_ARR(330)   // 低调　mi 的频率
#define L4      FREQUENCY_TO_ARR(349)   // 低调　fa 的频率
#define L4s     FREQUENCY_TO_ARR(370)   // 低调　fa# 的频率
#define L5b     L4s                     // 低调　sob 的频率
#define L5      FREQUENCY_TO_ARR(392)   // 低调　so 的频率
#define L5s     FREQUENCY_TO_ARR(415)   // 低调　so# 的频率
#define L6b     L5s                     // 低调　lab 的频率
#define L6      FREQUENCY_TO_ARR(440)   // 低调　la 的频率
#define L6s     FREQUENCY_TO_ARR(466)   // 低调　la# 的频率
#define L7b     L6s                     // 低调　sib 的频率
#define L7      FREQUENCY_TO_ARR(494)   // 低调　si 的频率

#define M1      FREQUENCY_TO_ARR(523)   // 中调　do 的频率
#define M1s     FREQUENCY_TO_ARR(554)   // 中调　do# 的频率
#define M2b     M1s                     // 中调　reb 的频率
#define M2      FREQUENCY_TO_ARR(587)   // 中调　re 的频率
#define M2s     FREQUENCY_TO_ARR(622)   // 中调　re# 的频率
#define M3b     M2s                     // 中调　mib 的频率
#define M3      FREQUENCY_TO_ARR(659)   // 中调　mi 的频率
#define M4      FREQUENCY_TO_ARR(698)   // 中调　fa 的频率
#define M4s     FREQUENCY_TO_ARR(740)   // 中调　fa# 的频率
#define M5b     M4s                     // 中调　sob 的频率
#define M5      FREQUENCY_TO_ARR(784)   // 中调　so 的频率
#define M5s     FREQUENCY_TO_ARR(831)   // 中调　so# 的频率
#define M6b     M5s                     // 中调　lab 的频率
#define M6      FREQUENCY_TO_ARR(880)   // 中调　la 的频率
#define M6s     FREQUENCY_TO_ARR(932)   // 中调　la# 的频率
#define M7b     M6s                     // 中调　sib 的频率
#define M7      FREQUENCY_TO_ARR(988)   // 中调　si 的频率

#define H1      FREQUENCY_TO_ARR(1046)  // 高调　do 的频率
#define H1s     FREQUENCY_TO_ARR(1109)  // 高调　do# 的频率
#define H2b     H1s                     // 高调　reb 的频率
#define H2      FREQUENCY_TO_ARR(1175)  // 高调　re 的频率
#define H2s     FREQUENCY_TO_ARR(1245)  // 高调　re# 的频率
#define H3b     H2s                     // 高调　mib 的频率
#define H3      FREQUENCY_TO_ARR(1318)  // 高调　mi 的频率
#define H4      FREQUENCY_TO_ARR(1397)  // 高调　fa 的频率
#define H4s     FREQUENCY_TO_ARR(1480)  // 高调　fa# 的频率
#define H5b     H4s                     // 高调　sob 的频率
#define H5      FREQUENCY_TO_ARR(1568)  // 高调　so 的频率
#define H5s     FREQUENCY_TO_ARR(1661)  // 高调　so# 的频率
#define H6b     H5s                     // 高调　lab 的频率
#define H6      FREQUENCY_TO_ARR(1760)  // 高调　la 的频率
#define H6s     FREQUENCY_TO_ARR(1865)  // 高调　la# 的频率
#define H7b     H6s                     // 高调　sib 的频率
#define H7      FREQUENCY_TO_ARR(1976)  // 高调　si 的频率

#define Z0      0                       // 静音
// clang-format on
