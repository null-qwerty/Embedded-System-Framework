#pragma once

enum Status {
    STATUS_SUCCESS = 0, // 成功
    STATUS_INITUALIZED = 1, // 已初始化
    STATUS_DEINITUALIZED = 2, // 未初始化
    STATUS_INITUALIZING = 3, // 正在初始化
    STATUS_DEINITUALIZING = 4, // 正在反初始化
    STATUS_ERROR = 5, // 错误
};