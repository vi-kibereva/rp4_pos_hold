#pragma once
#ifndef PLATFORM_H
#define PLATFORM_H

#include <unordered_map>
#include <string>

enum class KeyCode {
    Up,
    Left,
    Right,
    Quit,
    Unknown
};

class Keyboard {
public:
    static bool isPressed(KeyCode key);
    static void update();
};

#endif