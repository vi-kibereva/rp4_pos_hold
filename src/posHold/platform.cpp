#include "platform.h"
#include <iostream>

#ifdef _WIN32
#include <windows.h>

bool Keyboard::isPressed(KeyCode key)
{
    switch (key) {
        case KeyCode::Up:    return GetAsyncKeyState(VK_UP) & 0x8000;
        case KeyCode::Left:  return GetAsyncKeyState(VK_LEFT) & 0x8000;
        case KeyCode::Right: return GetAsyncKeyState(VK_RIGHT) & 0x8000;
        case KeyCode::Quit:  return GetAsyncKeyState(VK_ESCAPE) & 0x8000;
        default:             return false;
    }
}

void Keyboard::update() {} // No-op on Windows

#else
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <cstdio>
#include <chrono>
#include <unordered_map>

static struct termios orig_termios;
static bool terminal_initialized = false;

// Track last time each key was seen (for hold detection)
static std::unordered_map<char, std::chrono::steady_clock::time_point> key_last_seen;
static const auto HOLD_TIMEOUT = std::chrono::milliseconds(150); // If no input in 150ms, key is released

static void initTerminal()
{
    if (!terminal_initialized) {
        tcgetattr(STDIN_FILENO, &orig_termios);
        struct termios raw = orig_termios;
        raw.c_lflag &= ~(ICANON | ECHO);
        raw.c_cc[VMIN] = 0;
        raw.c_cc[VTIME] = 0;
        tcsetattr(STDIN_FILENO, TCSANOW, &raw);
        fcntl(STDIN_FILENO, F_SETFL, fcntl(STDIN_FILENO, F_GETFL, 0) | O_NONBLOCK);
        terminal_initialized = true;

        atexit([]() {
            if (terminal_initialized) {
                tcsetattr(STDIN_FILENO, TCSANOW, &orig_termios);
            }
        });
    }
}

void Keyboard::update()
{
    initTerminal();

    auto now = std::chrono::steady_clock::now();

    // Read all available characters and update their timestamps
    char c = getchar();
    std::cout << "Read char: '" << c << "' (ASCII: " << (int)c << ")" << std::endl;
    while ((c) != EOF) {
        // Debug output
        std::cout << "Read char: '" << c << "' (ASCII: " << (int)c << ")" << std::endl;

        // Handle escape sequences for arrow keys
        if (c == '\x1B') { // ESC character
            char seq[2];
            if (read(STDIN_FILENO, &seq[0], 1) == 1 &&
                read(STDIN_FILENO, &seq[1], 1) == 1) {
                if (seq[0] == '[') {
                    switch (seq[1]) {
                        case 'A': c = 'w'; break; // Up arrow -> w
                        case 'D': c = 'a'; break; // Left arrow -> a
                        case 'C': c = 'd'; break; // Right arrow -> d
                        default: continue; // Ignore other escape sequences
                    }
                    std::cout << "Arrow key mapped to: '" << c << "'" << std::endl;
                } else {
                    continue; // Not an arrow key sequence
                }
            } else {
                continue; // Incomplete sequence
            }
        }

        key_last_seen[c] = now;
        c = getchar();
    }

    // Remove keys that haven't been seen recently (released)
    for (auto it = key_last_seen.begin(); it != key_last_seen.end(); ) {
        if (now - it->second > HOLD_TIMEOUT) {
            std::cout << "Key '" << it->first << "' released (timeout)" << std::endl;
            it = key_last_seen.erase(it);
        } else {
            ++it;
        }
    }
}

bool Keyboard::isPressed(KeyCode key)
{
    char target = 0;
    switch (key) {
        case KeyCode::Up:    target = 'w'; break;
        case KeyCode::Left:  target = 'a'; break;
        case KeyCode::Right: target = 'd'; break;
        case KeyCode::Quit:  target = 'q'; break;
        default: return false;
    }

    return key_last_seen.find(target) != key_last_seen.end();
}
#endif