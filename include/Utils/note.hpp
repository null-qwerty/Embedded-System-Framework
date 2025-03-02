#pragma once

struct Note {
    Note();
    Note(int frequency, int duration);
    ~Note();

    int frequency;
    int duration;
    float loudness = 1.0;
};