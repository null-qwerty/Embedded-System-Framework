#pragma once

class Note {
public:
    Note();
    Note(int frequency, int duration);
    ~Note();

    Note &play();

private:
    int frequency;
    int duration;
};