#pragma once

#include <cstdint>
#include <string>

#include "shm.h"

class Hdf5HandLogger
{
public:
    Hdf5HandLogger();
    ~Hdf5HandLogger();

    bool is_open() const;
    const std::string &file_path() const;

    void append(const SHMmsgs &sample);

private:
    class Impl;
    Impl *impl_;
};
