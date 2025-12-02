// SPDX-License-Identifier: MIT
// Copyright (c) 2025 Truesense Srl

#ifndef UWBACTIVEROUNDS
#define UWBACTIVEROUNDS

#include "UWB.hpp"

// Note: phActiveRoundsConfig_t is not defined in the HAL headers.
// This class provides a minimal implementation for compatibility.
// Active rounds configuration may not be needed for basic DL-TDoA operation.
class UWBActiveRounds {
private:
    size_t size;
    size_t capacity;  // This is the maximum number of elements the array can hold
    void* configs;    // Placeholder - actual type not available

public:
    // Constructor with predefined capacity
    UWBActiveRounds(size_t capacity) : size(0), capacity(capacity), configs(nullptr) {
        // Note: phActiveRoundsConfig_t type is not available, so we can't allocate it
        // This is acceptable since empty rounds are typically used in DL-TDoA
    }

    // Destructor
    ~UWBActiveRounds() {
        // No cleanup needed since configs is not allocated
    }

    // Add a new configuration to the array
    // Note: Implementation disabled since phActiveRoundsConfig_t is not defined
    void addConfig(const void* config) {
        if (size < capacity) {
            size++;
        }
        // Actual configuration storage is not implemented due to missing type definition
    }

    // Access a config element - disabled due to missing type
    void* get(size_t index) {
        return nullptr;
    }

    // Get current size of the array
    size_t getSize() const {
        return size;
    }

    // Get configs pointer - returns nullptr since type is not defined
    void* getConfigs() {
        return nullptr;
    }
};


#endif/* UWBACTIVEROUNDS */
