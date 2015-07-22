#include "peak.h"

uint8_t has_ping(uint32_t* buff, uint32_t size, uint8_t threshold)
{
    uint32_t sum = 0;
    for (int i = 0; i < size; i++)
    {
        sum += abs(buff[i] - DC_OFFSET);
    }

    // Determine maximum energy.
    uint32_t max_sum = DC_OFFSET * size;

    // Return whether the buffer's sum is at least a percentage of the max.
    return (100 * sum) >= (threshold * max_sum);
}
