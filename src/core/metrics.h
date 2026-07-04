#ifndef PATHFINDING_METRICS_H
#define PATHFINDING_METRICS_H

#include <cstddef>

struct Metrics
{
    std::size_t path_size {};
    std::size_t nodes_expanded {};
    double search_time {};
    bool path_found {false};
};

#endif //PATHFINDING_METRICS_H