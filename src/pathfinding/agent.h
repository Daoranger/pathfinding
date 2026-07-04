// Copyright 2026, Hoang Nguyen
//
// agent.h
//
// Defines the Agent class, which represent an AI agent with a starting position.
// It task is to find the path to the goal position

#ifndef PATHFINDING_AGENT_H
#define PATHFINDING_AGENT_H

#include <iostream>
#include <set>
#include <unordered_map>
#include <unordered_set>
#include <vector>
#include <deque>

#include "../grid/cell.h"
#include "../grid/grid.h"
#include "../core/metrics.h"
#include "snapshot.h"


// Return true if cell a have lower f value than cell b.
// Used by the open set as it acts like a priority queue,
// where cell is lower f have higher priority
struct CompareCell
{
    const std::unordered_map<Cell*, double>& f_map;

    bool operator()(Cell* a, Cell* b) const
    {
        double fa = f_map.count(a) ? f_map.at(a) : std::numeric_limits<double>::infinity();
        double fb = f_map.count(b) ? f_map.at(b) : std::numeric_limits<double>::infinity();
        if (fa != fb)
            return fa < fb;
        return a < b;
    }
};

class Agent
{

public:

    Agent(Cell* start_cell, Cell* goal_cell, const Grid& grid, sf::Color color,
          bool record_search_snapshots = true);
    void runAStar();
    void runDijkstra();
    void runGreedy();
    void runBFS();
    void runDFS();

    /// Bidirectional A* (BA*)
    void runBidirectionalAStar();

    /// Drop path, snapshots, and search maps so bulk teardown (e.g. Reset) stays fast.
    void clearSearchState() noexcept;

    /// Used by A*, Greedy, BA*: `h = Manhattan * (nudge ? 1.001 : 1) * weight` (f = g + h).
    void setHeuristicOptions(bool tie_break_nudge, double weight) noexcept;

    sf::Color getColor() const;
    Cell* getStartCell() const;
    Cell* getGoalCell() const;

    std::vector<Snapshot> snapshots_;
    int snapshot_index_ = 0;
    std::vector<Cell*> path_;
    Metrics metrics_;

private:

    // Return the distance between the current cell to goal cell
    double heuristic(const Cell& currCell, const Cell& goalCell);

    // Return valid neighbors of current cell
    std::vector<std::pair<int, int>> getValidNeighbors(const Cell& currCell) const;

    // Uses to extract nodes in open set for snapshots
    std::vector<Cell*> extractNodes(const std::set<Cell*,CompareCell>& set);

    // Uses to extract nodes in closed set for snapshots
    std::vector<Cell*> extractNodes(const std::unordered_set<Cell*>& unordered_set);

    std::vector<Cell*> extractNodes(const std::deque<Cell*>& deque);


    const Grid& grid_;

    sf::Color color_;
    Cell* start_cell_;
    Cell* goal_cell_;

    bool record_search_snapshots_;

    bool heuristic_tie_break_nudge_ = true;
    double heuristic_weight_ = 1.0;

    std::unordered_map<Cell*, double> g_;
    std::unordered_map<Cell*, double> f_;
    std::unordered_map<Cell*, double> h_;
    std::unordered_map<Cell*, Cell*> parent_;

};


#endif //PATHFINDING_AGENT_H