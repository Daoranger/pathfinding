//
// Created by hoang on 4/2/2026.
//

#include "agent.h"

#include <algorithm>
#include <limits>

#include "../grid/grid.h"

Agent::Agent(Cell* start_cell, Cell* goal_cell, const Grid& grid, sf::Color color, bool record_search_snapshots)
    : start_cell_(start_cell)
    , goal_cell_(goal_cell)
    , grid_(grid)
    , color_(color)
    , record_search_snapshots_(record_search_snapshots)
{
}

void Agent::setHeuristicOptions(bool tie_break_nudge, double weight) noexcept
{
    heuristic_tie_break_nudge_ = tie_break_nudge;
    heuristic_weight_ = std::max(0.0, weight);
}

void Agent::clearSearchState() noexcept
{
    snapshots_.clear();
    snapshot_index_ = 0;
    path_.clear();
    metrics_ = {};
    g_.clear();
    f_.clear();
    h_.clear();
    parent_.clear();
}

void Agent::runAStar()
{
    snapshots_.clear();
    snapshot_index_ = 0;
    path_.clear();
    metrics_ = {};

    Snapshot snapshot;
    std::set<Cell*, CompareCell> openSet(CompareCell{f_});
    std::unordered_set<Cell*> closedSet;

    // start cell
    g_[start_cell_] = 0;
    h_[start_cell_] = heuristic(*start_cell_, *goal_cell_);
    f_[start_cell_] = g_.at(start_cell_) + h_.at(start_cell_);
    parent_[start_cell_] = nullptr;


    // add start cell to openList
    openSet.insert(start_cell_);
    if (record_search_snapshots_)
    {
        snapshot.frontier_ = extractNodes(openSet);
        snapshot.explored_ = extractNodes(closedSet);
        snapshots_.push_back(snapshot);
    }

    // processing current cell in open set
    while (!openSet.empty())
    {
        // current cell = node with lowest m_f in openList
        Cell* currCell = *openSet.begin();

        // if current cell is goal, reconstruct and return path
        if (*currCell == *goal_cell_)
        {
            std::vector<Cell*> path;

            while (currCell != nullptr)
            {
                path.push_back(currCell);
                currCell = parent_.at(currCell);
            }
            // change from goal-to-start order to start-to-goal order
            std::reverse(path.begin(), path.end());
            path_ = path;
            metrics_.nodes_expanded = closedSet.size() + 1;
            metrics_.path_found = true;
            return;
        }

        // remove current cell from openList
        openSet.erase(currCell);

        // add current cell to closedList
        closedSet.insert(currCell);

        if (record_search_snapshots_)
        {
            snapshot.frontier_ = extractNodes(openSet);
            snapshot.explored_ = extractNodes(closedSet);
            snapshots_.push_back(snapshot);
        }

        // for each neighbor of current cell
        for (auto& n : getValidNeighbors(*currCell))
        {
            Cell* neighborCell = const_cast<Cell*>(&grid_.cells_[n.first][n.second]);

            // if neighbor in closedList
            if (closedSet.find(neighborCell) != closedSet.end())
            {
                continue;
            }

            // the distance from start to a neighbor, if we follow current path
            // because I am using a uniform grid where you can only move to adjacent cells,
            // the edge cost between any two neighbors is always 1
            double tentative_g = g_.at(currCell) + 1.0;

            // if neighbor is not in openList or this path to neighbor cell is better than any previous one
            // this mean path from start to current cell cost less than path from start to any previous one
            if (openSet.find(neighborCell) == openSet.end() || tentative_g < g_.at(neighborCell))
            {
                openSet.erase(neighborCell);
                parent_[neighborCell] = currCell;
                g_[neighborCell] = tentative_g;
                h_[neighborCell] = heuristic(*neighborCell, *goal_cell_);
                f_[neighborCell] = g_.at(neighborCell) + h_.at(neighborCell);
                openSet.insert(neighborCell);
            }
        }
    }

    // no path found
    metrics_.nodes_expanded = closedSet.size();
    metrics_.path_found = false;
}

void Agent::runDijkstra()
{
    snapshots_.clear();
    snapshot_index_ = 0;
    path_.clear();
    metrics_ = {};

    Snapshot snapshot;
    std::set<Cell*, CompareCell> openSet(CompareCell{f_});
    std::unordered_set<Cell*> closedSet;

    // start cell
    g_[start_cell_] = 0;
    f_[start_cell_] = g_.at(start_cell_);
    parent_[start_cell_] = nullptr;


    // add start cell to openList
    openSet.insert(start_cell_);
    if (record_search_snapshots_)
    {
        snapshot.frontier_ = extractNodes(openSet);
        snapshot.explored_ = extractNodes(closedSet);
        snapshots_.push_back(snapshot);
    }

    // processing current cell in open set
    while (!openSet.empty())
    {
        // current cell = node with lowest m_f in openList
        Cell* currCell = *openSet.begin();

        // if current cell is goal, reconstruct and return path
        if (*currCell == *goal_cell_)
        {
            std::vector<Cell*> path;

            while (currCell != nullptr)
            {
                path.push_back(currCell);
                currCell = parent_.at(currCell);
            }
            // change from goal-to-start order to start-to-goal order
            std::reverse(path.begin(), path.end());
            path_ = path;
            metrics_.nodes_expanded = closedSet.size() + 1;
            metrics_.path_found = true;
            return;
        }

        // remove current cell from openList
        openSet.erase(currCell);

        // add current cell to closedList
        closedSet.insert(currCell);

        if (record_search_snapshots_)
        {
            snapshot.frontier_ = extractNodes(openSet);
            snapshot.explored_ = extractNodes(closedSet);
            snapshots_.push_back(snapshot);
        }

        // for each neighbor of current cell
        for (auto& n : getValidNeighbors(*currCell))
        {
            Cell* neighborCell = const_cast<Cell*>(&grid_.cells_[n.first][n.second]);

            // if neighbor in closedList
            if (closedSet.find(neighborCell) != closedSet.end())
            {
                continue;
            }

            // the distance from start to a neighbor, if we follow current path
            // because I am using a uniform grid where you can only move to adjacent cells,
            // the edge cost between any two neighbors is always 1
            double tentative_g = g_.at(currCell) + 1.0;

            // if neighbor is not in openList or this path to neighbor cell is better than any previous one
            // this mean path from start to current cell cost less than path from start to any previous one
            if (openSet.find(neighborCell) == openSet.end() || tentative_g < g_.at(neighborCell))
            {
                openSet.erase(neighborCell);
                parent_[neighborCell] = currCell;
                g_[neighborCell] = tentative_g;
                f_[neighborCell] = g_.at(neighborCell);
                openSet.insert(neighborCell);
            }
        }
    }

    // no path found
    metrics_.nodes_expanded = closedSet.size();
    metrics_.path_found = false;
}

void Agent::runGreedy()
{
    snapshots_.clear();
    snapshot_index_ = 0;
    path_.clear();
    metrics_ = {};

    Snapshot snapshot;
    std::set<Cell*, CompareCell> openSet(CompareCell{f_});
    std::unordered_set<Cell*> closedSet;

    // start cell
    h_[start_cell_] = heuristic(*start_cell_, *goal_cell_);
    f_[start_cell_] = h_.at(start_cell_);
    parent_[start_cell_] = nullptr;


    // add start cell to openList
    openSet.insert(start_cell_);
    if (record_search_snapshots_)
    {
        snapshot.frontier_ = extractNodes(openSet);
        snapshot.explored_ = extractNodes(closedSet);
        snapshots_.push_back(snapshot);
    }

    // processing current cell in open set
    while (!openSet.empty())
    {
        // current cell = node with lowest m_f in openList
        Cell* currCell = *openSet.begin();

        // if current cell is goal, reconstruct and return path
        if (*currCell == *goal_cell_)
        {
            std::vector<Cell*> path;

            while (currCell != nullptr)
            {
                path.push_back(currCell);
                currCell = parent_.at(currCell);
            }
            // change from goal-to-start order to start-to-goal order
            std::reverse(path.begin(), path.end());
            path_ = path;
            metrics_.nodes_expanded = closedSet.size() + 1;
            metrics_.path_found = true;
            return;
        }

        // remove current cell from openList
        openSet.erase(currCell);

        // add current cell to closedList
        closedSet.insert(currCell);

        if (record_search_snapshots_)
        {
            snapshot.frontier_ = extractNodes(openSet);
            snapshot.explored_ = extractNodes(closedSet);
            snapshots_.push_back(snapshot);
        }

        // for each neighbor of current cell
        for (auto& n : getValidNeighbors(*currCell))
        {
            Cell* neighborCell = const_cast<Cell*>(&grid_.cells_[n.first][n.second]);

            // if neighbor in closedList
            if (closedSet.find(neighborCell) != closedSet.end())
            {
                continue;
            }

            // if neighbor is not in openList or this path to neighbor cell is better than any previous one
            // this mean path from start to current cell cost less than path from start to any previous one
            if (openSet.find(neighborCell) == openSet.end())
            {
                openSet.erase(neighborCell);
                parent_[neighborCell] = currCell;
                h_[neighborCell] = heuristic(*neighborCell, *goal_cell_);
                f_[neighborCell] = h_.at(neighborCell);
                openSet.insert(neighborCell);
            }
        }
    }

    // no path found
    metrics_.nodes_expanded = closedSet.size();
    metrics_.path_found = false;
}

void Agent::runBFS()
{
    snapshots_.clear();
    snapshot_index_ = 0;
    path_.clear();
    metrics_ = {};

    Snapshot snapshot;
    std::deque<Cell*> openDeque;
    std::unordered_set<Cell*> visitedSet;

    // start cell
    parent_[start_cell_] = nullptr;
    openDeque.push_back(start_cell_);
    visitedSet.insert(start_cell_);

    if (record_search_snapshots_)
    {
        snapshot.frontier_ = extractNodes(openDeque);
        snapshot.explored_ = extractNodes(visitedSet);
        snapshots_.push_back(snapshot);
    }

    // processing current cell in open set
    while (!openDeque.empty())
    {
        Cell* currCell = openDeque.front();
        openDeque.pop_front();

        // if current cell is goal, reconstruct and return path
        if (*currCell == *goal_cell_)
        {
            std::vector<Cell*> path;

            while (currCell != nullptr)
            {
                path.push_back(currCell);
                currCell = parent_.at(currCell);
            }
            // change from goal-to-start order to start-to-goal order
            std::reverse(path.begin(), path.end());
            path_ = path;
            metrics_.nodes_expanded = visitedSet.size() + 1;
            metrics_.path_found = true;
            return;
        }

        // for each neighbor of current cell
        for (auto& n : getValidNeighbors(*currCell))
        {
            Cell* neighborCell = const_cast<Cell*>(&grid_.cells_[n.first][n.second]);

            // if neighbor in closedList (we have visited)
            if (visitedSet.find(neighborCell) != visitedSet.end())
            {
                continue;
            }

            // if we haven't visited the cell
            visitedSet.insert(neighborCell);
            parent_[neighborCell] = currCell;
            openDeque.push_back(neighborCell);
        }

        // take Snapshot AFTER adding neighbors
        if (record_search_snapshots_)
        {
            snapshot.frontier_ = extractNodes(openDeque);
            snapshot.explored_ = extractNodes(visitedSet);
            snapshots_.push_back(snapshot);
        }
    }

    // no path found
    metrics_.nodes_expanded = visitedSet.size();
    metrics_.path_found = false;
}

void Agent::runDFS()
{
    snapshots_.clear();
    snapshot_index_ = 0;
    path_.clear();
    metrics_ = {};

    Snapshot snapshot;
    std::deque<Cell*> openDeque;
    std::unordered_set<Cell*> visitedSet;

    // start cell
    parent_[start_cell_] = nullptr;
    openDeque.push_back(start_cell_);
    visitedSet.insert(start_cell_);

    if (record_search_snapshots_)
    {
        snapshot.frontier_ = extractNodes(openDeque);
        snapshot.explored_ = extractNodes(visitedSet);
        snapshots_.push_back(snapshot);
    }

    // processing current cell in open set
    while (!openDeque.empty())
    {
        Cell* currCell = openDeque.back();
        openDeque.pop_back();

        // if current cell is goal, reconstruct and return path
        if (*currCell == *goal_cell_)
        {
            std::vector<Cell*> path;

            while (currCell != nullptr)
            {
                path.push_back(currCell);
                currCell = parent_.at(currCell);
            }
            // change from goal-to-start order to start-to-goal order
            std::reverse(path.begin(), path.end());
            path_ = path;
            metrics_.nodes_expanded = visitedSet.size() + 1;
            metrics_.path_found = true;
            return;
        }

        // for each neighbor of current cell
        for (auto& n : getValidNeighbors(*currCell))
        {
            Cell* neighborCell = const_cast<Cell*>(&grid_.cells_[n.first][n.second]);

            // if neighbor in closedList (we have visited)
            if (visitedSet.find(neighborCell) != visitedSet.end())
            {
                continue;
            }

            // if we haven't visited the cell
            visitedSet.insert(neighborCell);
            parent_[neighborCell] = currCell;
            openDeque.push_back(neighborCell);
        }

        // take Snapshot AFTER adding neighbors
        if (record_search_snapshots_)
        {
            snapshot.frontier_ = extractNodes(openDeque);
            snapshot.explored_ = extractNodes(visitedSet);
            snapshots_.push_back(snapshot);
        }
    }

    // no path found
    metrics_.nodes_expanded = visitedSet.size();
    metrics_.path_found = false;
}

// Bidirectional A* (BA*): round-robin scheduling for balanced visuals; stop when L + R >= mu.
void Agent::runBidirectionalAStar()
{
    snapshots_.clear();
    snapshot_index_ = 0;
    path_.clear();
    metrics_ = {};

    if (start_cell_ == nullptr || goal_cell_ == nullptr)
    {
        metrics_.path_found = false;
        return;
    }
    if (*start_cell_ == *goal_cell_)
    {
        path_ = {start_cell_};
        metrics_.path_found = true;
        metrics_.path_size = 1;
        metrics_.nodes_expanded = 1;
        return;
    }

    std::unordered_map<Cell*, double> g_forward_;
    std::unordered_map<Cell*, double> f_forward_;
    std::unordered_map<Cell*, Cell*> parent_forward_;
    std::set<Cell*, CompareCell> open_forward_{CompareCell{f_forward_}};
    std::unordered_set<Cell*> closed_forward_;

    // Backward:  g_b, f_b, parent_b, open_b, closed_b
    std::unordered_map<Cell*, double> g_backward_;
    std::unordered_map<Cell*, double> f_backward_;
    std::unordered_map<Cell*, Cell*> parent_backward_;
    std::set<Cell*, CompareCell> open_backward_{CompareCell{f_backward_}};
    std::unordered_set<Cell*> closed_backward_;

    g_forward_[start_cell_] = 0;
    f_forward_[start_cell_] = heuristic(*start_cell_, *goal_cell_);
    parent_forward_[start_cell_] = nullptr;
    open_forward_.insert(start_cell_);

    g_backward_[goal_cell_] = 0;
    f_backward_[goal_cell_] = heuristic(*goal_cell_, *start_cell_);
    parent_backward_[goal_cell_] = nullptr;
    open_backward_.insert(goal_cell_);

    double mu = std::numeric_limits<double>::infinity();
    Cell* best_meeting = nullptr;

    auto try_meeting = [&](Cell* n)
    {
        if (g_forward_.count(n) && g_backward_.count(n))
        {
            const double c = g_forward_.at(n) + g_backward_.at(n);
            if (c < mu)
            {
                mu = c;
                best_meeting = n;
            }
        }
    };

    Snapshot snapshot;
    const auto push_merged_snapshot = [&]()
    {
        if (!record_search_snapshots_)
            return;
        std::vector<Cell*> frontier;
        frontier.reserve(open_forward_.size() + open_backward_.size());
        frontier.insert(frontier.end(), open_forward_.begin(), open_forward_.end());
        frontier.insert(frontier.end(), open_backward_.begin(), open_backward_.end());
        std::unordered_set<Cell*> explored;
        explored.reserve(closed_forward_.size() + closed_backward_.size());
        explored.insert(closed_forward_.begin(), closed_forward_.end());
        explored.insert(closed_backward_.begin(), closed_backward_.end());
        snapshot.frontier_ = std::move(frontier);
        snapshot.explored_ = extractNodes(explored);
        snapshots_.push_back(snapshot);
    };

    push_merged_snapshot();

    bool forward_turn = true;
    while (!open_forward_.empty() || !open_backward_.empty())
    {
        // Optimal stop (check before expanding this step)
        if (mu < std::numeric_limits<double>::infinity())
        {
            const double L = open_forward_.empty()
                ? std::numeric_limits<double>::infinity()
                : f_forward_.at(*open_forward_.begin());
            const double R = open_backward_.empty()
                ? std::numeric_limits<double>::infinity()
                : f_backward_.at(*open_backward_.begin());
            if (L + R >= mu)
                break;
        }

        const bool can_f = !open_forward_.empty();
        const bool can_b = !open_backward_.empty();
        if (!can_f && !can_b)
            break;

        Cell* currCell = nullptr;
        bool expand_forward = false;
        if (forward_turn && can_f)
        {
            currCell = *open_forward_.begin();
            open_forward_.erase(currCell);
            closed_forward_.insert(currCell);
            expand_forward = true;
        }
        else if (!forward_turn && can_b)
        {
            currCell = *open_backward_.begin();
            open_backward_.erase(currCell);
            closed_backward_.insert(currCell);
            expand_forward = false;
        }
        else if (can_f)
        {
            currCell = *open_forward_.begin();
            open_forward_.erase(currCell);
            closed_forward_.insert(currCell);
            expand_forward = true;
        }
        else if (can_b)
        {
            currCell = *open_backward_.begin();
            open_backward_.erase(currCell);
            closed_backward_.insert(currCell);
            expand_forward = false;
        }
        else
            break;

        forward_turn = !forward_turn;

        try_meeting(currCell);

        if (expand_forward && *currCell == *goal_cell_)
        {
            std::vector<Cell*> path;
            for (Cell* x = goal_cell_; x != nullptr; x = parent_forward_.at(x))
                path.push_back(x);
            std::reverse(path.begin(), path.end());
            path_ = std::move(path);
            metrics_.nodes_expanded = closed_forward_.size() + closed_backward_.size();
            metrics_.path_found = true;
            metrics_.path_size = path_.size();
            return;
        }
        if (!expand_forward && *currCell == *start_cell_)
        {
            std::vector<Cell*> path;
            for (Cell* x = start_cell_; x != nullptr; x = parent_backward_.at(x))
            {
                path.push_back(x);
                if (x == goal_cell_)
                    break;
            }
            path_ = std::move(path);
            metrics_.nodes_expanded = closed_forward_.size() + closed_backward_.size();
            metrics_.path_found = true;
            metrics_.path_size = path_.size();
            return;
        }

        if (expand_forward)
        {
            for (auto& n : getValidNeighbors(*currCell))
            {
                Cell* neighborCell = const_cast<Cell*>(&grid_.cells_[n.first][n.second]);
                if (closed_forward_.find(neighborCell) != closed_forward_.end())
                    continue;
                const double tentative_g = g_forward_.at(currCell) + 1.0;
                const bool have_g = g_forward_.count(neighborCell) != 0;
                if (have_g && tentative_g >= g_forward_.at(neighborCell))
                    continue;

                open_forward_.erase(neighborCell);
                parent_forward_[neighborCell] = currCell;
                g_forward_[neighborCell] = tentative_g;
                f_forward_[neighborCell] = tentative_g + heuristic(*neighborCell, *goal_cell_);
                open_forward_.insert(neighborCell);
                try_meeting(neighborCell);
            }
        }
        else
        {
            for (auto& n : getValidNeighbors(*currCell))
            {
                Cell* neighborCell = const_cast<Cell*>(&grid_.cells_[n.first][n.second]);
                if (closed_backward_.find(neighborCell) != closed_backward_.end())
                    continue;
                const double tentative_g = g_backward_.at(currCell) + 1.0;
                const bool have_g = g_backward_.count(neighborCell) != 0;
                if (have_g && tentative_g >= g_backward_.at(neighborCell))
                    continue;

                open_backward_.erase(neighborCell);
                parent_backward_[neighborCell] = currCell;
                g_backward_[neighborCell] = tentative_g;
                f_backward_[neighborCell] = tentative_g + heuristic(*neighborCell, *start_cell_);
                open_backward_.insert(neighborCell);
                try_meeting(neighborCell);
            }
        }

        push_merged_snapshot();
    }

    metrics_.nodes_expanded = closed_forward_.size() + closed_backward_.size();

    if (best_meeting != nullptr && mu < std::numeric_limits<double>::infinity())
    {
        std::vector<Cell*> head;
        for (Cell* x = best_meeting; x != nullptr; x = parent_forward_.at(x))
            head.push_back(x);
        std::reverse(head.begin(), head.end());

        std::vector<Cell*> tail;
        for (Cell* x = best_meeting; x != nullptr; x = parent_backward_.at(x))
        {
            tail.push_back(x);
            if (x == goal_cell_)
                break;
        }
        path_ = std::move(head);
        for (std::size_t i = 1; i < tail.size(); ++i)
            path_.push_back(tail[i]);
        metrics_.path_found = true;
        metrics_.path_size = path_.size();
        return;
    }

    metrics_.path_found = false;
}

sf::Color Agent::getColor() const
{
    return color_;
}

Cell * Agent::getStartCell() const
{
    return start_cell_;
}

Cell * Agent::getGoalCell() const
{
    return goal_cell_;
}

double Agent::heuristic(const Cell &currCell, const Cell &goalCell)
{
    const double manhattan = static_cast<double>(std::abs(currCell.x_ - goalCell.x_))
        + static_cast<double>(std::abs(currCell.y_ - goalCell.y_));
    const double tie = heuristic_tie_break_nudge_ ? 1.001 : 1.0;
    return manhattan * tie * heuristic_weight_;
}

std::vector<std::pair<int, int>> Agent::getValidNeighbors(const Cell &currCell) const
{
    int currCellX = currCell.x_;
    int currCellY = currCell.y_;

    // pair<int, int>: first = row, second = col
    std::vector<std::pair<int, int>> possible_neighbors {
                {currCellX - 1, currCellY},
                {currCellX + 1, currCellY},
                {currCellX, currCellY - 1},
                {currCellX, currCellY + 1}
    };

    std::vector<std::pair<int, int>> valid_neighbors {};

    for (auto& pn : possible_neighbors)
    {
        if ((pn.first >= 0 && pn.second >=0) && (pn.first < grid_.getRows() && pn.second < grid_.getCols()) && (grid_.cells_[pn.first][pn.second].getType() != CellType::obstacle))
        {
            valid_neighbors.push_back(pn);
        }
    }
    return valid_neighbors;
}

std::vector<Cell *> Agent::extractNodes(const std::set<Cell *, CompareCell> &set)
{
    return std::vector<Cell*>(set.begin(), set.end());
}

std::vector<Cell *> Agent::extractNodes(const std::unordered_set<Cell *> &unordered_set)
{
    return std::vector<Cell*>(unordered_set.begin(), unordered_set.end());
}

std::vector<Cell *> Agent::extractNodes(const std::deque<Cell*>& deque)
{
    return std::vector<Cell*>(deque.begin(), deque.end());
}
