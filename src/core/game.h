// Copyright 2026, Hoang Nguyen
//
// game.h
//
// Defines the Game class, which manages the main application loop,
// handles user input, and orchestrates the grid, pathfinding, and visualization.

#ifndef PATHFINDING_GAME_H
#define PATHFINDING_GAME_H


#include "SFML/Graphics/RenderWindow.hpp"

#include "../grid/grid.h"
#include "../pathfinding/agent.h"

enum class InputMode
{
    kSelecting,
    kDeselecting
};

enum class PlacementState
{
    kNeedsStart,
    kNeedsGoal,
    kPlacingObstacles
};

enum class AppState
{
    kIdle,
    kAnimating,
    kDone
};

enum class GameMode
{
    kSinglePathfinding,
    kMultiPathfinding,
};

enum class ParallelStrategy
{
    kSequential,
    kPerAgentThread,
    kOpenMP
};

enum class Algorithm
{
    kAStar,
    kDijkstra,
    kGreedy,
    kBFS,
    kDFS,
    kBidirectionalAStar,
};

class Game
{

public:

    Game();
    void run();
    void processEvents();
    void update();
    void draw();
    void drawAgents();
    void drawAgent(Agent& agent);
    void onDrag(const sf::Event::MouseMoved& mouseEvent, const sf::Vector2f& worldPos);
    void onMouseClick(const sf::Event::MouseButtonPressed& mouseEvent, const sf::Vector2f& worldPos);
    void runAlgorithm();
    void runAlgorithmOnAgent(Agent* agent, Algorithm algorithm);
    void initAgents();
    void saveGridToFile();
    void loadGridFromFile();

private:

    sf::Vector2f getGridOffset() const;
    void selectCell(int row, int col);
    void deselectCell(int row, int col);
    void reset();
    void renderImGuiPanels();

    Cell* start_cell_ = nullptr;
    Cell* goal_cell_ = nullptr;

    std::vector<std::unique_ptr<Agent>> agents_;

    sf::RenderWindow window_;
    sf::View view_;
    Grid grid_;
    sf::Clock snapshot_clock_;
    float delay_ = 0.01;
    sf::Clock imgui_clock_;

    Metrics metrics_{};

    PlacementState placement_state_ = PlacementState::kNeedsStart;
    InputMode input_mode_ = InputMode::kSelecting;
    AppState app_state_ = AppState::kIdle;
    GameMode game_mode_ = GameMode::kSinglePathfinding;
    ParallelStrategy parallel_strategy_ = ParallelStrategy::kOpenMP;
    Algorithm algorithm_ = Algorithm::kAStar;
    bool is_dragging_ = false;
    bool show_pathfinding_panel_ = true;

    static constexpr int kMaxMultiAgents = 10000;
    /// Above this, agents are not pre-allocated (fast Reset); built on Run search.
    static constexpr int kEagerMultiAgentBuildLimit = 2048;
    int multi_agent_count_ = 2;
    bool record_search_snapshots_ = true;

    /// A*, Greedy, BA*: optional 1.001 tie-break on equal Manhattan; off uses pure Manhattan.
    bool heuristic_tie_nudge_ = true;
    /// Integer multiplier on h in f = g + h (1 = standard A*; >1 weighted / greedier).
    int heuristic_weight_ = 1;

};      


#endif  // PATHFINDING_GAME_H