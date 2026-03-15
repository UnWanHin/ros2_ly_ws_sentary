#pragma once

#include <vector>
#include <deque>
#include <controller/controller.hpp>

namespace CONTROLLER
{
    class BoardSelector
    {
    public:
        BoardSelector();
        ~BoardSelector();

        BoardInformation selectBestBoard(BoardInformations &board_info);

    private:
    };
} // namespace CONTROLLER