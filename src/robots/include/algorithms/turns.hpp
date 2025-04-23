#pragma once

    enum class TurnType {
        RIGHT, // Only right is open
        LEFT, // Only left is open
        LEFT_FRONT, //Left and Front are open
        RIGHT_FRONT, // Right and front are open
        CROSS, // Crossroad
        T_TURN // Left and Right are open 
    };
