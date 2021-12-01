#include "krembot.ino.h"

// define states enum for later use
enum State
{
    turn,
    move
};

State state;
SandTimer sandTimer;
int turningTime;
float turningDistance;
float turnRight;

void ex2_controller::setup()
{
    krembot.setup();
    // this is the initial turning distance. In some simulations we've used different values
    turningDistance = 18;
    // the krembot starts at move state
    state = State::move;
}

void ex2_controller::loop()
{
    krembot.loop();

    // act based on the current state
    switch (state)
    {
    case State::move:
    {
        float distance = krembot.RgbaFront.readRGBA().Distance;
        // turn if you're too close to an obstacle or spontaniously in rare cases
        // the spontanious turning was used only in some cases (which we've mentioned in the report)
        if ((distance < turningDistance) or ((rand() % 1000) < 1))
        {
            // set turning time
            if (rand() % 11 < 8)                   // in 80% of the cases
                turningTime = (rand() % 40) + 180; // the turning distance is in [180, 220]
            else
                turningTime = (rand() % 200) + 100; // turning distance in [100, 300] - wider range

            // turn right at 20% of the cases
            turnRight = rand() % 11;
            if (turnRight > 8)
                turnRight = 1;
            else
                turnRight = -1;

            // change state to turn
            state = State::turn;
            sandTimer.start(turningTime);
            krembot.Led.write(255, 0, 0);
        }
        else
        {
            krembot.Base.drive(100, 0);
        }
        break;
    }

    case State::turn:
    {
        if (sandTimer.finished())
        {
            // if time is up, change state to move and set the distance for the following step
            state = State::move;
            krembot.Led.write(0, 255, 0);
            // to add some randomness into our simulation, the robot turns at a range of [17.5, 18.5] away from an obstacle
            // this wasn't used in most of the simulations (we've mentioned in the report where it was used)
            turningDistance = 18 + (1 - (rand() % 11) / 10);
        }
        else
        {
            krembot.Base.drive(0, turnRight * 100);
        }
        break;
    }
    }
}
