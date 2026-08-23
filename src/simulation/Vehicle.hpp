#pragma once

struct Vehicle
{
    int id = 0;

    // Distance parcourue le long de la route, en mètres.
    float position = 0.0f;

    // m/s
    float speed = 0.0f;

    // m/s²
    float acceleration = 0.0f;

    int lane = 0;

    // m/s
    float desiredSpeed = 0.0f;
};