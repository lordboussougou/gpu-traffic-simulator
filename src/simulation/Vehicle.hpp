#pragma once

struct Vehicle
{
    int id = 0;
    int currentEdgeId = 0;

    float position = 0.0f;          // Distance parcourue le long de la route, en mètres.
    float speed = 0.0f;             // m/s   
    float acceleration = 0.0f;      // m/s²

    int lane = 0;
    float desiredSpeed = 0.0f;      // m/s
};