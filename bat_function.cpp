//Markus Buchholz

#include <iostream>
#include <vector>
#include <tuple>
#include <algorithm>
#include <math.h>
#include <random>

//--------Path Planner--------------------------------------------------------------

float xmin = -5.0; // 0.0;
float xmax = 5.0;  // 50.0;
float ymin = -5.0; // 0.0;
float ymax = 5.0;  // 50.0;

float obsX = 25.0;
float obsY = 25.0;
float obsR = 5.0;

float goalX = 45.0;
float goalY = 45.0;

float startX = 2.0;
float startY = 2.0;

//----------------------------------------------------------------------------------
int EVOLUTIONS = 100;
int BATS = 200;

float Rmin = 0.0; // puls rate
float Rmax = 1.0;

float Amin = 1.0; // loundness rate
float Amax = 2.0;

float Fmin = 0.0; // frequency rate
float Fmax = 2.0;

float WALKSTEP = 0.001;
float ALPHA = 0.9;
float GAMMA = 0.9;

//--------------------------------------------------------------------------------

struct Pos
{

    float x;
    float y;
};
//--------------------------------------------------------------------------------

struct Velo
{

    float x;
    float y;
};
//--------------------------------------------------------------------------------

float generateNormalRandom()
{

    std::random_device engine;
    std::mt19937 gen(engine());
    std::normal_distribution<float> distrib(-1.0, 1.0);
    return distrib(gen);
}

//--------------------------------------------------------------------------------

float euclid(Pos a, Pos b)
{

    return std::sqrt(std::pow(a.x - b.x, 2) + std::pow(a.y - b.y, 2));
}
//--------------------------------------------------------------------------------

float generateRandom()
{

    std::random_device engine;
    std::uniform_real_distribution<float> distrib(0, 1.0);
    return distrib(engine);
}

//--------------------------------------------------------------------------------
float valueGenerator(float low, float high)
{

    return low + generateRandom() * (high - low);
}

//--------------------------------------------------------------------------------

std::vector<Pos> initPosXY()
{

    std::vector<Pos> pos;

    for (int ii = 0; ii < BATS; ii++)
    {

        pos.push_back({valueGenerator(xmin, xmax), valueGenerator(ymin, ymax)});
    }

    return pos;
}
//--------------------------------------------------------------------------------
std::vector<float> initPulsRate()
{

    std::vector<float> pulse;

    for (int ii = 0; ii < BATS; ii++)
    {

        pulse.push_back({valueGenerator(Rmin, Rmax)});
    }


    return pulse;
}

//--------------------------------------------------------------------------------
std::vector<float> initLoudness()
{

    std::vector<float> loud;

    for (int ii = 0; ii < BATS; ii++)
    {

        loud.push_back({valueGenerator(Amin, Amax)});
    }

    return loud;
}

//--------------------------------------------------------------------------------
std::vector<float> initFrequency()
{

    std::vector<float> freq;

    for (int ii = 0; ii < BATS; ii++)
    {

        freq.push_back({valueGenerator(Fmin, Fmax)});
    }

    return freq;
}

//--------------------------------------------------------------------------------

std::vector<Velo> initVelocity()
{

    std::vector<Velo> velo;

    for (int ii = 0; ii < BATS; ii++)
    {

        velo.push_back({generateRandom(), generateRandom()});
    }

    return velo;
}

//--------------------------------------------------------------------------------

std::vector<float> function(std::vector<Pos> pos)
{
    std::vector<float> funcValue;

    for (auto &ii : pos)
    {

        funcValue.push_back(ii.x * ii.y);
    }

    return funcValue;
}

//--------------------------------------------------------------------------------

float func(Pos pos)
{

    return pos.x * pos.y;
}

//--------------------------------------------------------------------------------

Pos positionUpdateCheck(Pos actualPos)
{

    Pos Pnew = actualPos;

    if (Pnew.x < xmin)
    {
        Pnew.x = xmin;
    }

    if (Pnew.x > xmax)
    {
        Pnew.x = xmax;
    }

    if (Pnew.y < ymin)
    {
        Pnew.y = ymin;
    }

    if (Pnew.y > ymax)
    {
        Pnew.y = ymax;
    }

    return Pnew;
}

//-------------------------------------------------------------------------------
bool compareMin(std::pair<Pos, float> a, std::pair<Pos, float> b)
{

    return a.second < b.second;
}

//-------------------------------------------------------------------------------

// min
std::tuple<Pos, float> findBestPosFuncValue(std::vector<Pos> positions, std::vector<float> func)
{

    std::vector<std::pair<Pos, float>> best;

    for (int ii = 0; ii < func.size(); ii++)
    {

        best.push_back(std::pair<Pos, float>(positions[ii], func[ii]));
    }

    std::sort(best.begin(), best.end(), compareMin);

    return best[0];
}

//--------------------------------------------------------------------------------

float updateFreq()
{

    return Fmin + (Fmax - Fmin) * generateRandom();
}

//--------------------------------------------------------------------------------

Velo updateVelo(Pos pos, Pos best, Velo velo, float freq)
{

    Velo veloNew;

    veloNew.x = velo.x + (pos.x - best.x) * freq;
    veloNew.y = velo.y + (pos.y - best.y) * freq;

    return veloNew;
}

//--------------------------------------------------------------------------------

Pos updatePos(Pos pos, Velo velo)
{

    Pos posNew;

    posNew.x = pos.x + velo.x;
    posNew.y = pos.y + velo.y;

    return positionUpdateCheck(posNew);
}

//--------------------------------------------------------------------------------

Pos updatePosWalk(Pos best)
{

    Pos posNew;

    posNew.x = best.x + WALKSTEP * generateRandom();
    posNew.y = best.y + WALKSTEP * generateRandom();

    return positionUpdateCheck(posNew);
}
//--------------------------------------------------------------------------------




void runBat()
{

    std::vector<Pos> currentPositions = initPosXY();
    std::vector<float> currentValueFunction = function(currentPositions);

    std::vector<Velo> currentVelo = initVelocity();
    std::vector<float> currentFreq = initFrequency();
    std::vector<float> currentPulse = initPulsRate();
    std::vector<float> currentLoud = initLoudness();

    for (int ii = 0; ii < EVOLUTIONS; ii++)
    {

        std::tuple<Pos, float> bestPosFuncValue = findBestPosFuncValue(currentPositions, currentValueFunction);
        Pos bestPos = std::get<0>(bestPosFuncValue);
        float bestFuncValue = std::get<1>(bestPosFuncValue);

        for (int jj = 0; jj < BATS; jj++)
        {

            currentFreq[jj] = updateFreq();
            currentVelo[jj] = updateVelo(currentPositions[jj], bestPos, currentVelo[jj], currentFreq[jj]);
            currentPositions[jj] = updatePos(currentPositions[jj], currentVelo[jj]);

            if (currentPulse[jj] < generateRandom())
            {

                currentPositions[jj] = updatePosWalk(bestPos);

                currentValueFunction[jj] = func(currentPositions[jj]);
            }
            else
            {

                float newFuncValue = func(currentPositions[jj]);

                if ((newFuncValue < currentValueFunction[jj]) && currentLoud[jj] > generateRandom())
                {

                    currentValueFunction[jj] = newFuncValue;
                    currentLoud[jj] = ALPHA * currentLoud[jj];
                    currentPulse[jj] = currentPulse[jj] * (1 - std::exp(-GAMMA)); 
                }
            }
        }
    }

    for (auto &ii : currentValueFunction){
        std::cout << " f = " << ii << std::endl;
    }
}


//--------------------------------------------------------------------------------

int main()
{

    runBat();
}