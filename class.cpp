#include <iostream>
#include <cmath>
using namespace std;

class Server
{
public:
    void Rob();
    void Graf();
    void Marshrut();
};
class Control
{
public:
    void rigth();
    void left();
    void back();
    void straight();
};
class Robot
{
private:
    int x;
    int y;
public:
    void start();
    void stop();
    void clean();
};
class Graffity
{
private:
    int x1;
    int x2;
};
class Camera
{
public:
    void Rob(Robot* p);
    void Graf(Graffity* q);
};
int main()
{

}

