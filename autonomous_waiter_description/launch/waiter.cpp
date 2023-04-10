#include <conio.h>
#include <iostream>
using namespace std;

int main(int argc, char *argv[])
{
    int c = 0;
    while (true)
    {
        switch ((c = getch()))
        {
        case 48:
            system("ros2 param set /cmd_vel_unstamped 0");
            cout << endl
                 << "0" << endl; // key 0
            break;
        case 49:
            system("ros2 param set /cmd_vel_unstamped 1");
            cout << endl
                 << "1" << endl; // key 1
            break;
        case 50:
            system("ros2 param set /cmd_vel_unstamped 2");
            cout << endl
                 << "2" << endl; // key 2
            break;
        case 51:
            system("ros2 param set /cmd_vel_unstamped 3");
            cout << endl
                 << "3" << endl; // key 3
            break;
        case 52:
            system("ros2 param set /cmd_vel_unstamped 4");
            cout << endl
                 << "4" << endl; // key 4
            break;
        case 53:
            system("ros2 param set /cmd_vel_unstamped 5");
            cout << endl
                 << "5" << endl; // key 5
            break;
        case 54:
            system("ros2 param set /cmd_vel_unstamped 6");
            cout << endl
                 << "6" << endl; // key 6
            break;
        case 55:
            system("ros2 param set /cmd_vel_unstamped 7");
            cout << endl
                 << "7" << endl; // key 7
            break;
        case 56:
            system("ros2 param set /cmd_vel_unstamped 8");
            cout << endl
                 << "8" << endl; // key 8
            break;
        case 57:
            system("ros2 param set /cmd_vel_unstamped 9");
            cout << endl
                 << "9" << endl; // key 9
            break;
        default:
            system("ros2 param set /cmd_vel_unstamped 0");
            cout << endl
                 << "None: Defaulting to 0" << endl; // not number
            break;
        }
    }

    return 0;
}