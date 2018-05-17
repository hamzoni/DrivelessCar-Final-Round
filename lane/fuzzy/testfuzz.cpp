#include "realangle.h"
#include "iostream"
#include "fuzzylogic.h"
#include "getparam.h"


using namespace std;
int main(){
    init("/home/ubuntu/Desktop/DriverlessCarChallenge");
    int isNgaBa = 0;
    int isObj = 0;
    double angle = 25;
    cout << fuzzy(angle, isNgaBa, isObj) << endl;
}
