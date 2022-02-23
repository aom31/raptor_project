#include <iostream>
#include <string>
#include <fstream>
#include <stdio.h>
#include <iostream>
#include <stdlib.h>
#include <string.h>
#include <sstream>

using namespace std;

int main(int argc, char* argv[] )
{
    ifstream data("/home/aom/raptor_testcode/src/path_reference/ref_data/path_data1.txt");
    double x,y;
    string strOne;


    cout << "x \t" << "y \t" << endl; 
    // while (data >> x >> y)
    // {
    //     cout << x << " \t" <<y <<"\t" << endl;
        

    // }
    while (getline(data,strOne))
    {
        stringstream ss;
        ss << strOne;
        ss >> x >>y;
        cout << x << "\t" << y << endl;
        
    }
    
    
    return 0;


}