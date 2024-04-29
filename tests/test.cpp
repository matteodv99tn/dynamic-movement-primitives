#include <iostream>
#include <vector>


using namespace std;

int
main() {
    vector<int> v;

    v.reserve(5);


    for (int i = 0; i < 10; i++) v.push_back(i);

    for (auto x : v) cout << x << endl;

    return 0;
}
