#include <string>
#include <vector>
#include <random>
namespace fucking_cool
{
namespace tool
{
    //to split a string, or reversely split a string
    std::vector<std::string> Split(const std::string &str, const std::string &delim, int split_times = -1);
    std::vector<std::string> RSplit(const std::string &str, const std::string &delim, int split_times = -1);
    template <typename T = int>
        void ShuffleVector(std::vector<T> &v)
    {
        if(v.size() == 0) return;
        std::random_device rd;
        std::default_random_engine e(rd());
        std::uniform_int_distribution<int> u(0, v.size() - 1);

        for(int i = 0; i != v.size(); ++i)
        {
            std::swap<T>(v[i], v[u(e)]);
        }
    }
}
}