#include "CppExtension.h"
#include <algorithm>
namespace fucking_cool
{
namespace tool
{
    std::vector<std::string> Split(const std::string &str, const std::string &delim, int split_times)
    {
        size_t start = 0;
        std::vector<std::string> results;
        int current_split_time = 0;
        if(split_times == 0)
        {
            results.push_back(str);
            return results;
        }
        while(start < str.size())
        {
            size_t new_start = str.find(delim, start);
            
            if(new_start != std::string::npos)
            {
                current_split_time += 1;
                if(new_start - start > 0)
                    results.push_back(str.substr(start, new_start));
                start = new_start + delim.size();
                
            }
            else break; 
            if(split_times > 0 && current_split_time >= split_times )
            break;
        }
        if( start < str.size())
        results.push_back(str.substr(start, str.size()));
        return results;
    }
    std::vector<std::string> RSplit(const std::string &str, const std::string &delim, int split_times)
    {
        size_t end = str.size();
        std::vector<std::string> results;
        int current_split_time = 0;
        if(split_times == 0)
        {
            results.push_back(str);
            return results;
        }
        while(end > 0)
        {
            
            size_t new_end = str.rfind(delim, end);
            if(new_end != std::string::npos)
            {
                current_split_time += 1;
                if(end - new_end - delim.size() > 0)
                    results.push_back(str.substr(new_end + delim.size(), end));
                end = new_end;
            }
            else break; 
            if(split_times > 0 && current_split_time >= split_times)
            break;
        }
        if( end > 0)
        results.push_back(str.substr(0, end));

        std::reverse(results.begin(), results.end());
        return results;
    }
}
}