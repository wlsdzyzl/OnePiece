#ifndef TICK_TOCK_H
#define TICK_TOCK_H
#include <chrono>
#include <map>
#include "ConsoleColor.h"
namespace fucking_cool
{
namespace tool
{
    class Duration
    {
        public:
        void TICK()
        {
            start_time = std::chrono::steady_clock::now();
        }
        void TOCK()
        {
            end_time = std::chrono::steady_clock::now();
        }
        float Elapsed()
        {
            std::chrono::duration<float> diff = end_time-start_time;
            if(diff.count() < 0)
            {
                std::cout<<YELLOW<<"[TICKTOCK]::[WARNING]::Elased time is less than 0." <<RESET<<std::endl;
            }
            return diff.count()*1000;
        }

        std::chrono::steady_clock::time_point start_time;
        std::chrono::steady_clock::time_point end_time;
        
    };

    class Timer
    {
        public:
        void TICK(const std::string &name)
        {
            durations[name].TICK();
        }
        void TOCK(const std::string &name)
        {
            if(durations.find(name) == durations.end())
            {
                std::cout<<YELLOW<<"[TICKTOCK]::[WARNING]::TOCK without TICK!" <<RESET<<std::endl;
                return;
            }
            durations[name].TOCK();
        }
        float Elapsed(const std::string &name)
        {
            return durations[name].Elapsed();
        }

        void Log(const std::string &name)
        {
            std::cout<<name+"::"<<durations[name].Elapsed()<<"ms" <<std::endl;
        }

        void LogAll()
        {
            for(auto i = durations.begin(); i!= durations.end(); ++i)
            {
                Log(i->first);
            }
        }

        void Reset()
        {
            durations.clear();
        }
        std::map<std::string, Duration> durations;
        
    };
}
}
#endif
