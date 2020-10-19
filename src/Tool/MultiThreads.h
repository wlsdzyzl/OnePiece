#ifndef MULTI_THREADS_H
#define MULTI_THREADS_H
#include <thread>
//#include <functional>
//f(x) = y => f(X) = Y
#define MAX_THREADS 100
namespace one_piece
{
namespace tool
{

    template <class T1, class T2>
    void MultiThreads(const std::vector<T1> &p1, std::vector<T2> &p2, std::function<void(const T1 &, T2&)> &f)
    {
        if(p1.size() != p2.size())
        {
            std::cout <<RED<<"[MultiThread]::[ERROR]::The number of parameters is not equal to the number of return values."<<RESET<<std::endl;
            return;
        }
        std::vector<std::thread> threads;
        if(p1.size() <= MAX_THREADS)
        {
            for(size_t i = 0; i!= p1.size(); ++i)
            {
                threads.push_back(std::thread(f,std::ref(p1[i]),std::ref(p2[i])));
            }
            for(size_t i = 0; i!= p1.size(); ++i)
            {
                threads[i].join();
            }
        }
        else
        {
            //std::cout<<"cube size: "<<p1.size()<<std::endl;  
            threads.resize(MAX_THREADS);
            int max_iteration_time = p1.size() / MAX_THREADS;
            for(int iter = 0; iter <= max_iteration_time; ++iter)
            {
                size_t j = iter *MAX_THREADS;
                for(size_t i = 0; j< p1.size() && i < MAX_THREADS; ++j, ++i)
                {
                    threads[i] = std::thread(f, std::ref(p1[j]),std::ref(p2[j]));
                }
                //std::cout<<j<<std::endl;       
                for(size_t i = 0; i < MAX_THREADS; ++i)
                {
                    if(threads[i].joinable())
                    threads[i].join();
                }       
  
                if(j >= p1.size()) break;
            }
        }
    }

}
}
#endif