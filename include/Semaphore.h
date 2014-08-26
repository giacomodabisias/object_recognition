#include <mutex>
#include <condition_variable>

class Semaphore
{
private:
    std::mutex mutex_;
    std::condition_variable threads_condition_;
    std::condition_variable main_condition_;
    int count_;
    int max_count_;
    bool stop_;


public:
    Semaphore(int max_count): count_(0), max_count_(max_count), stop_(false){}

	void Notify2main()
	{
	    std::unique_lock<std::mutex> lock(mutex_);
	    if(++count_ == max_count_){
	    	threads_condition_.notify_all();
	    	count_ = 0;
	    }
	}

	void Wait4threads()
	{
	    std::unique_lock<std::mutex> lock(mutex_);
	    if(count_ < max_count_ )
	        threads_condition_.wait(lock);
	}

	void Notify2threads()
	{
		std::unique_lock<std::mutex> lock(mutex_);
		main_condition_.notify_all();

	}

	void Wait4main()
	{
		std::unique_lock<std::mutex> lock(mutex_);
		main_condition_.wait(lock);
	}

	void SetStop()
	{
		std::unique_lock<std::mutex> lock(mutex_);
		stop_ = !stop_;
	}

	bool ToStop()
	{
		std::unique_lock<std::mutex> lock(mutex_);
		return stop_;
	}

};