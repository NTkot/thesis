#ifndef MovingAverage_HPP
#define MovingAverage_HPP

#include <vector>
#include <numeric>

template <typename T>
class MovingAverage 
{
    private:
        std::vector<T> _window_elements;

    public:
        MovingAverage(int window_size) : _window_elements(window_size) {};

        float value() {return ((float)std::accumulate(this->_window_elements.begin(),
                                                      this->_window_elements.end(), 0)) / this->size();};

        int size() {return this->_window_elements.size();};

        float insert(T element)
        {
            std::move(std::begin(this->_window_elements) + 1, 
                      std::end(this->_window_elements), 
                      std::begin(this->_window_elements));

            this->_window_elements.back() = element;

            return this->value();
        };
};

#endif