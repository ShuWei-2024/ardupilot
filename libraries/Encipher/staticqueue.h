#ifndef __STATICQUEUE_H_
#define __STATICQUEUE_H_

#include "queue.h"

template < typename T, int N >
class StaticQueue:public Queue<T>
{
protected:
    T m_space[N];
    int m_front;
    int m_rear;
    int m_length;
public:
    StaticQueue()
    {
        m_front = 0;
        m_rear = 0;
        m_length = 0;
    }

    int capacity() const
    {
        return N;
    }

    void add(const T& e) override
    {
        m_space[m_rear] = e;
        m_rear = (m_rear + 1) % N;

        if(m_length >= N) {
            remove();
        }
        
        m_length++;
    }

    bool remove() override
    {
        if(m_length > 0) {
            m_front = (m_front + 1) % N;
            m_length--;

            return true;
        } else {
            return false;
        }
    }

    bool remove(int n)
    {
        bool ret = true;

        while (n-- && ret)
        {
            ret |= remove();
        }

        return ret;
    }

    T front(bool &s) const override
    {
        s = m_length > 0 ? true : false;

        return m_space[m_front];
    }

    // return 如果不是nullptr 要 delete[] 
    // T 若为自定义类型，必须对 = 进行重载
    T* front(int n) const
    {
        if(length() < n) {
            return nullptr;
        }

        T* ret = new T[n];

        for(int i=0; i<n; i++)
        {
            ret[i] = (*this)[i];
        }

        return ret;
    }

    T dequeue(bool &s)
    {
        T ret = front(s);

        if(s) {
            remove();            
        }

        return ret;
    }

    T dequeue()
    {
        bool s;
        return dequeue(s);
    }

    void clear() override
    {
        m_front = 0;
        m_rear = 0;
        m_length = 0;
    }

    int length() const override
    {
        return m_length;
    }

    // i 必须小于 length(), 否则返回值无意义
    T operator [](const int& i) const
    {
        return m_space[(i + m_front) % N];
    }
};

#endif // __STATICQUEUE_H_
