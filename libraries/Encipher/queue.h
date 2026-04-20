#ifndef __QUEUE_H_
#define __QUEUE_H_

template < typename T >
class Queue
{
public:
    virtual void add(const T& e) = 0;
    virtual bool remove() = 0;
    virtual T front(bool &s) const = 0;
    virtual void clear() = 0;
    virtual int length() const = 0;
};

#endif // __QUEUE_H_
