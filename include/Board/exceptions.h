#ifndef exceptions_h
#define exceptions_h

#include <exception>
#include <string>

using std::exception;
using std::string;


namespace controller{


class FailedOpeningPortException: public exception{
public:
    FailedOpeningPortException(string description);
    virtual const char* what() const throw();

private:
    string _desc;
};


class FailedChangingBaudRateException: public exception{
public:
    FailedChangingBaudRateException(string description);
    virtual const char* what() const throw();

private:
    string _desc;
};


}

#endif
