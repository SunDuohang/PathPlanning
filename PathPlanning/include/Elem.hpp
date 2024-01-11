/**
  ******************************************************************************
  * @file           : Element.hpp
  * @author         : SUNX
  * @brief          : None
  * @attention      : None
  * @date           : 2023/12/20
  * @last_change    : 
  ******************************************************************************
  */



#ifndef PATHPLANNING_ELEM_HPP
#define PATHPLANNING_ELEM_HPP

#include <iostream>
#include <cstring>
#include <boost/any.hpp>
#include <typeinfo>
#include <vector>
#include <stdexcept>

union ListElement{
    unsigned char byte;
    unsigned short word;
    unsigned int dword;
    unsigned long long qword;
    float f;
    double d;
};

class Elem
{
public:
    Elem() = default;
    template<typename ValueType>
    Elem(const ValueType& value)
    {
        this->data = value;
    }

    Elem(const Elem& other)
    {
        this->data = other.data;
    }
    bool operator==(bool value)
    {
        return ((bool)toByte()) == value;
    }

    bool operator==(char value)
    {
        return toByte() == value;
    }
    bool operator==(unsigned char value)
    {
        return toByte() == value;
    }
    bool operator==(short value)
    {
        return toInt16() == value;
    }
    bool operator==(unsigned short value)
    {
        return toInt16() == value;
    }
    bool operator==(int value)
    {
        return toInt32() == value;
    }
    bool operator==(unsigned int value)
    {
        return toInt32() == value;
    }

    Elem operator=(const Elem& e)
    {
        this->data = e.data;
        return this->data;
    }

    /*static Elem operator+(const Elem& left ,const Elem& right)
    {
        if (left.type() == typeid(int))
            return this->data + e.toInt32();
    }*/

    unsigned char toByte()
    {
        ListElement e = this->convert();
        return e.byte;
    }
    short toInt16()
    {
        ListElement e = this->convert();
        return e.word;
    }
    int toInt32()
    {
        ListElement e = this->convert();
        return e.dword;
    }
    long long toInt64()
    {
        ListElement e = this->convert();
        return e.qword;
    }
    float toFloat()
    {
        ListElement e = this->convert();
        return e.f;
    }
    double toDouble()
    {
        ListElement e = this->convert();
        return e.d;
    }
    std::string toString()
    {
        if (this->isType<std::string>())
            return  "\""+this->cast<std::string>()+ "\"";
        else if (this->isType<const char*>())
            return  "\""+std::string(this->cast<const char*>())+ "\"";
        else if (this->isType<char*>())
            return "\""+std::string(this->cast<char*>())+"\"";
        else if (this->isType<bool>())
            return this->cast<bool>() ? "true" : "false";
        else if (this->isType<char>() || this->isType<unsigned char>())
            return std::to_string(this->toByte());
        else if (this->isType<short>() || this->isType<unsigned short>())
            return std::to_string(this->toInt16());
        else if (this->isType<int>() || this->isType<unsigned int>())
            return std::to_string(this->toInt32());
        else if (this->isType<long>() || this->isType<unsigned long>())
            return std::to_string(this->toInt32());
        else if (this->isType<long long>() || this->isType<unsigned long long>())
            return std::to_string(this->toInt64());
        else if (this->isType<float>())
            return std::to_string(this->toFloat());
        else if (this->isType<double>())
            return std::to_string(this->toDouble());
        else
            return std::string("undefined");
    }

    template<class  T>
    T cast() {
        if (this->data.type() != typeid(T)){
            std::logic_error ex(("data type:"+std::string(this->data.type().name())+" is not "+std::string(typeid(T).name())).c_str());
            throw std::exception(ex);
        }
        T result = boost::any_cast<T>(this->data);
        return result;

    }
    const boost::typeindex::type_info& type()
    {
        return this->data.type();
    }

    template<class T>
    bool isType()
    {
        if (this->empty()) return false;
        return this->data.type() == typeid(T);
    }

    bool empty() {
        return this->data.empty();
    }
private:
    ListElement convert()
    {
        ListElement e{ 0 };
        if (this->data.type() == typeid(unsigned char))
            e.byte = boost::any_cast<unsigned char>(this->data);

        else if (this->data.type() == typeid(char))
            e.byte = boost::any_cast<char>(this->data);

        else if (this->data.type() == typeid(short))
            e.word = boost::any_cast<short>(this->data);

        else if (this->data.type() == typeid(unsigned short))
            e.word = boost::any_cast<unsigned short>(this->data);

        else if (this->data.type() == typeid(int))
            e.dword = boost::any_cast<int>(this->data);

        else if (this->data.type() == typeid(unsigned int))
            e.dword = boost::any_cast<unsigned int>(this->data);

        else if (this->data.type() == typeid(long))
            e.dword = boost::any_cast<long>(this->data);

        else if (this->data.type() == typeid(unsigned long))
            e.dword = boost::any_cast<unsigned long>(this->data);

        else if (this->data.type() == typeid(long long))
            e.qword = boost::any_cast<long long>(this->data);

        else if (this->data.type() == typeid(unsigned long long))
            e.qword = boost::any_cast<unsigned long long>(this->data);

        else if (this->data.type() == typeid(float))
            e.f = boost::any_cast<float>(this->data);

        else if (this->data.type() == typeid(double))
            e.d = boost::any_cast<double>(this->data);
        else
        {
            std::string name = this->data.type().name();
            std::logic_error ex(("data type " + name + " is not supported").c_str());
            throw std::exception(ex);
        }
        return e;
    }
private:
    boost::any data;
};

#endif //PATHPLANNING_ELEM_HPP
