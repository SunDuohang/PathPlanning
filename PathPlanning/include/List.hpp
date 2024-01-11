/**
  ******************************************************************************
  * @file           : List.hpp
  * @author         : SUNX
  * @brief          : None
  * @attention      : None
  * @date           : 2023/12/20
  * @last_change    : 
  ******************************************************************************
  */



#ifndef PATHPLANNING_LIST_HPP
#define PATHPLANNING_LIST_HPP

#include<iostream>
#include<string.h>
#include <vector>
#include "Elem.hpp"


class List
{
public:
	List() = default;
	List(const List& list, bool isRestruct = true)
	{
		if (isRestruct)
		{
			for (int i = 0; i < list.data.size(); i++)
			{
				this->data.push_back(list.data[i]);
			}
			return;
		}
		this->data.push_back(list);
	}
	List(const std::string& str)
	{
		for (int i = 0; i < str.size(); i++)
		{
			this->data.push_back((unsigned char)str[i]);
		}
	}



	/* 其实现必须在头文件,否则会找不到符号 */
	template<typename... Arg>
	List(Arg... args)
	{
		append(args...);
	}

	template<typename T>
	void append(T value)
	{
		data.push_back(value);
	}


	template <typename Head, typename... Rail>
	void append(Head head, Rail... last)
	{
		data.push_back(head);
		append(last...);
	}


	List operator+(const int i)
	{
		this->data.push_back(i);
		return this;
	}
	List operator+(const List& list)
	{
		List tempList;
		for (int i = 0; i < this->data.size(); i++)
		{
			tempList.data.push_back(this->data[i]);
		}
		for (int i = 0; i < list.data.size(); i++)
		{
			tempList.data.push_back(list.data[i]);
		}
		return tempList;
	}

	List operator=(const List& list)
	{
		this->data.clear();
		for (int i = 0; i < list.data.size(); i++)
		{
			this->data.push_back(list.data[i]);
		}
		return this;
	}


	List operator+=(const std::string& str)
	{
		for (int i = 0; i < str.size(); i++)
		{
			this->data.push_back(str[i]);
		}
		return this;
	}
	List operator+=(const List& list)
	{
		for (int i = 0; i < list.data.size(); i++)
		{
			this->data.push_back(list.data[i]);
		}
		return this;
	}
	List operator+=(unsigned char c)
	{
		this->data.push_back(c);
		return this;
	}

	Elem& operator[](int index)
	{
		return data[index];
	}

	void set(int index, Elem a)
	{
		this->data[index] = a;
	}
	Elem get(int index)
	{
		return this->data[index];
	}

	void foreach(void(*callback)(Elem&))
	{
		for (int i = 0; i < this->data.size(); i++)
		{
			callback(this->data[i]);
		}
	}

	std::string toByteString()
	{
		std::string str;
		for (int i = 0; i < this->data.size(); i++)
		{
			str += this->data[i].toByte();
		}
		if (str.size() != this->data.size()) {
            std::logic_error ex(("List to string failed,"+ std::to_string(this->data.size() - str.size())+" byte(s) are missing").c_str());
            throw std::exception(ex);
        }
		return str;
	}
	int size()
	{
		return this->data.size();
	}

	bool empty()
	{
		return this->data.size() == 0;
	}
	bool exist(int index)
	{
		if (this->data.size() >= index && !this->data[index].empty())
		{
			return true;
		}
		return false;
	}
	static void fromJson(std::string jsonValue) {

	}
//    std::string toIntArray()
//	{
//        std::string intArray;
//		for (auto i = 0; i < this->data.size() - 1; i++)
//		{
//			intArray += std::to_string((unsigned int)data[i].toByte()) + ", ";
//		}
//		intArray += std::to_string((unsigned int)data[this->data.size() - 1].toByte());
//		return intArray;
//	}
    std::string toStringArray()
	{
        std::string array = "[";
		for (auto i = 0; i < this->data.size(); i++)
		{
			array += data[i].toString()+ (i==(this->data.size()-1)? "":",");
		}
		array += "]";
		return array;
	}

    int* toIntArray(){
        int size = this->data.size();
        int *arr = new int [size];
        int *p = arr;
        for (Elem i: data) {
            *p++ = i.toInt16();
        }
        return arr;
    }

    std::vector<int> toIntVector(){
        std::vector<int> arr;
        for (Elem i: data)
            arr.push_back(i.toInt16());
        return arr;
    }
private:
    std::vector<Elem>  data;
};

#endif //PATHPLANNING_LIST_HPP
