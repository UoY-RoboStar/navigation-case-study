
#ifndef ROBOCALC_FUNCTIONS_H_
#define ROBOCALC_FUNCTIONS_H_

#include "DataTypes.h"
#include <vector>
#include <set>

namespace robocalc
{
	namespace functions
	{
		bool hasMaxReal(std::vector<double> a);
		double maxReal(std::vector<double> a);
		double real(unsigned int x);
		std::vector<> rotateSeq(std::vector<> s, int n);
		int round(double x);
		bool hasMinReal(std::vector<double> a);
		double minReal(std::vector<double> a);
		std::vector<double> rangeStep(double x, double y, double step);
		std::vector<> mapSeq( f, std::vector<> seq);
		std::vector<std::tuple<, >> zipSeqs(std::vector<> seq1, std::vector<> seq2);
		 foldSeq( f, std::vector<> seq,  init);
		
		template<typename T>
		std::set<T> set_union(std::set<T> s1, std::set<T> s2)
		{
			std::set<T> ret;
			ret.insert(s1.begin(), s1.end());
			ret.insert(s2.begin(), s2.end());
			return ret;
		}
		
		template<typename T>
		unsigned int size(std::set<T> s)
		{
			return s.size();
		}
	}
}

#endif
