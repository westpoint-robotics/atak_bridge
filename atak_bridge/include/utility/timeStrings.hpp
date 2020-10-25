
#pragma once

#include <boost/date_time/posix_time/posix_time.hpp>

namespace Utility {

inline std::wstring CurrentISOTimeString()
{
	namespace bpt = boost::posix_time;

	bpt::wtime_facet  *l_pTimeFacet  = new bpt::wtime_facet(L"%Y-%m-%dT%H:%M:%S");
	bpt::ptime         l_ptTimeStamp = bpt::second_clock::local_time();
	std::wstringstream wss;

	wss.imbue(std::locale(std::locale(""), l_pTimeFacet));
	wss << l_ptTimeStamp << std::flush;
	return(wss.str());
};

inline std::wstring CurrentISOTimeString_uS()
{
	namespace bpt = boost::posix_time;

	bpt::wtime_facet  *l_pTimeFacet  = new bpt::wtime_facet(L"%Y-%m-%dT%H:%M:%S.%f%Q");
	bpt::ptime         l_ptTimeStamp = bpt::microsec_clock::local_time();
	std::wstringstream wss;

	wss.imbue(std::locale(std::locale(""), l_pTimeFacet));
	wss << l_ptTimeStamp << std::flush;
	return(wss.str());
};

/// convert a ptime to a ISO timestring compatible with xs:datetime formatting
inline std::wstring ISOTimeString(const boost::posix_time::ptime& l_ptTimeStamp) {
	namespace bpt = boost::posix_time;

	bpt::wtime_facet* l_pTimeFacet = new bpt::wtime_facet(L"%Y-%m-%dT%H:%M:%S");
	std::wstringstream wss;

	wss.imbue(std::locale(std::locale(""), l_pTimeFacet));
	wss << l_ptTimeStamp << std::flush;
	return(wss.str());

}

/// convert a ptime in UTC to a ISO timestring compatible with xs:datetime formatting
inline std::wstring ISOTimeStringZ(const boost::posix_time::ptime& l_ptTimeStamp) {
	namespace bpt = boost::posix_time;

	bpt::wtime_facet* l_pTimeFacet = new bpt::wtime_facet(L"%Y-%m-%dT%H:%M:%SZ");
	std::wstringstream wss;

	wss.imbue(std::locale(std::locale(""), l_pTimeFacet));
	wss << l_ptTimeStamp << std::flush;
	return(wss.str());

}

}
