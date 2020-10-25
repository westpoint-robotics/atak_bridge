#pragma once

#include <string>
#include <sstream>
#include <locale>

namespace Utility {

// This function 
	inline std::wstring quickS2W(const std::string &str) {
		std::wstringstream ss;
		ss << str.c_str();
		return(ss.str());
	}

	inline std::string quickW28(const std::wstring &str) {
		std::string s(str.begin(), str.end());
		return(s);
#if 0
  // Here is the proper answer, but <convert> is not not present on
  // all system types.

  typedef std::codecvt_utf8<wchar_t>          convert_type;
  std::wstring_convert<convert_type, wchar_t> converter;

  //use converter (.to_bytes: wstr->str, .from_bytes: str->wstr)
  std::string converted_str = converter.to_bytes( string_to_convert );
  return(converted_str);
#endif
	}

}
