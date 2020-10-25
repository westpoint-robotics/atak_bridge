#pragma once

#include <stdlib.h>
#include <xercesc/util/Xerces_autoconf_config.hpp>
#include <xercesc/util/XMLString.hpp>
#include <string>
#include <locale>

namespace Utility {

// This class encapsulates the conversion from wide char (wchar_t)
//  to a xerces string pointer.  It performs 3 functions
//  1 - reduce code necessary for conversion
//  2 - automatic clean up of memory
//  3 - abstraction of operations necessary for conversion

XERCES_CPP_NAMESPACE_USE;
class xStr
{
private:
	XMLCh *xmlptr;

public:
	operator const XMLCh *() const
	{
		return(xmlptr);
	}

public:
	// destruction
	~xStr()
	{
		if (xmlptr)
			XMLString::release(&xmlptr);
	}

	// construction
	xStr()
	{
		xmlptr = NULL;
	}

	// construction from wide character array
	xStr(const wchar_t *wstr0)
	{
		std::wstring wstr(wstr0);
		size_t       maxL = 1 + wstr.length();
		char        *str0 = new char[maxL];

		// clear the ansi string
		memset(str0, 0, maxL);
#ifdef _WIN32
#pragma warning(disable:4996)
#endif
		size_t       converted = wcstombs(str0, wstr0, maxL - 1);
#ifdef _WIN32
#pragma warning(default:4996)
#endif
		if (converted == (size_t)-1)
			xmlptr = XMLString::transcode("!ERROR!");
		else
			xmlptr = XMLString::transcode(str0);
		delete[] str0;
	}

	// construction from ansi character array
	xStr(const char *str)
	{
		xmlptr = XMLString::transcode(str);
	}

	// C++11 Delegate constructor
	xStr(const std::string &str)
		: xStr(str.c_str()) {}

	// C++11 Delegate constructor
	xStr(const std::wstring &str)
		: xStr(str.c_str()) {}

	// copy constructor
	xStr(const xStr &src)
	{
	  xmlptr = NULL;
	  if (src.xmlptr)
		xmlptr = XMLString::replicate(src.xmlptr);
	}
 
	// assignment from xStr
	const xStr &operator =(const xStr &src)
	{
	  if (xmlptr)
		XMLString::release(&xmlptr);

	  xmlptr = NULL;

	  if (src.xmlptr)
		xmlptr = XMLString::replicate(src.xmlptr);
	  return(*this);
	}
};

// This class performs the inverse of the above, converting a
//  a xerces string pointer to a wide string.
class aStr
{
  private:
	char *ansiptr;

  public:
	operator std::wstring () const
	{
	  if (ansiptr == NULL)
		return(std::wstring(L""));

	  std::string  str(ansiptr);
	  std::wstring wstr(str.begin(), str.end());
	  return(wstr);
	}

  public:
	// destruction
   ~aStr()
	{
	  if (ansiptr)
		XMLString::release(&ansiptr);
	}

	// construction
	aStr()
	{
	  ansiptr = NULL;
	}

	// construction from XMLCh array pointer
	aStr(const XMLCh *str)
	{
	  ansiptr = XMLString::transcode(str);
	}

	// copy constructor
	aStr(const aStr &src)
	{
	  ansiptr = NULL;
	  if (src.ansiptr)
		ansiptr = XMLString::replicate(src.ansiptr);
	}
 
	// assignment from aStr
	const aStr &operator =(const aStr &src)
	{
	  if (ansiptr)
		XMLString::release(&ansiptr);

	  ansiptr = NULL;

	  if (src.ansiptr)
		ansiptr = XMLString::replicate(src.ansiptr);
	  return(*this);
	}
};

}

