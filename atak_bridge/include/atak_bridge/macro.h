#pragma once

#include <functional>
#include <mutex>

// Google C++ coding convention uses this macro to disable the implicitly
// defined copy-constructor and assignment-operator in classes.
#define DISALLOW_COPY_AND_ASSIGN(TypeName)	\
   TypeName(const TypeName&) = delete;	\
   void operator=(const TypeName&) = delete

#define MBR_FN(FCN) \
	std::bind(FCN, this)

#define MBR_CB(FCN) \
   std::bind(FCN, this, std::placeholders::_1)

#define MBR_TF(FCN) \
   std::bind(FCN, this, std::placeholders::_1, std::placeholders::_2)

#define MBR_TP(FCN,P) \
	std::bind(FCN, this, std::placeholders::_1, P)

#define ULOCK(mtx) \
   std::unique_lock<std::mutex> lock(mtx)

#define RULOCK(mtx) \
   std::unique_lock<std::recursive_mutex> lock(mtx)

