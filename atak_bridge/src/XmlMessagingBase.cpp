#include "messaging/XmlMessagingBase.hpp"

namespace Messaging {
	XERCES_CPP_NAMESPACE::DOMImplementationLS* XmlMessagingBase::pImplementationLS = nullptr;
	XERCES_CPP_NAMESPACE::DOMImplementation* XmlMessagingBase::pImplementation = nullptr;
}
