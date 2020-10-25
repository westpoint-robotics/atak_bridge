#pragma once
#include "macro.h"

#include <xercesc/dom/DOM.hpp>
#include <xercesc/util/PlatformUtils.hpp>

namespace Messaging {

	using namespace std;

	class XmlMessagingBase {
	public:

		using DOMDocument = XERCES_CPP_NAMESPACE::DOMDocument;
		static DOMDocument* createDocument() {
			initialize();
			return pImplementation->createDocument();
		}

		using DOMLSSerializer = XERCES_CPP_NAMESPACE::DOMLSSerializer;
		static DOMLSSerializer* createLSSerializer() {
			initialize();
			return pImplementationLS->createLSSerializer();
		}

		using DOMLSOutput = XERCES_CPP_NAMESPACE::DOMLSOutput;
		static DOMLSOutput* createLSOutput() {
			initialize(); 
			return pImplementationLS->createLSOutput();
		}

		using DOMLSParser = XERCES_CPP_NAMESPACE::DOMLSParser;
		static DOMLSParser* createLSParser() {
			initialize();
			return pImplementationLS->createLSParser(DOMImplementationLS::DOMImplementationLSMode::MODE_SYNCHRONOUS, nullptr);
		}

		using DOMLSInput = XERCES_CPP_NAMESPACE::DOMLSInput;
		static DOMLSInput* createLSInput() {
			initialize();
			return pImplementationLS->createLSInput();
		}

		using XMLString = XERCES_CPP_NAMESPACE::XMLString;
	protected:
		using DOMImplementationLS = XERCES_CPP_NAMESPACE::DOMImplementationLS;
		using DOMImplementation = XERCES_CPP_NAMESPACE::DOMImplementation;

		XmlMessagingBase() {
			initialize();
		}

		static bool initialized() { return pImplementationLS != nullptr; }

		static void initialize() {
			if (!initialized()) {
				XERCES_CPP_NAMESPACE::XMLPlatformUtils::Initialize();
				XMLCh *strLS = XMLString::transcode("LS");
				XMLCh *strCore = XMLString::transcode("Core");
				pImplementationLS = XERCES_CPP_NAMESPACE::DOMImplementationRegistry::getDOMImplementation(strLS);
				pImplementation = XERCES_CPP_NAMESPACE::DOMImplementationRegistry::getDOMImplementation(strCore);
				XMLString::release(&strLS);
				XMLString::release(&strCore);
				atexit(&XmlMessagingBase::shutdown);
			}
		}

		virtual ~XmlMessagingBase() {
		}

		static DOMImplementationLS* pImplementationLS;

		static DOMImplementation* pImplementation;

		static void shutdown() {
			XERCES_CPP_NAMESPACE::XMLPlatformUtils::Terminate();
		}

	private:

		friend class DirectoryServer;
		friend class DirectoryServerFactory;

		DISALLOW_COPY_AND_ASSIGN(XmlMessagingBase);
	};
}

