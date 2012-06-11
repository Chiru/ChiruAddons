/**
    For conditions of distribution and use, see copyright notice in LICENSE

    @file   CieMapFwd.h
    @brief  Forward declarations for commonly used CieMap types. */


#pragma once

// From RdfModule
class IMemoryStore;

/// The CieMap namespace contains the abstractions introduced in the Service Fusion paper.
/** @todo Rename (ServiceFusion?) or remove this poorly named namespace. */
namespace CieMap
{
    class IContainer;
    class IEventManager;
    class IVisualContainer;
    class Layout;
    class ContainerFactory;
    class Container;
    class ScriptManager;
    class EventManager;
    class HttpRequestResponse;
    class IScript;
    class IHttpRequestService;
    class HttpRequest;
    struct Tag;
}
