// For conditions of distribution and use, see copyright notice in LICENSE

#pragma once

#include "IComponent.h"
#include "VisualContainer.h"

class EC_VisualContainer : public IComponent
{
    Q_OBJECT
    COMPONENT_NAME("EC_VisualContainer", 601)
    Q_PROPERTY(VisualContainer* vc READ GetVisualContainer WRITE SetVisualContainer);

public:
    /// Do not directly allocate new components using operator new, but use the factory-based SceneAPI::CreateComponent functions instead.
    explicit EC_VisualContainer(Scene* scene);
    virtual ~EC_VisualContainer();

    VisualContainer *GetVisualContainer() const;
    void SetVisualContainer(VisualContainer *vc);

private slots:
    /// Remove visual container from the component if its beeing destoyed.
    void HandleVisualContainerDestroyed(QObject *obj);

private:
    VisualContainer *container;
};